#include <navlearn_localization_eval/localization_metrics.hpp>

using namespace std::placeholders;

namespace navlearn_localization_metrics
{

static inline double wrap_to_pi(double a)
{
    constexpr double kPi = 3.14159265358979323846;
    while (a >  kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

LocalizationMetrics::LocalizationMetrics(const std::string &name) : Node(name)
    , header_written_(false)
    , is_active_(false)
    , is_first_pose_(true)
    , is_first_tf_(true)
    , is_first_scan_(true)
    , is_first_odom_(true)
    , is_first_gt_(true)
    , error_x_threshold_(0.2)
    , error_y_threshold_(0.2)
    , error_yaw_threshold_(0.1)
    , ttc_state_(TtcState::IDLE)
    , ttc_done_(false)
    , ttr_state_(TtrState::IDLE)
    , ttr_done_(false)
    , ttr_attempted_(false)
{
    int timer_period_ = this->declare_parameter<int>("timer_period", 100);

    csv_path_ = this->declare_parameter<std::string>("csv_path", "localization_evaluation.csv");
    scan_child_frame_ = this->declare_parameter<std::string>("scan_child_frame", "laser_link");
    odom_parent_frame_ = this->declare_parameter<std::string>("odom_parent_frame", "odom");
    odom_child_frame_ = this->declare_parameter<std::string>("odom_child_frame", "base_footprint");
    tf_parent_frame_ = this->declare_parameter<std::string>("tf_parent_frame", "map");
    tf_child_frame_ = this->declare_parameter<std::string>("tf_child_frame", "odom");
    bad_init_test_ = this->declare_parameter<bool>("bad_init_test", false);
    kidnap_test_ = this->declare_parameter<bool>("kidnap_test", false);

    ttc_hold_sec_ = this->declare_parameter<double>("ttc_hold_sec", 2.0);
    ttc_timeout_sec_ = this->declare_parameter<double>("ttc_timeout_sec", 10.0);
    ttr_hold_sec_ = this->declare_parameter<double>("ttr_hold_sec", 2.0);
    ttr_timeout_sec_ = this->declare_parameter<double>("ttr_timeout_sec", 15.0);
    ttr_error_pos_threshold_m_ = this->declare_parameter<double>("ttr_error_pos_threshold_m", 0.20);
    ttr_error_yaw_threshold_rad_ = this->declare_parameter<double>("ttr_error_yaw_threshold_rad", 0.10);

    episode_topic = this->declare_parameter<std::string>("episode_topic", "/navlearn/episode_event");
    amcl_topic_ = this->declare_parameter<std::string>("amcl_topic", "/amcl_pose");
    tf_topic_ = this->declare_parameter<std::string>("tf_topic", "/tf");
    ground_truth_tf_topic_ = this->declare_parameter<std::string>("ground_truth_tf_topic", "/bumperbot/ground_truth_pose");
    initial_pose_topic = this->declare_parameter<std::string>("initial_pose_topic", "/navlearn/bad_initialpose_event");
    kidnap_topic_ = this->declare_parameter<std::string>("kidnap_topic", "/navlearn/kidnap_event");
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/bumperbot_controller/odom");

    ttc_arm_for_next_goal_ = false;
    last_initialpose_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    if(bad_init_test_ && kidnap_test_){
        RCLCPP_FATAL(get_logger(), "Cannot have bad initial pose and kidnap robot conditions true at the same time");
        throw std::runtime_error("bad_init_test and kidnap_test both true");
    }
    
    // Check Parent Directory Exists
    try {
        std::filesystem::path p(csv_path_);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }
    } catch (...) {}

    // Open csv file with append
    csv_.open(csv_path_, std::ios::app);
    if (!csv_.is_open()) {
        RCLCPP_FATAL(get_logger(), "Failed to open CSV file: %s", csv_path_.c_str());
        throw std::runtime_error("cannot open metrics CSV");
    }
    if (csv_.tellp() > 0) header_written_ = true;       // Avoiding rewriting of header

    RCLCPP_INFO(get_logger(), "AMCLEval writing to %s", csv_path_.c_str());

    episode_event_sub_ = create_subscription<navlearn_msgs::msg::EpisodeEvent>(episode_topic, 10,
        std::bind(&LocalizationMetrics::onEpisodeEvent, this, _1));
    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(amcl_topic_, 10,
        std::bind(&LocalizationMetrics::onAMCLPose, this, _1));
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(tf_topic_, 10,
        std::bind(&LocalizationMetrics::onTF, this, _1));
    ground_truth_tf_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(ground_truth_tf_topic_, 10,
        std::bind(&LocalizationMetrics::onGroundTruthTF, this, _1));
    if(bad_init_test_)
    {
        initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(initial_pose_topic, 10,
        std::bind(&LocalizationMetrics::onInitialPose, this, _1));
    }
    if(kidnap_test_)
    {
        kidnap_event_sub_ = create_subscription<navlearn_msgs::msg::KidnapEvent>(kidnap_topic_, 10,
            std::bind(&LocalizationMetrics::onKidnapEvent, this, _1));
    }
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_, 10,
        std::bind(&LocalizationMetrics::onScan, this, _1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 10,
        std::bind(&LocalizationMetrics::onOdometry, this, _1));
    
    timer_ = create_wall_timer(std::chrono::milliseconds(timer_period_), std::bind(&LocalizationMetrics::timerCallback, this));
}

// Miscellaneous Helpers
double LocalizationMetrics::getYaw(const geometry_msgs::msg::Quaternion &q)
{
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

std::string LocalizationMetrics::uuid_to_string(const unique_identifier_msgs::msg::UUID & id)
{
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (uint8_t b : id.uuid) {
        oss << std::setw(2) << static_cast<int>(b);
    }
    return oss.str();
}

void LocalizationMetrics::trim(std::deque<rclcpp::Time>& q, const rclcpp::Time &now, double max_buffer)
{
    while(!q.empty() && (now - q.front()).seconds() > max_buffer){ q.pop_front();}
}

void LocalizationMetrics::write_csv(std::string_view metric_class, std::string_view metric_source, 
    const boost::unordered_map<std::string, double> &metrics, const rclcpp::Time &t)
{
    if(!header_written_)
    {
        csv_ << "GoalID,Metric Class,Metric Source,Metric Name,Metric Value,Timestamp\n";
        header_written_ = true;
    }

    for (const auto& metric : metrics)
    {
        csv_ << goal_id_ << ","
             << metric_class << ","
             << metric_source << ","
             << metric.first << ","
             << metric.second << ","
             << t.seconds() << "\n";
    }
    
    csv_.flush();
}

void LocalizationMetrics::reset_episode_state()
{
  const auto ct = get_clock()->get_clock_type();

  // Mission outcome
  goal_result_ = 0;
  nav_time_ = 0.0;
  collision_count_ = 0;
  recoveries_count_ = 0;
  mission_outcome_.clear();

  // Localization Quality
  is_first_pose_ = true;
  prev_pose_ = PoseStatistics{};
  curr_pose_ = PoseStatistics{};
  pose_monotonicity_violations_ = 0;

  // Localization Stability (TF)
  is_first_tf_ = true;
  prev_tf_ = TFStatistics{};
  curr_tf_ = TFStatistics{};
  tf_monotonicity_violations_ = 0;
  tf_frame_mismatch_count_ = 0;
  tf_gap_count_ = 0;
  tf_stamp_.clear();

  // Ground truth queue
  is_first_gt_ = true;
  ground_truth_statistic_ = TFStatistics{};
  gt_matched_ = TFStatistics{};
  gt_stats_.clear();

  // TTC
  reset_ttc_state();

  // TTR
  reset_ttr_state();

  // Lidar
  is_first_scan_ = true;
  prev_scan_ = rclcpp::Time(0, 0, ct);
  curr_scan_ = rclcpp::Time(0, 0, ct);
  scan_monotonicity_violations_ = 0;
  scan_frame_mismatch_count_ = 0;
  scan_stamp_.clear();

  // Odom
  is_first_odom_ = true;
  prev_odom_ = rclcpp::Time(0, 0, ct);
  curr_odom_ = rclcpp::Time(0, 0, ct);
  odom_monotonicity_violations_ = 0;
  odom_frame_mismatch_count_ = 0;
  odom_stamp_.clear();
}

void LocalizationMetrics::reset_ttc_state()
{
  const auto ct = get_clock()->get_clock_type();
  ttc_start_time_     = rclcpp::Time(0, 0, ct);
  ttc_converged_time_ = rclcpp::Time(0, 0, ct);
  ttc_complete_time_  = rclcpp::Time(0, 0, ct);

  TTC = -1.0;
  ttc_outcome_ = 255;          // 255 = unknown/not yet determined

  ttc_state_ = TtcState::IDLE;
  ttc_done_ = false;
}

void LocalizationMetrics::reset_ttr_state()
{
    const auto ct = get_clock()->get_clock_type();
    ttr_start_time_ = rclcpp::Time(0, 0, ct);
    ttr_recovered_time_ = rclcpp::Time(0, 0, ct);
    ttr_complete_time_ = rclcpp::Time(0, 0, ct);

    TTR = -1.0;
    ttr_outcome_ = 255;

    ttr_state_ = TtrState::IDLE;
    ttr_done_ = false;
    ttr_attempted_ = false;

    kidnap_distance_ = -1.0;
    kidnap_angle_ = -1.0;

    ttr_stats_ = TTRStatistics{};
}

// Mission Outcome Helpers

void LocalizationMetrics::onEpisodeEvent(const navlearn_msgs::msg::EpisodeEvent &episode_event)
{
    if(episode_event.state == navlearn_msgs::msg::EpisodeEvent::START){
        const rclcpp::Time t(episode_event.header.stamp, get_clock()->get_clock_type());

        reset_episode_state();
        is_active_ = true;
        goal_id_ = uuid_to_string(episode_event.goal_id);

        reset_ttc_state();       // ensure clean TTC for this goal
        reset_ttr_state();

        if (bad_init_test_)
        {
            if (ttc_arm_for_next_goal_) {
                ttc_state_ = TtcState::MEASURING;
                ttc_start_time_ = t;
                ttc_arm_for_next_goal_ = false;
                RCLCPP_INFO(get_logger(), "TTC: ARMED -> MEASURING at t=%.3f", t.seconds());
            } else {
                RCLCPP_WARN(get_logger(), "Goal started but TTC was NOT armed (no initialpose between goals).");
            }
        }

        ttr_stats_.goal_start_x_ = episode_event.start_pose.pose.position.x;
        ttr_stats_.goal_start_y_ = episode_event.start_pose.pose.position.y;
        ttr_stats_.goal_start_yaw_ = getYaw(episode_event.start_pose.pose.orientation);
        ttr_stats_.have_goal_start_pose_ = true;
        
        return;
    } else if(episode_event.state == navlearn_msgs::msg::EpisodeEvent::END)
    {
        const rclcpp::Time t(episode_event.header.stamp, get_clock()->get_clock_type());
        is_active_ = false;
        goal_result_ = episode_event.result;
        nav_time_ = static_cast<double>(episode_event.nav_time.sec) + 1e-9 * static_cast<double>(episode_event.nav_time.nanosec);
        collision_count_ = 0;       // Placeholder for now
        recoveries_count_ = 0;      // Placeholder for now

        if(bad_init_test_)
        {
            double ttc_sec = -1.0;

            if (ttc_done_ && ttc_state_ != TtcState::TIMED_OUT) {
                // Converge + hold completed successfully
                ttc_sec = (ttc_complete_time_ - ttc_start_time_).seconds();
                ttc_outcome_ = 1;   // success
            } else if (ttc_state_ == TtcState::TIMED_OUT) {
                ttc_outcome_ = 0;   // timeout
            } else if (ttc_state_ == TtcState::MEASURING ||
                       ttc_state_ == TtcState::CONVERGED) {
                ttc_outcome_ = 2;   // ended early / not converged
            } else {
                ttc_outcome_ = 3;   // not armed / not started
            }

            write_csv("Localization Responsiveness", "TTC",
                boost::unordered_map<std::string, double>{
                    {"TTC", ttc_sec},
                    {"TTC Outcome", static_cast<double>(ttc_outcome_)}
                }, t);
        }

        if(kidnap_test_)
        {
            double ttr_sec_ = -1.0;

            if(ttr_outcome_ == 4) {
                ttr_sec_ = -1.0;
            } else if(ttr_done_ && ttr_state_ != TtrState::TIMED_OUT) {
                // Recovered + hold completed
                ttr_sec_ = (ttr_complete_time_ - ttr_start_time_).seconds();
                ttr_outcome_ = 1;
            } else if(ttr_state_ == TtrState::TIMED_OUT) {
                ttr_outcome_ = 0;
            } else if(ttr_state_ == TtrState::KIDNAPPED ||
                      ttr_state_ == TtrState::RECOVERING ||
                      ttr_state_ == TtrState::RECOVERED) {
                ttr_outcome_ = 2;
            } else {
                ttr_outcome_ = 3;
            }

            write_csv("Localization Recovery", "TTR",
                boost::unordered_map<std::string, double>{
                    {"TTR", ttr_sec_},
                    {"TTR Outcome", static_cast<double>(ttr_outcome_)},
                    {"Kidnap Distance", kidnap_distance_},
                    {"Kidnap Angle", kidnap_angle_}
                }, t);
        }

        write_csv("Mission Outcome", "Episode Event",boost::unordered_map<std::string, double> {
            {"Goal Result", goal_result_},
            {"Navigation Time", nav_time_},
            {"Collisions", collision_count_},
            {"Recoveries", recoveries_count_}
        }, t);

        write_csv("Input Validity", "AMCLPose", boost::unordered_map<std::string, double> {
            {"Pose Monotonicity", pose_monotonicity_violations_}
        }, t);

        write_csv("Input Validity", "Lidar", boost::unordered_map<std::string, double> {
            {"Lidar Monotonicity", scan_monotonicity_violations_},
            {"Lidar Frame Mistmatch", scan_frame_mismatch_count_}
        }, t);

        write_csv("Input Validity", "Odometry", boost::unordered_map<std::string, double> {
            {"Odometry Monotonicity", odom_monotonicity_violations_},
            {"Odometry Frame Mismatch", odom_frame_mismatch_count_}
        }, t);

        write_csv("Input Validity", "TF", boost::unordered_map<std::string, double> {
            {"TF Monotonicity", tf_monotonicity_violations_},
            {"TF Frame Drop", tf_frame_mismatch_count_},
            {"TF Gap Count", tf_gap_count_}
        }, t);
        
        goal_id_ = "Robot Idle";
        reset_episode_state();
    }
}

// Localization Quality Helpers

LocalizationMetrics::PoseStatistics LocalizationMetrics::fillPoseStatistics(const geometry_msgs::msg::PoseWithCovarianceStamped &pose, const rclcpp::Time &t)
{
    LocalizationMetrics::PoseStatistics out;
    out.t = t;
    out.mean_x = pose.pose.pose.position.x;
    out.mean_y = pose.pose.pose.position.y;
    out.mean_yaw = getYaw(pose.pose.pose.orientation);
    out.cov_xx = pose.pose.covariance[0];
    out.cov_yy = pose.pose.covariance[7];
    out.cov_yawyaw = pose.pose.covariance[35];

    return out;
}

void LocalizationMetrics::onAMCLPose(const geometry_msgs::msg::PoseWithCovarianceStamped &amcl_pose)
{
    if(!is_active_) return;

    const rclcpp::Time t(amcl_pose.header.stamp, get_clock()->get_clock_type());

    if(is_first_pose_)
    {
        is_first_pose_ = false;
        curr_pose_ = fillPoseStatistics(amcl_pose, t);
        return;
    }

    if((t <= curr_pose_.t)){
        pose_monotonicity_violations_++;
        return;
    }

    prev_pose_ = curr_pose_;
    curr_pose_ = fillPoseStatistics(amcl_pose, t);
    write_csv("Localization Quality", "AMCLPose", boost::unordered_map<std::string, double> {
            {"MeanX", curr_pose_.mean_x},
            {"MeanY", curr_pose_.mean_y},
            {"MeanYaw", curr_pose_.mean_yaw},
            {"CovXX", curr_pose_.cov_xx},
            {"CovYY", curr_pose_.cov_yy},
            {"CovYawYaw", curr_pose_.cov_yawyaw}
        }, t);

    if (bad_init_test_)
    {
        // Only process TTC while actively measuring and not yet concluded
        if (ttc_state_ != TtcState::MEASURING && ttc_state_ != TtcState::CONVERGED) return;
        if (ttc_done_) return;

        if(ttc_state_ == TtcState::MEASURING)
        {
            if(!linear_interpolate(gt_stats_, gt_matched_, t, 0.30 , 0.30)) return;
            double pose_error_x = gt_matched_.transform_x - curr_pose_.mean_x;
            double pose_error_y = gt_matched_.transform_y - curr_pose_.mean_y;
            double pose_error_yaw = wrap_to_pi(gt_matched_.transform_yaw - curr_pose_.mean_yaw);

            RCLCPP_INFO(get_logger(), "TTC error: ErrorX: %f || ErrorY: %f || ErrorYaw: %f", pose_error_x, pose_error_y, pose_error_yaw);

            if(std::fabs(pose_error_x) < error_x_threshold_ && std::fabs(pose_error_y) < error_y_threshold_ && std::fabs(pose_error_yaw) < error_yaw_threshold_)
            {
                ttc_state_ = TtcState::CONVERGED;
                ttc_converged_time_ = t;
                RCLCPP_INFO(get_logger(), "TTC: MEASURING -> CONVERGED at t=%.3f", t.seconds());
                return;
            }
        }

        if(ttc_state_ == TtcState::CONVERGED)
        {
            if((t - ttc_converged_time_).seconds() >= ttc_hold_sec_)
            {
                ttc_done_ = true;
                ttc_complete_time_ = t;
                RCLCPP_INFO(get_logger(), "TTC: CONVERGED -> hold complete (done) at t=%.3f", t.seconds());
            }
        }

        return;
    }

    if(!kidnap_test_) return;
    // Only process TTR while in an active measuring state and not yet concluded
    if(ttr_state_ != TtrState::KIDNAPPED &&
       ttr_state_ != TtrState::RECOVERING &&
       ttr_state_ != TtrState::RECOVERED) return;
    if(ttr_done_) return;
    if(ttr_outcome_ == 4) return;

    if(!linear_interpolate(gt_stats_, gt_matched_, t, 0.30, 0.30)) return;

    const double err_x = gt_matched_.transform_x - curr_pose_.mean_x;
    const double err_y = gt_matched_.transform_y - curr_pose_.mean_y;
    const double err_pos = std::hypot(err_x, err_y);
    const double err_yaw = std::fabs(wrap_to_pi(gt_matched_.transform_yaw - curr_pose_.mean_yaw));

    const bool ok =
        (std::fabs(err_x) < ttr_error_pos_threshold_m_) &&
        (std::fabs(err_y) < ttr_error_pos_threshold_m_) &&
        (std::fabs(err_pos) < ttr_error_pos_threshold_m_) &&
        (std::fabs(err_yaw) < ttr_error_yaw_threshold_rad_);

    if(ttr_state_ != TtrState::RECOVERED && ok)
    {
        ttr_state_ = TtrState::RECOVERED;
        ttr_recovered_time_ = t;
        RCLCPP_INFO(get_logger(), "TTR: RECOVERING -> RECOVERED at t=%.3f", t.seconds());
        return;
    }

    if(ttr_state_ == TtrState::RECOVERED && !ok)
    {
        ttr_state_ = TtrState::RECOVERING;
        ttr_recovered_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        RCLCPP_INFO(get_logger(), "TTR: RECOVERED -> RECOVERING (threshold broken) at t=%.3f", t.seconds());
        return;
    }

    if(ttr_state_ == TtrState::RECOVERED)
    {
        if((t - ttr_recovered_time_).seconds() >= ttr_hold_sec_)
        {
            ttr_done_ = true;
            ttr_complete_time_ = t;
            RCLCPP_INFO(get_logger(), "TTR: RECOVERED -> hold complete (done) at t=%.3f", t.seconds());
        }
    }
}

// Localization Stability Helpers

LocalizationMetrics::TFStatistics LocalizationMetrics::fillTFStatistics(const geometry_msgs::msg::TransformStamped &transform, const rclcpp::Time &t)
{
    LocalizationMetrics::TFStatistics out;
    out.t = t;
    out.transform_x = transform.transform.translation.x;
    out.transform_y = transform.transform.translation.y;
    out.transform_yaw = getYaw(transform.transform.rotation);

    return out;
}

boost::unordered_map<std::string, double> LocalizationMetrics::getLocalizationStability(const TFStatistics prev, const TFStatistics curr)
{
    const double jump_lin = std::sqrt(std::pow((curr.transform_x - prev.transform_x), 2) + std::pow((curr.transform_y - prev.transform_y), 2));
    const double jump_ang = wrap_to_pi(curr.transform_yaw - prev.transform_yaw);

    return {
        {"Jump Linear", jump_lin},
        {"Jump Angular", jump_ang}
    };
}


void LocalizationMetrics::onTF(const tf2_msgs::msg::TFMessage::SharedPtr transforms)
{
    if(!is_active_) return;

    for(const auto& transform : transforms->transforms)
    {
        if(transform.header.frame_id == tf_parent_frame_ && transform.child_frame_id == tf_child_frame_)
        {
            const rclcpp::Time t(transform.header.stamp, get_clock()->get_clock_type());

            if(is_first_tf_)
            {
                is_first_tf_ = false;
                curr_tf_ = fillTFStatistics(transform, t);
                tf_stamp_.push_back(curr_tf_.t);
                trim(tf_stamp_, t, 2.0);
                return;
            }

            if(t <= curr_tf_.t)
            {
                tf_monotonicity_violations_++;
                return;
            }

            prev_tf_ = curr_tf_;
            curr_tf_ = fillTFStatistics(transform, t);
            tf_stamp_.push_back(curr_tf_.t);
            trim(tf_stamp_, t, 2.0);

            const double gap = (curr_tf_.t - prev_tf_.t).seconds();
            if(gap >= 0.2){
                tf_gap_count_++;
            }
            write_csv("Localization Stability", "TF", getLocalizationStability(prev_tf_, curr_tf_), t);
        } else if(transform.header.frame_id == tf_parent_frame_ && transform.child_frame_id != tf_child_frame_)
        {
            tf_frame_mismatch_count_++;
            return;
        }
        continue;
    }
}

// Localization Responsiveness Helpers

bool LocalizationMetrics::linear_interpolate(const std::deque<TFStatistics>& q, TFStatistics &out, rclcpp::Time t_req, double max_gap_s, double max_stale_s)
{
    const size_t n = q.size();
    if (n == 0) return false;

    if (n >= 2) {
        const double dt_latest_s = (q.back().t - q[n - 2].t).seconds();
        if (dt_latest_s > 0.0 && dt_latest_s <= max_gap_s) {
            t_req = t_req - rclcpp::Duration::from_seconds(0.5 * dt_latest_s);
        }
    }

    const rclcpp::Duration max_gap = rclcpp::Duration::from_seconds(max_gap_s);

    for (size_t i = n; i-- > 0; ) {
        if (q[i].t > t_req) continue;

        const double stale_s = (t_req - q[i].t).seconds();
        if (stale_s >= max_stale_s) return false;

        if (i + 1 < n)
        {
            const rclcpp::Duration dt = q[i + 1].t - q[i].t;
            if (dt <= rclcpp::Duration::from_nanoseconds(0) || dt > max_gap)
            {
                out = q[i];
                return true;
            }

            const double dt_s = dt.seconds();
            const double alpha = (t_req - q[i].t).seconds() / dt_s;

            const int64_t dt_ns = dt.nanoseconds();
            const int64_t interp_ns = static_cast<int64_t>(std::llround(alpha * static_cast<double>(dt_ns)));
            out.t = q[i].t + rclcpp::Duration::from_nanoseconds(interp_ns);
            out.transform_x = q[i].transform_x + alpha * (q[i + 1].transform_x - q[i].transform_x);
            out.transform_y = q[i].transform_y + alpha * (q[i + 1].transform_y - q[i].transform_y);

            const double dyaw = wrap_to_pi(q[i + 1].transform_yaw - q[i].transform_yaw);
            out.transform_yaw = wrap_to_pi(q[i].transform_yaw + alpha * dyaw);

            return true;
        }

        out = q[i];
        return true;
    }

    return false;
}

void LocalizationMetrics::trim(std::deque<TFStatistics>& q, const rclcpp::Time& now, double max_buffer)
{
  while (!q.empty() && (now - q.front().t).seconds() > max_buffer) {q.pop_front();}
}

void LocalizationMetrics::onGroundTruthTF(const geometry_msgs::msg::TransformStamped gt_transform)
{
    if(!is_active_) return;

    const rclcpp::Time t(gt_transform.header.stamp, get_clock()->get_clock_type());
    ground_truth_statistic_ = fillTFStatistics(gt_transform, t);
    gt_stats_.push_back(ground_truth_statistic_);
    trim(gt_stats_, get_clock()->now(), 2.0);    
}

void LocalizationMetrics::onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &initial_pose)
{
    if(!bad_init_test_) return;
    if(kidnap_test_) return;
    if(is_active_) return;

    const rclcpp::Time t(initial_pose.header.stamp, get_clock()->get_clock_type());
    ttc_arm_for_next_goal_ = true;
    last_initialpose_time_ = t;
    ttc_state_ = TtcState::ARMED;

    RCLCPP_INFO(get_logger(), "TTC: IDLE -> ARMED (initialpose t=%.3f)", t.seconds());
}

void LocalizationMetrics::onKidnapEvent(const navlearn_msgs::msg::KidnapEvent &kidnap_event)
{
    if(!is_active_) return;
    if(!kidnap_test_) return;
    if(uuid_to_string(kidnap_event.goal_id) != goal_id_) return;
    if(ttr_attempted_) return;
    if(!ttr_stats_.have_goal_start_pose_) return;

    const rclcpp::Time t(kidnap_event.header.stamp, get_clock()->get_clock_type());
    ttr_attempted_ = true;

    if(kidnap_event.success)
    {
        ttr_state_ = TtrState::KIDNAPPED;
        ttr_start_time_ = t;

        const double dx = ttr_stats_.goal_start_x_ - kidnap_event.kidnap_pose.position.x;
        const double dy = ttr_stats_.goal_start_y_ - kidnap_event.kidnap_pose.position.y;
        kidnap_distance_ = std::hypot(dx, dy);
        kidnap_angle_ = std::fabs(wrap_to_pi(ttr_stats_.goal_start_yaw_ - getYaw(kidnap_event.kidnap_pose.orientation)));

        RCLCPP_INFO(get_logger(), "TTR: IDLE -> KIDNAPPED at t=%.3f (dist=%.2f m, angle=%.2f rad)",
            t.seconds(), kidnap_distance_, kidnap_angle_);

        // Immediately advance to RECOVERING — robot is actively recovering from this point
        ttr_state_ = TtrState::RECOVERING;
        RCLCPP_INFO(get_logger(), "TTR: KIDNAPPED -> RECOVERING at t=%.3f", t.seconds());
    } else {
        ttr_outcome_ = 4;
        TTR = -1.0;
        RCLCPP_WARN(get_logger(), "Kidnap event reported failure (success=false).");
    }
}

// Input Validity Helpers

void LocalizationMetrics::onScan(const sensor_msgs::msg::LaserScan &scan)
{
    if(!is_active_) return;

    const rclcpp::Time t(scan.header.stamp, get_clock()->get_clock_type());

    scan_stamp_.push_back(t);
    trim(scan_stamp_, t, 2.0);

    if(is_first_scan_)
    {
        is_first_scan_ = false;
        curr_scan_ = t;
        return;
    }

    if(scan.header.frame_id != scan_child_frame_){
        scan_frame_mismatch_count_++;
        return;
    }

    if(t <= curr_scan_)
    {
        scan_monotonicity_violations_++;
        return;
    }

    prev_scan_ = curr_scan_;
    curr_scan_ = t;
}

void LocalizationMetrics::onOdometry(const nav_msgs::msg::Odometry &odom)
{
    if(!is_active_) return;

    const rclcpp::Time t(odom.header.stamp, get_clock()->get_clock_type());

    odom_stamp_.push_back(t);
    trim(odom_stamp_, t, 2.0);

    if(is_first_odom_)
    {
        is_first_odom_ = false;
        curr_odom_ = t;
        return;
    }

    if(odom.child_frame_id != odom_child_frame_ || odom.header.frame_id != odom_parent_frame_)
    {
        odom_frame_mismatch_count_++;
        return;
    }

    if(t <= curr_odom_)
    {
        odom_monotonicity_violations_++;
        return;
    }

    prev_odom_ = curr_odom_;
    curr_odom_ = t;
}

void LocalizationMetrics::timerCallback()
{
    if(!is_active_) return;
    if (scan_stamp_.size() < 2 || odom_stamp_.size() < 2 || tf_stamp_.size() < 2) return;

    const rclcpp::Time t = get_clock()->now();

    const auto scan_dt = (scan_stamp_.back() - scan_stamp_.front()).seconds();
    const auto odom_dt = (odom_stamp_.back() - odom_stamp_.front()).seconds();
    const auto tf_dt = (tf_stamp_.back() - tf_stamp_.front()).seconds();

    if(scan_dt <= 0.0 || odom_dt <= 0.0 || tf_dt <= 0.0) return;

    const double lidar_rate = (scan_stamp_.size() - 1) / scan_dt;
    const double odom_rate = (odom_stamp_.size() - 1) / odom_dt;
    const double tf_rate = (tf_stamp_.size() - 1) / tf_dt;

    if((ttc_state_ == TtcState::MEASURING || ttc_state_ == TtcState::CONVERGED) && !ttc_done_)
    {
        if((t - ttc_start_time_).seconds() > ttc_timeout_sec_)
        {
            RCLCPP_WARN(get_logger(), "TTC: MEASURING -> TIMED_OUT elapsed=%.3f", (t - ttc_start_time_).seconds());
            ttc_state_ = TtcState::TIMED_OUT;
            ttc_done_ = true;
            ttc_outcome_ = 0;
        }
    }

    if(kidnap_test_ &&
       (ttr_state_ == TtrState::RECOVERING || ttr_state_ == TtrState::RECOVERED ||
        ttr_state_ == TtrState::KIDNAPPED) &&
       !ttr_done_ && ttr_outcome_ != 4)
    {
        if((t - ttr_start_time_).seconds() > ttr_timeout_sec_)
        {
            RCLCPP_WARN(get_logger(), "TTR: RECOVERING -> TIMED_OUT elapsed=%.3f", (t - ttr_start_time_).seconds());
            ttr_state_ = TtrState::TIMED_OUT;
            ttr_done_ = true;
            ttr_outcome_ = 0;
        }
    }

    write_csv("Input Validity", "Lidar", boost::unordered_map<std::string, double> {
    {"Lidar Rate", lidar_rate}
    }, t);

    write_csv("Input Validity", "Odometry", boost::unordered_map<std::string, double> {
    {"Odometry Rate", odom_rate}
    }, t);

    write_csv("Input Validity", "TF", boost::unordered_map<std::string, double> {
    {"TF Rate", tf_rate}
    }, t);
}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navlearn_localization_metrics::LocalizationMetrics>("localization_metrics");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}