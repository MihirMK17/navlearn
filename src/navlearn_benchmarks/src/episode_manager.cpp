// Copyright 2026 NavLearn Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// episode_manager.cpp

#include <chrono>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
#include <limits>

#include "navlearn_benchmarks/episode_manager.hpp"
#include "navlearn_benchmarks/navlearn_utils.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace navlearn_benchmarks{

EpisodeManager::EpisodeManager(const rclcpp::NodeOptions & options)
  // Initializer order follows declaration order in the header (-Wreorder). None of
  // these members depend on another, so the order is a compiler-warning contract only.
  : rclcpp::Node("episode_manager", options)
  , reinit_in_flight_(false)
  , last_reinit_time_(0,0,RCL_ROS_TIME)
  , is_first_bad_init_(false)
  , have_amcl_pose_(false)
  , kidnap_stage_(KidnapStage::IDLE)
  , kidnap_sampled_delay_sec_(0.0)
  , have_start_gt_(false)
  , kidnap_target_pose_(unsampledKidnapPose_())
  , have_kidnap_reference_(false)
  , idx_(0)
  , active_(false)
  , goal_request_inflight_(false)
  , stamp_received_(0,0,RCL_ROS_TIME)
  , qos_profile_sub(10)
  , pose_sequence_pending_(false)
  , pose_seq_stage_(PoseSeqStage::IDLE)
{
  dwell_sec_ = this->declare_parameter<double>("dwell_sec", 5.0);
  goal_source_ = this->declare_parameter<std::string>("goal_source", "map_random");
  // One seed for the whole campaign; every draw is derived from it plus the run and goal
  // indices. run_index must be distinct per episode or episodes repeat conditions — the
  // defect this replaces. The harness supplies it.
  campaign_seed_ = this->declare_parameter<int64_t>("campaign_seed", 42);
  run_index_     = this->declare_parameter<int64_t>("run_index", 0);

  // The retired seeds. They are still declared so that a stale config or script setting
  // them fails loudly instead of being silently ignored — silent acceptance is exactly
  // how initial_pose_seed sat at 1337 for an entire campaign without anyone noticing that
  // every episode was drawing the same five perturbations.
  for (const char * retired : {"goal_seed", "initial_pose_seed", "kidnap_seed"}) {
    if (this->declare_parameter<int>(retired, -1) != -1) {
      RCLCPP_FATAL(get_logger(),
        "Parameter '%s' is retired and no longer has any effect. It combined a fixed "
        "constant with the goal index by addition and ignored the run index, so every "
        "episode in a cell drew the same perturbation. Use 'campaign_seed' (one value for "
        "the whole campaign) and 'run_index' (distinct per episode) instead.", retired);
      throw std::runtime_error(std::string("retired parameter set: ") + retired);
    }
  }
  goal_poses_x_ = this->declare_parameter<std::vector<double>>("goal_poses_x", {});
  goal_poses_y_ = this->declare_parameter<std::vector<double>>("goal_poses_y", {});
  goal_poses_yaw_ = this->declare_parameter<std::vector<double>>("goal_poses_yaw", {});
  goals_num_ = this->declare_parameter<int>("goals_num", 4);
  action_server_ = this->declare_parameter<std::string>("action_server", "/navigate_to_pose");
  episode_pub_topic_ = this->declare_parameter<std::string>("episode_pub_topic", "/navlearn/episode_event");
  fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "map");
  robot_frame_ = this->declare_parameter<std::string>("robot_frame", "base_link");

  goal_min_clearance_m_ = declare_parameter<double>("goal_min_clearance_m", 0.75);
  goal_occ_thresh_      = declare_parameter<int>("goal_occ_thresh", 50);
  goal_reject_unknown_  = declare_parameter<bool>("goal_reject_unknown", true);
  goal_min_distance_m_  = declare_parameter<double>("goal_min_distance_m", 0.0);
  have_spawn_pose_      = false;

  bad_init_test_ = declare_parameter<bool>("bad_init_test", false);
  bad_init_pose_topic_ = declare_parameter<std::string>("bad_init_pose_topic", "/navlearn/bad_initialpose_event");
  set_initial_pose_srv_ = this->declare_parameter<std::string>("set_initial_pose_srv", "/set_initial_pose");

  est_error_frame_ = declare_parameter<std::string>("est_error_frame", "base_footprint");
  gt_error_frame_  = declare_parameter<std::string>("gt_error_frame",  "base_footprint_gt");

  amcl_pose_topic_ = declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
  // Convergence criterion for the terminal record: trace of the position covariance
  // (xx + yy). 0.25 m^2 corresponds to roughly a 0.35 m standard deviation per axis.
  // This is a methodological choice that appears in the paper, so the value used is
  // written into every terminal report rather than left implicit here.
  terminal_converged_cov_threshold_m2_ =
    declare_parameter<double>("terminal_converged_cov_threshold_m2", 0.25);

  end_error_pos_threshold_m_  = declare_parameter<double>("end_error_pos_threshold_m", 0.10);
  end_error_yaw_threshold_rad_ = declare_parameter<double>("end_error_yaw_threshold_rad", 0.10);

  bad_init_lin_range_m_  = declare_parameter<double>("bad_init_lin_range_m", 0.10);
  bad_init_yaw_range_rad_ = declare_parameter<double>("bad_init_yaw_range_rad", 0.20);

  pose_apply_pos_tol_m_  = declare_parameter<double>("pose_apply_pos_tol_m", 0.02);
  pose_apply_yaw_tol_rad_ = declare_parameter<double>("pose_apply_yaw_tol_rad", 0.02);
  pose_apply_timeout_sec_ = declare_parameter<double>("pose_apply_timeout_sec", 2.0);
  pose_sequence_timeout_sec_ = declare_parameter<double>("pose_sequence_timeout_sec", 6.0);

  correct_cov_xy_ = declare_parameter<double>("correct_cov_xy", 1e-6);
  correct_cov_yaw_ = declare_parameter<double>("correct_cov_yaw", 1e-4);
  bad_cov_xy_ = declare_parameter<double>("bad_cov_xy", 0.25);
  bad_cov_yaw_ = declare_parameter<double>("bad_cov_yaw", 0.25);


  set_pose_service_ = this->declare_parameter<std::string>("set_pose_service", "/gz_set_pose_server/set_entity_pose");
  entity_name_ = this->declare_parameter<std::string>("entity_name", "bumperbot");

  kidnap_enabled_ = this->declare_parameter<bool>("kidnap_enabled", false);

  // Whether the teleport is accompanied by an AMCL global reinit. False is the true
  // kidnapped-robot condition: the filter gets no hint and must recover from scans alone.
  // See the callReinitGlobalLocalization call site for why this was previously forced on.
  kidnap_notify_localizer_ =
    this->declare_parameter<bool>("kidnap_notify_localizer", false);

  clear_global_costmap_srv_ = this->declare_parameter<std::string>(
    "clear_global_costmap_srv", "/global_costmap/clear_entirely_global_costmap");
  clear_local_costmap_srv_ = this->declare_parameter<std::string>(
    "clear_local_costmap_srv", "/local_costmap/clear_entirely_local_costmap");
  // Clearing between goals keeps episodes independent. Within-episode costmap corruption
  // is a real phenomenon worth measuring; carrying it into the NEXT episode is
  // contamination, and n=25 per cell assumes the episodes are independent trials.
  clear_costmaps_between_goals_ = this->declare_parameter<bool>(
    "clear_costmaps_between_goals", true);

  // deprecated (kept only so old configs don’t crash). Not used in logic.
  kidnap_delay_sec_ = this->declare_parameter<double>("kidnap_delay_sec", 0.0);

  kidnap_delay_min_sec_ = this->declare_parameter<double>("kidnap_delay_min_sec", 1.0);
  kidnap_delay_max_sec_ = this->declare_parameter<double>("kidnap_delay_max_sec", 5.0);
  kidnap_min_travel_m_  = this->declare_parameter<double>("kidnap_min_travel_m", 0.25);
  kidnap_distance_m_    = this->declare_parameter<double>("kidnap_distance_m", 0.20);
  kidnap_max_distance_m_ = this->declare_parameter<double>("kidnap_max_distance_m", 3.0);

  // Continuous severity. In "curve" mode each goal draws its own displacement from
  // [kidnap_magnitude_min_m, kidnap_magnitude_max_m], so the sweep itself is the
  // experiment and the curve is the reported result rather than a discarded calibration
  // input. "fixed" keeps the banded behaviour, which the ablation legs still need.
  //
  // Default is "fixed" so no existing cell changes meaning by upgrading.
  kidnap_magnitude_mode_ =
    this->declare_parameter<std::string>("kidnap_magnitude_mode", "fixed");
  kidnap_magnitude_min_m_ =
    this->declare_parameter<double>("kidnap_magnitude_min_m", 0.3);
  kidnap_magnitude_max_m_ =
    this->declare_parameter<double>("kidnap_magnitude_max_m", 3.0);
  kidnap_magnitude_scale_ =
    this->declare_parameter<std::string>("kidnap_magnitude_scale", "log");
  // Half-width of the annulus the destination is drawn from, as a fraction of the
  // commanded magnitude. Free space rarely offers a valid pose at exactly one radius, so
  // a thin band is the difference between a sampler that usually succeeds and one that
  // reports "no valid pose" for reasons that are about geometry rather than severity.
  kidnap_magnitude_band_ =
    this->declare_parameter<double>("kidnap_magnitude_band", 0.05);

  if (kidnap_magnitude_mode_ != "fixed" && kidnap_magnitude_mode_ != "curve") {
    RCLCPP_FATAL(get_logger(),
      "kidnap_magnitude_mode must be 'fixed' or 'curve', got '%s'. Refusing to run: a "
      "misspelled mode would silently fall back to a severity the cell did not request.",
      kidnap_magnitude_mode_.c_str());
    throw std::invalid_argument("invalid kidnap_magnitude_mode");
  }

  if (kidnap_magnitude_mode_ == "curve") {
    const auto scale = (kidnap_magnitude_scale_ == "linear")
                         ? navlearn::MagnitudeScale::LINEAR
                         : navlearn::MagnitudeScale::LOG;
    if (kidnap_magnitude_scale_ != "linear" && kidnap_magnitude_scale_ != "log") {
      RCLCPP_FATAL(get_logger(),
        "kidnap_magnitude_scale must be 'log' or 'linear', got '%s'",
        kidnap_magnitude_scale_.c_str());
      throw std::invalid_argument("invalid kidnap_magnitude_scale");
    }
    // Throws on an inverted range or a non-positive log bound; allowed to escape so a
    // misconfigured sweep fails to start rather than running a range nobody asked for.
    magnitude_sampler_ = std::make_unique<navlearn::MagnitudeSampler>(
      kidnap_magnitude_min_m_, kidnap_magnitude_max_m_, scale);

    RCLCPP_INFO(get_logger(),
      "Kidnap severity: CURVE, %s-uniform over [%.3f, %.3f] m, drawn per goal from the "
      "campaign seed; destination sampled in a +/-%.0f%% band around the draw.",
      kidnap_magnitude_scale_.c_str(), kidnap_magnitude_min_m_, kidnap_magnitude_max_m_,
      100.0 * kidnap_magnitude_band_);
  }

  // The TTC leg sweeps the same way, over the bad-initialization displacement. Its own
  // range so a cell can sweep one perturbation without inheriting the other's bounds,
  // and its own seed stream so enabling both cannot correlate them.
  bad_init_magnitude_mode_ =
    this->declare_parameter<std::string>("bad_init_magnitude_mode", "fixed");
  bad_init_magnitude_min_m_ =
    this->declare_parameter<double>("bad_init_magnitude_min_m", 0.05);
  bad_init_magnitude_max_m_ =
    this->declare_parameter<double>("bad_init_magnitude_max_m", 2.0);
  bad_init_magnitude_scale_ =
    this->declare_parameter<std::string>("bad_init_magnitude_scale", "log");

  if (bad_init_magnitude_mode_ != "fixed" && bad_init_magnitude_mode_ != "curve") {
    RCLCPP_FATAL(get_logger(),
      "bad_init_magnitude_mode must be 'fixed' or 'curve', got '%s'",
      bad_init_magnitude_mode_.c_str());
    throw std::invalid_argument("invalid bad_init_magnitude_mode");
  }

  if (bad_init_magnitude_mode_ == "curve") {
    if (bad_init_magnitude_scale_ != "linear" && bad_init_magnitude_scale_ != "log") {
      RCLCPP_FATAL(get_logger(),
        "bad_init_magnitude_scale must be 'log' or 'linear', got '%s'",
        bad_init_magnitude_scale_.c_str());
      throw std::invalid_argument("invalid bad_init_magnitude_scale");
    }
    bad_init_magnitude_sampler_ = std::make_unique<navlearn::MagnitudeSampler>(
      bad_init_magnitude_min_m_, bad_init_magnitude_max_m_,
      (bad_init_magnitude_scale_ == "linear") ? navlearn::MagnitudeScale::LINEAR
                                              : navlearn::MagnitudeScale::LOG);

    RCLCPP_INFO(get_logger(),
      "Bad-init severity: CURVE, %s-uniform over [%.3f, %.3f] m, drawn per goal and "
      "applied at exactly that displacement (polar, not the legacy square draw). "
      "Orientation error stays at its configured +/-%.3f rad, so the sweep moves one "
      "variable.",
      bad_init_magnitude_scale_.c_str(), bad_init_magnitude_min_m_,
      bad_init_magnitude_max_m_, bad_init_yaw_range_rad_);
  }
  kidnap_z_             = this->declare_parameter<double>("kidnap_z", 0.01);
  kidnap_max_sample_tries_ = this->declare_parameter<int>("kidnap_max_sample_tries", 1500);

  kidnap_verify_pos_tol_m_ = this->declare_parameter<double>("kidnap_verify_pos_tol_m", 0.05);
  kidnap_verify_yaw_tol_rad_ = this->declare_parameter<double>("kidnap_verify_yaw_tol_rad", 0.05);
  kidnap_verify_timeout_sec_ = this->declare_parameter<double>("kidnap_verify_timeout_sec", 1.0);

  kidnap_event_topic_ = this->declare_parameter<std::string>("kidnap_event_topic", "/navlearn/kidnap_event");

  recovery_timeout_sec_ = declare_parameter<double>("recovery_timeout_sec", 15.0);
  recovery_stage_ = RecoveryStage::IDLE;

  if(goals_num_ <= 0) 
  {
    RCLCPP_FATAL(get_logger(), "Bad param: goals_num (%d) must be > 0", goals_num_);
    rclcpp::shutdown();
  }

  RCLCPP_INFO(get_logger(), "EpisodeManager config: dwell_sec:%.2f, goal_source:%s, goals_num:%d, action server:%s",
              dwell_sec_, goal_source_.c_str(), goals_num_, action_server_.c_str());

  next_allowed_send_ = this->get_clock()->now();

  declare_parameter<std::string>("reliability", "Reliable");
  declare_parameter<std::string>("durability", "Transient");

  qos_profile_sub.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos_profile_sub.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, action_server_);
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("map", qos_profile_sub,
              std::bind(&EpisodeManager::mapCallback, this, _1));
  clear_global_costmap_client_ =
    create_client<nav2_msgs::srv::ClearEntireCostmap>(clear_global_costmap_srv_);
  clear_local_costmap_client_ =
    create_client<nav2_msgs::srv::ClearEntireCostmap>(clear_local_costmap_srv_);

  amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
              amcl_pose_topic_, qos_profile_sub,
              std::bind(&EpisodeManager::amclPoseCallback, this, _1));
  episode_pub_ = create_publisher<navlearn_msgs::msg::EpisodeEvent>(episode_pub_topic_, rclcpp::QoS(10).reliable());
  kidnap_pub_ = create_publisher<navlearn_msgs::msg::KidnapEvent>(kidnap_event_topic_, rclcpp::QoS(10).reliable());
  timer_ = create_wall_timer(200ms, std::bind(&EpisodeManager::timerCallback, this));

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  set_initial_pose_client_ = create_client<nav2_msgs::srv::SetInitialPose>(set_initial_pose_srv_);
  bad_init_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(bad_init_pose_topic_, 10);

  set_pose_entity_client_ = this->create_client<navlearn_msgs::srv::SetEntityPose>(set_pose_service_);

  reinit_cooldown_ = rclcpp::Duration::from_seconds(10.0);
  last_reinit_time_ = this->get_clock()->now() - reinit_cooldown_;

  reinit_global_loc_client_ = create_client<std_srvs::srv::Empty>("/reinitialize_global_localization");
  if (!reinit_global_loc_client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_WARN(this->get_logger(), "Service /reinitialize_global_localization not available (yet). Will retry on call.");
  }

  have_map_ = false;

  RCLCPP_INFO(get_logger(), "Kidnap client: %s (entity=%s) kidnap_event_topic=%s",
              set_pose_service_.c_str(), entity_name_.c_str(), kidnap_event_topic_.c_str());
  
  if(goal_source_ == "static")
  {
    loadGoals();
  }
}

// ---------- helpers ----------
bool EpisodeManager::lookupPose(const std::string & child_frame, const rclcpp::Time & t, geometry_msgs::msg::PoseStamped & out)
{
  try {
    const auto tr = tf_buffer_->lookupTransform(fixed_frame_, child_frame, t, 150ms);
    out.header = tr.header;
    out.pose.position.x = tr.transform.translation.x;
    out.pose.position.y = tr.transform.translation.y;
    out.pose.position.z = tr.transform.translation.z;
    out.pose.orientation = tr.transform.rotation;
    return true;
  } catch (...) {
    try {
      const auto tr_latest = tf_buffer_->lookupTransform(fixed_frame_, child_frame, rclcpp::Time(0,0,RCL_ROS_TIME), 150ms);
      out.header = tr_latest.header;
      out.pose.position.x = tr_latest.transform.translation.x;
      out.pose.position.y = tr_latest.transform.translation.y;
      out.pose.position.z = tr_latest.transform.translation.z;
      out.pose.orientation = tr_latest.transform.rotation;
      return true;
    } catch (...) {
      out = geometry_msgs::msg::PoseStamped();
      return false;
    }
  }
}

bool EpisodeManager::sampleStartPoseAt(const rclcpp::Time & t, geometry_msgs::msg::PoseStamped & out) {
  return lookupPose(robot_frame_, t, out);
}

bool EpisodeManager::hasClearanceCell(int col, int row, int width, int height, double res,
                const std::vector<int8_t> & data) const 
{
  const int r = static_cast<int>(std::ceil(goal_min_clearance_m_ / res));
  const int r2 = r * r;

  if (col - r < 0 || col + r >= width || row - r < 0 || row + r >= height) {
    return false;
  }

  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (dx*dx + dy*dy > r2) continue;

      const int c = col + dx;
      const int rr = row + dy;
      const int idx = rr * width + c;

      const int v = static_cast<int>(data[idx]);
      if (goal_reject_unknown_ && v < 0) return false;
      if (v >= goal_occ_thresh_) return false;
    }
  }
  return true;
}

bool EpisodeManager::inExclusionZone(double x, double y) const
{
  const bool in_x = (x > -2.4) && (x < 0.7);
  const bool in_y = (y> 3.3) && (y < 5.4);

  return in_x && in_y;
}

void EpisodeManager::loadGoals() {
  if (goal_source_ == "static")
  {
    if (goal_poses_x_.size() != goal_poses_y_.size() ||
        goal_poses_x_.size() != goal_poses_yaw_.size())
    {
      RCLCPP_FATAL(get_logger(), "Goal Poses length mismatch!");
      rclcpp::shutdown();
      return;
    }

    goal_poses_.clear();
    goal_poses_.reserve(goal_poses_x_.size());

    for (std::size_t i = 0; i < goal_poses_x_.size(); ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = fixed_frame_;
      p.header.stamp = this->get_clock()->now();
      p.pose.position.x = goal_poses_x_[i];
      p.pose.position.y = goal_poses_y_[i];
      p.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, goal_poses_yaw_[i] * M_PI / 180.0);
      p.pose.orientation = tf2::toMsg(q);

      goal_poses_.push_back(p);
    }
  }
  else if (goal_source_ == "map_random")
  {
    if (!have_map_) {
      RCLCPP_WARN(get_logger(),
                  "loadGoals(map_random) called before map; will retry when map is available");
      return;
    }

    const auto & info = latest_map_.info;
    const int width   = static_cast<int>(info.width);
    const int height  = static_cast<int>(info.height);
    const double res  = info.resolution;
    const auto & data = latest_map_.data;

    if (width <= 0 || height <= 0 || data.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "loadGoals(map_random): invalid map (width=%d, height=%d, data.size=%zu)",
                   width, height, data.size());
      return;
    }

    goal_poses_.clear();
    goal_poses_.reserve(goals_num_);

    std::uniform_int_distribution<int> dist(0, width * height - 1);

    const int max_tries_per_goal = 1000;

    for (int i = 0; i < goals_num_; ++i) {
      // Seeded per goal rather than once for the sequence, so goal k of this episode is
      // independent of how many goals preceded it and of every other episode. Derived
      // without reference to the controller, so all arms navigate to identical goals.
      std::mt19937_64 gen(seedFor(navlearn::seed::Stream::GOAL_POSITION,
                                  static_cast<uint64_t>(i)));
      int cell_index = -1;

      for (int attempt = 0; attempt < max_tries_per_goal; ++attempt) {
        int idx = dist(gen);
        if (idx < 0 || idx >= static_cast<int>(data.size())) {
          continue;
        }

        if (data[idx] == 0) {
          int row = idx / width;
          int col = idx % width;

          const double x = info.origin.position.x + (col + 0.5) * res;
          const double y = info.origin.position.y + (row + 0.5) * res;

          if(inExclusionZone(x,y)) continue;
          if(!hasClearanceCell(col, row, width, height, res, data)) continue;

          if (goal_min_distance_m_ > 0.0) {
            const geometry_msgs::msg::PoseStamped & ref =
              goal_poses_.empty() ? spawn_pose_ : goal_poses_.back();
            const double dx = x - ref.pose.position.x;
            const double dy = y - ref.pose.position.y;
            if (std::hypot(dx, dy) < goal_min_distance_m_) continue;
          }

          cell_index = idx;
          break;
        }
      }

      if (cell_index < 0) {
        RCLCPP_WARN(get_logger(),
                    "Failed to find free cell for goal %d after %d attempts",
                    i, max_tries_per_goal);
        continue;
      }

      int row = cell_index / width;
      int col = cell_index % width; 

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = fixed_frame_;
      pose.header.stamp    = this->get_clock()->now();

      pose.pose.position.x = info.origin.position.x + (col + 0.5) * res;
      pose.pose.position.y = info.origin.position.y + (row + 0.5) * res;
      pose.pose.position.z = 0.0;

      // Separate stream: goal orientation must not correlate with goal position.
      std::mt19937_64 yaw_gen(seedFor(navlearn::seed::Stream::GOAL_ORIENTATION,
                                      static_cast<uint64_t>(i)));
      std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);
      double yaw = dist_theta(yaw_gen);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      pose.pose.orientation = tf2::toMsg(q);

      goal_poses_.push_back(pose);
    }

    if (goal_poses_.empty()) {
      RCLCPP_WARN(get_logger(),
                  "loadGoals(map_random): generated 0 valid goals (map may be fully occupied/unknown)");
    } else {
      RCLCPP_INFO(get_logger(),
                  "loadGoals(map_random): generated %zu random goals from map",
                  goal_poses_.size());
    }
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Invalid goal_source in EpisodeManager config");
    return;
  }
}

void EpisodeManager::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  have_map_ = true;
  latest_map_ = *map;

  if (!have_spawn_pose_) {
    if (sampleStartPoseAt(this->get_clock()->now(), spawn_pose_)) {
      have_spawn_pose_ = true;
      RCLCPP_INFO(get_logger(), "Spawn pose captured: (x=%.2f, y=%.2f)",
                  spawn_pose_.pose.position.x, spawn_pose_.pose.position.y);
    } else {
      RCLCPP_WARN(get_logger(), "mapCallback: could not capture spawn pose yet (TF not ready)");
    }
  }

  RCLCPP_DEBUG(get_logger(), "Map received: %u x %u, res=%.3f", latest_map_.info.width, latest_map_.info.height, latest_map_.info.resolution);

  if (goal_source_ == "map_random" && goal_poses_.empty()) {
    RCLCPP_INFO(get_logger(), "First map received, generating %d random goals", goals_num_);
    loadGoals();
    RCLCPP_INFO(get_logger(), "Generated %zu random goals from map", goal_poses_.size());
  }
}

void EpisodeManager::onSetInitialPose(rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedFuture future)
{
  (void)future;
  // We intentionally don't assume any response fields exist across Nav2/ROS2 variants.
  // Application is verified via TF convergence in isInitialPoseApplied().
}

bool EpisodeManager::needCorrection(const geometry_msgs::msg::PoseStamped &ground_truth_pose, const geometry_msgs::msg::PoseStamped &estimated_pose)
{
  const double dx = ground_truth_pose.pose.position.x - estimated_pose.pose.position.x;
  const double dy = ground_truth_pose.pose.position.y - estimated_pose.pose.position.y;
  const double pos_err = std::hypot(dx, dy);

  tf2::Quaternion q_gt, q_est;
  tf2::fromMsg(ground_truth_pose.pose.orientation, q_gt);
  tf2::fromMsg(estimated_pose.pose.orientation, q_est);

  double roll_gt, pitch_gt, yaw_gt;
  tf2::Matrix3x3(q_gt).getRPY(roll_gt, pitch_gt, yaw_gt);

  double roll_est, pitch_est, yaw_est;
  tf2::Matrix3x3(q_est).getRPY(roll_est, pitch_est, yaw_est);

  const double yaw_err = std::fabs(navlearn::wrap_to_pi(yaw_gt - yaw_est));

  return (pos_err >= end_error_pos_threshold_m_) || (yaw_err >= end_error_yaw_threshold_rad_);
}

geometry_msgs::msg::PoseWithCovarianceStamped EpisodeManager::buildPoseWithCovariance(
  const geometry_msgs::msg::PoseStamped & base,
  double cov_xy,
  double cov_yaw)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header = base.header;
  msg.header.frame_id = fixed_frame_;
  msg.header.stamp = this->get_clock()->now();
  msg.pose.pose = base.pose;

  for (auto &v : msg.pose.covariance) v = 0.0;
  msg.pose.covariance[0]  = cov_xy;
  msg.pose.covariance[7]  = cov_xy;
  msg.pose.covariance[35] = cov_yaw;
  return msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped EpisodeManager::buildPerturbedPose(const geometry_msgs::msg::PoseStamped & base)
{
  // The TTC displacement. Previously mt19937(1337 + goal_index), which ignored the run
  // index entirely: every episode in a cell drew the same handful of offsets, so a cell
  // of 25 episodes tested 5 distinct conditions. Now distinct per (run, goal), and still
  // identical across controllers so the arms face the same perturbations.
  // In curve mode the displacement magnitude is the independent variable, so it is drawn
  // per goal and applied at exactly that distance. The legacy square draw bounds each
  // axis rather than the displacement, which makes its nominal range a label rather than
  // a measurement -- see bad_init_offset.hpp.
  navlearn::PoseOffset offset;
  bad_init_commanded_magnitude_m_ = -1.0;

  if (bad_init_magnitude_sampler_) {
    bad_init_commanded_magnitude_m_ = bad_init_magnitude_sampler_->sample(
      seedFor(navlearn::seed::Stream::BAD_INIT_MAGNITUDE, idx_));
    offset = navlearn::polarOffset(
      seedFor(navlearn::seed::Stream::INITIAL_POSE, idx_),
      bad_init_commanded_magnitude_m_, bad_init_yaw_range_rad_);
  } else {
    offset = navlearn::squareOffset(
      seedFor(navlearn::seed::Stream::INITIAL_POSE, idx_),
      bad_init_lin_range_m_, bad_init_yaw_range_rad_);
  }

  const double dx = offset.dx;
  const double dy = offset.dy;
  const double dyaw = offset.dyaw;

  geometry_msgs::msg::PoseStamped p = base;
  p.header.frame_id = fixed_frame_;
  p.header.stamp = this->get_clock()->now();

  p.pose.position.x += dx;
  p.pose.position.y += dy;

  tf2::Quaternion q_base, q_noise;
  tf2::fromMsg(base.pose.orientation, q_base);
  q_noise.setRPY(0.0, 0.0, dyaw);
  tf2::Quaternion q_new = q_base * q_noise;
  q_new.normalize();
  p.pose.orientation = tf2::toMsg(q_new);

  return buildPoseWithCovariance(p, bad_cov_xy_, bad_cov_yaw_);
}

bool EpisodeManager::sendInitialPoseRequest(const geometry_msgs::msg::PoseWithCovarianceStamped & pose_msg)
{
  while (!set_initial_pose_client_->wait_for_service(200ms))
  {
    if (!rclcpp::ok()) 
    {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the set_initial_pose service. Exiting.");
      return false;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "set_initial_pose service not available, waiting again...");
  }

  auto request = std::make_shared<nav2_msgs::srv::SetInitialPose::Request>();
  request->pose = pose_msg;

  (void)set_initial_pose_client_->async_send_request(request, std::bind(&EpisodeManager::onSetInitialPose, this, _1));
  return true;
}

bool EpisodeManager::isInitialPoseApplied(const geometry_msgs::msg::PoseWithCovarianceStamped & target)
{
  geometry_msgs::msg::PoseStamped est;
  if (!lookupPose(est_error_frame_, rclcpp::Time(0,0,RCL_ROS_TIME), est)) {
    return false;
  }

  const double dx = est.pose.position.x - target.pose.pose.position.x;
  const double dy = est.pose.position.y - target.pose.pose.position.y;
  const double pos_err = std::hypot(dx, dy);

  tf2::Quaternion q_tgt, q_est;
  tf2::fromMsg(target.pose.pose.orientation, q_tgt);
  tf2::fromMsg(est.pose.orientation, q_est);

  double r1, p1, yaw_tgt;
  tf2::Matrix3x3(q_tgt).getRPY(r1, p1, yaw_tgt);

  double r2, p2, yaw_est;
  tf2::Matrix3x3(q_est).getRPY(r2, p2, yaw_est);

  const double yaw_err = std::fabs(navlearn::wrap_to_pi(yaw_tgt - yaw_est));

  return (pos_err <= pose_apply_pos_tol_m_) && (yaw_err <= pose_apply_yaw_tol_rad_);
}

void EpisodeManager::startPoseSequence(const rclcpp::Time & t)
{
  geometry_msgs::msg::PoseStamped gt_pose;
  if (!lookupPose(gt_error_frame_, t, gt_pose)) {
    RCLCPP_WARN(get_logger(), "Cannot start pose sequence: TF %s->%s unavailable. Sending goal without injection.",
                fixed_frame_.c_str(), gt_error_frame_.c_str());
    sendGoal();
    return;
  }

  geometry_msgs::msg::PoseStamped est_pose;
  const bool have_est = lookupPose(est_error_frame_, t, est_pose);

  // Correct on EVERY goal that needs it, including goal 0. The old `idx_ > 0` gate
  // assumed a run starts with a converged filter, but a run inherits AMCL from wherever
  // the previous run left it -- all runs of a cell share one bringup -- and goal 0 then
  // depended on its own bad-init injection, an unverified single publish, to reset the
  // filter through possibly metres of inherited error. When that reset did not take, the
  // whole run started wrecked and stayed wrecked: measured 2026-08-01 as identical
  // 20-goal cells swinging 65% / 10% / 10% / 0% true success, with the collapsed cells'
  // localization error persisting 2.6-4.9 m across entire runs. Independence of episodes
  // must be constructed at every goal start, not inherited from the previous exit path.
  const bool need_correct =
    have_est && needCorrection(gt_pose, est_pose);

  correct_pose_msg_ = buildPoseWithCovariance(gt_pose, correct_cov_xy_, correct_cov_yaw_);
  bad_pose_msg_ = buildPerturbedPose(gt_pose);

  pose_sequence_pending_ = true;
  pose_seq_started_at_ = this->get_clock()->now();

  if (need_correct) {
    pose_seq_stage_ = PoseSeqStage::SEND_CORRECT;
  } else {
    pose_seq_stage_ = PoseSeqStage::SEND_BAD;
  }

  pose_stage_started_at_ = this->get_clock()->now();
}

void EpisodeManager::advancePoseSequence(const rclcpp::Time & t)
{
  (void)t;

  const auto now = this->get_clock()->now();

  if ((now - pose_seq_started_at_).seconds() > pose_sequence_timeout_sec_) {
    RCLCPP_WARN(get_logger(), "Pose sequence timed out (%.2fs). Sending goal anyway.", pose_sequence_timeout_sec_);
    pose_sequence_pending_ = false;
    pose_seq_stage_ = PoseSeqStage::IDLE;
    sendGoal();
    return;
  }

  switch (pose_seq_stage_)
  {
    case PoseSeqStage::SEND_CORRECT:
    {
      pose_wait_target_ = correct_pose_msg_;
      if (!sendInitialPoseRequest(correct_pose_msg_)) {
        RCLCPP_WARN(get_logger(), "Failed to send correct initial pose request. Sending goal anyway.");
        pose_sequence_pending_ = false;
        pose_seq_stage_ = PoseSeqStage::IDLE;
        sendGoal();
        return;
      }
      pose_stage_started_at_ = now;
      pose_seq_stage_ = PoseSeqStage::WAIT_CORRECT_APPLIED;
      return;
    }

    case PoseSeqStage::WAIT_CORRECT_APPLIED:
    {
      if ((now - pose_stage_started_at_).seconds() > pose_apply_timeout_sec_) {
        RCLCPP_WARN(get_logger(), "Correct pose apply timed out (%.2fs). Proceeding to BAD injection.", pose_apply_timeout_sec_);
        pose_seq_stage_ = PoseSeqStage::SEND_BAD;
        pose_stage_started_at_ = now;
        return;
      }

      if (isInitialPoseApplied(pose_wait_target_)) {
        pose_seq_stage_ = PoseSeqStage::SEND_BAD;
        pose_stage_started_at_ = now;
      }
      return;
    }

    case PoseSeqStage::SEND_BAD:
    {
      pose_wait_target_ = bad_pose_msg_;
      if (!sendInitialPoseRequest(bad_pose_msg_)) {
        RCLCPP_WARN(get_logger(), "Failed to send bad initial pose request. Sending goal anyway.");
        pose_sequence_pending_ = false;
        pose_seq_stage_ = PoseSeqStage::IDLE;
        sendGoal();
        return;
      }
      pose_stage_started_at_ = now;
      pose_seq_stage_ = PoseSeqStage::WAIT_BAD_APPLIED;
      return;
    }

    case PoseSeqStage::WAIT_BAD_APPLIED:
    {
      if ((now - pose_stage_started_at_).seconds() > pose_apply_timeout_sec_) {
        RCLCPP_WARN(get_logger(), "Bad pose apply timed out (%.2fs). Sending goal anyway.", pose_apply_timeout_sec_);
        bad_init_pose_pub_->publish(bad_pose_msg_);
        pose_sequence_pending_ = false;
        pose_seq_stage_ = PoseSeqStage::IDLE;
        is_first_bad_init_ = false;
        sendGoal();
        return;
      }

      if (isInitialPoseApplied(pose_wait_target_)) {
        bad_init_pose_pub_->publish(bad_pose_msg_);
        pose_sequence_pending_ = false;
        pose_seq_stage_ = PoseSeqStage::IDLE;
        is_first_bad_init_ = false;
        sendGoal();
      }
      return;
    }

    case PoseSeqStage::IDLE:
    default:
      pose_sequence_pending_ = false;
      return;
  }
}

// ---------------- kidnap helpers ----------------

geometry_msgs::msg::Pose EpisodeManager::unsampledKidnapPose_()
{
  // Sentinel for "no target was sampled this goal". NaN rather than zero because the
  // origin is legitimate free space on the campaign map: a stale or defaulted target
  // there reads as a plausible teleport, while NaN is unmistakably not a measurement.
  // Without this, a kidnap whose sampling failed published the PREVIOUS goal's target —
  // observed in cmp_ttr run 1 (2026-07-29), where goals 0 and 1 carried the identical
  // target (0.8616, 0.1936) and goal 1's kidnap never fired.
  geometry_msgs::msg::Pose p;
  p.position.x = std::numeric_limits<double>::quiet_NaN();
  p.position.y = std::numeric_limits<double>::quiet_NaN();
  p.position.z = std::numeric_limits<double>::quiet_NaN();
  p.orientation.w = 1.0;
  return p;
}

unique_identifier_msgs::msg::UUID EpisodeManager::makeUUID_(uint32_t seed)
{
  unique_identifier_msgs::msg::UUID id;
  std::mt19937 gen(seed);
  std::uniform_int_distribution<int> dist(0, 255);
  for (auto &b : id.uuid) {
    b = static_cast<uint8_t>(dist(gen));
  }
  return id;
}

bool EpisodeManager::sampleKidnapPoseFromMap_(const geometry_msgs::msg::PoseStamped & ref, geometry_msgs::msg::Pose & out_pose, double ring_min_m, double ring_max_m)
{
  if (!have_map_) return false;

  const auto & info = latest_map_.info;
  const int width   = static_cast<int>(info.width);
  const int height  = static_cast<int>(info.height);
  const double res  = info.resolution;
  const auto & data = latest_map_.data;

  if (width <= 0 || height <= 0 || data.empty()) return false;

  // The TTR teleport destination. Same defect as the TTC offset: fixed constant plus
  // goal index, run index absent.
  std::mt19937_64 gen(seedFor(navlearn::seed::Stream::KIDNAP_TARGET, idx_));
  std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);
  std::uniform_real_distribution<double> dist_unit(0.0, 1.0);

  // Sampled INSIDE the annulus, not rejected into it.
  //
  // This previously drew a uniformly random cell from the whole map and discarded it
  // unless it happened to land in the ring [kidnap_distance_m, kidnap_max_distance_m]
  // around the robot. At the medium preset that ring is 0.8-1.2 m, an area of about
  // 2.5 m^2 against a map of a couple of hundred square metres — so roughly one draw in a
  // hundred was even a candidate before the free-space, clearance and exclusion tests ran.
  // With the robot in a corridor or a small room, most of the ring is wall and all 1500
  // tries could miss: the pilot logged 7 "no valid pose found" failures, none of which
  // meant the ring was actually empty.
  //
  // Drawing radius and bearing directly puts every attempt in the ring by construction.
  // Radius uses sqrt of a uniform over the squared bounds so samples are area-uniform
  // rather than crowding the inner edge. The remaining rejections are then genuine: a
  // failure now means the ring really has no valid pose, which is a fact about the map
  // worth recording rather than an artifact of the sampler.
  const double r_min = std::max(0.0, ring_min_m);
  const double r_max = (ring_max_m > 0.0) ? ring_max_m : (r_min + 1.0);
  if (r_max < r_min) return false;

  for (int attempt = 0; attempt < kidnap_max_sample_tries_; ++attempt)
  {
    const double u = dist_unit(gen);
    const double r = std::sqrt(r_min * r_min + u * (r_max * r_max - r_min * r_min));
    const double bearing = dist_theta(gen);

    const double x = ref.pose.position.x + r * std::cos(bearing);
    const double y = ref.pose.position.y + r * std::sin(bearing);

    const int col = static_cast<int>((x - info.origin.position.x) / res);
    const int row = static_cast<int>((y - info.origin.position.y) / res);
    if (col < 0 || col >= width || row < 0 || row >= height) continue;

    const int idx = row * width + col;
    if (idx < 0 || idx >= static_cast<int>(data.size())) continue;
    if (data[idx] != 0) continue;

    if (inExclusionZone(x, y)) continue;
    if (!hasClearanceCell(col, row, width, height, res, data)) continue;

    out_pose.position.x = x;
    out_pose.position.y = y;
    out_pose.position.z = kidnap_z_;

    const double yaw = dist_theta(gen);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    out_pose.orientation = tf2::toMsg(q);

    return true;
  }

  return false;
}

void EpisodeManager::publishKidnapEvent_(bool success)
{
  navlearn_msgs::msg::KidnapEvent ev;
  ev.header.stamp = kidnap_event_time_;
  ev.header.frame_id = fixed_frame_;
  ev.goal_id = goal_id_;
  ev.kidnap_pose = kidnap_target_pose_;
  ev.success = success;
  ev.attempt_id = kidnap_attempt_id_;
  ev.reference_pose = kidnap_reference_pose_;
  ev.reference_available = have_kidnap_reference_;
  ev.commanded_magnitude_m = kidnap_commanded_magnitude_m_;

  if (!have_kidnap_reference_) {
    // Loud, because it is otherwise undetectable until analysis months later: the goal
    // still produces a complete-looking row, it simply has no measurable displacement,
    // and the distance-vs-ambiguity comparison quietly loses a trial.
    RCLCPP_ERROR(get_logger(),
      "Kidnap event for goal %zu carries NO reference pose. Realised displacement is "
      "unrecoverable for this goal and it must be excluded from any distance-based "
      "analysis.", idx_);
  }

  kidnap_pub_->publish(ev);
}

bool EpisodeManager::isKidnapGTVerified_()
{
  geometry_msgs::msg::PoseStamped gt;
  if (!lookupPose(gt_error_frame_, rclcpp::Time(0,0,RCL_ROS_TIME), gt)) {
    return false;
  }

  const double dx = gt.pose.position.x - kidnap_target_pose_.position.x;
  const double dy = gt.pose.position.y - kidnap_target_pose_.position.y;
  const double pos_err = std::hypot(dx, dy);

  const double yaw_gt = tf2::getYaw(gt.pose.orientation);
  const double yaw_tgt = tf2::getYaw(kidnap_target_pose_.orientation);
  const double yaw_err = std::fabs(navlearn::wrap_to_pi(yaw_gt - yaw_tgt));

  return (pos_err <= kidnap_verify_pos_tol_m_) && (yaw_err <= kidnap_verify_yaw_tol_rad_);
}

bool EpisodeManager::setEntityPose_(double x, double y, double z, double yaw)
{
  if (!set_pose_entity_client_->service_is_ready()) {
    if (!set_pose_entity_client_->wait_for_service(0s)) {
      return false;
    }
  }

  auto req = std::make_shared<navlearn_msgs::srv::SetEntityPose::Request>();
  req->name = entity_name_;

  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  p.orientation = tf2::toMsg(q);

  req->pose = p;

  kidnap_target_pose_ = p;

  kidnap_future_ = set_pose_entity_client_->async_send_request(req);
  return true;
}

void EpisodeManager::maybeKidnap(const rclcpp::Time & now)
{
  if (!kidnap_enabled_) return;

  if (kidnap_stage_ == KidnapStage::DONE) return;

  if (kidnap_stage_ == KidnapStage::WAIT_SERVICE_RESPONSE)
  {
    if (kidnap_future_.wait_for(0s) != std::future_status::ready) {
      return;
    }

    const auto res = kidnap_future_.get();
    if (!res || !res->success) {
      publishKidnapEvent_(false);
      kidnap_stage_ = KidnapStage::DONE;
      return;
    }

    publishKidnapEvent_(true);

    // A kidnap must not tell the localizer it has been kidnapped.
    //
    // This unconditionally called global localization reinit immediately after the
    // teleport, which scatters AMCL's particles across the whole map. That is not a
    // kidnapped robot. A real kidnapped robot's filter remains converged and confidently
    // wrong, with no odometry cue — the wheels never turned — and must discover the error
    // from scan mismatch alone. Handing it "you are completely lost" replaces the
    // phenomenon under study with global re-localization from scratch while driving, which
    // is a different and much harder problem, and one the harness inflicted on itself.
    //
    // It also accounts for the pilot's terminal covariance of 32 m^2: the filter was not
    // lost by the kidnap, it was scattered by this call.
    //
    // Kept as an opt-in parameter because "the robot knows it was moved" is a legitimate
    // alternative condition to study (a wheel-lift or IMU-drop sensor would provide it) —
    // but it is a different experiment and must be requested explicitly, not inherited.
    if (kidnap_notify_localizer_) {
      RCLCPP_WARN(get_logger(),
        "kidnap_notify_localizer is enabled: forcing AMCL global reinit after the "
        "teleport. The filter is being told it is lost, so this measures global "
        "re-localization, not unaided recovery from a kidnap.");
      callReinitGlobalLocalization();
    }
    kidnap_stage_ = KidnapStage::DONE;
    return;
  }

  if (kidnap_stage_ != KidnapStage::IDLE) return;

  const double elapsed = (now - goal_started_at_).seconds();

  geometry_msgs::msg::PoseStamped ref_pose;
  if (!lookupPose(gt_error_frame_, rclcpp::Time(0,0,RCL_ROS_TIME), ref_pose)) {
    return;
  }

  double traveled = 0.0;
  if (have_start_gt_) {
    traveled = std::hypot(ref_pose.pose.position.x - start_gt_pose_.pose.position.x,
                         ref_pose.pose.position.y - start_gt_pose_.pose.position.y);
  }

  const bool trigger =
    (elapsed >= kidnap_sampled_delay_sec_) || (traveled >= kidnap_min_travel_m_);

  if (!trigger) return;

  if (!have_map_) {
    return;
  }

  // Latched here, at the last point before the teleport is committed, because this is the
  // last instant at which the robot is still where it was. Everything after this either
  // moves it or fails trying, and both outcomes need the same "before" to be meaningful.
  kidnap_reference_pose_ = ref_pose.pose;
  have_kidnap_reference_ = true;

  // Severity for THIS goal. In curve mode it is drawn from the campaign seed, so every
  // goal in the sweep gets its own magnitude and the whole sweep still reproduces from a
  // single number. In fixed mode the configured band is used unchanged.
  double ring_min = kidnap_distance_m_;
  double ring_max = kidnap_max_distance_m_;
  kidnap_commanded_magnitude_m_ = -1.0;

  if (magnitude_sampler_) {
    kidnap_commanded_magnitude_m_ = magnitude_sampler_->sample(
      seedFor(navlearn::seed::Stream::PERTURBATION_MAGNITUDE, idx_));
    const double band = std::max(0.0, kidnap_magnitude_band_);
    ring_min = kidnap_commanded_magnitude_m_ * (1.0 - band);
    ring_max = kidnap_commanded_magnitude_m_ * (1.0 + band);
  }

  geometry_msgs::msg::Pose target;
  if (!sampleKidnapPoseFromMap_(ref_pose, target, ring_min, ring_max)) {
    // Reported as a failed attempt, not merely warned about. Previously this path
    // published nothing at all, so a goal whose kidnap never fired was recorded as an
    // ordinary TTR trial — the perturbation was the independent variable and its absence
    // was invisible in the data. The pilot hit this seven times in one run.
    RCLCPP_WARN(get_logger(),
      "Kidnap sample failed for goal %zu (no valid pose in the %.2f-%.2f m ring). This "
      "goal has NO perturbation applied and must not be pooled with kidnapped goals.",
      idx_, ring_min, ring_max);
    kidnap_event_time_ = this->get_clock()->now();
    publishKidnapEvent_(false);
    kidnap_stage_ = KidnapStage::DONE;
    return;
  }

  // An identifier, not an experimental condition — but derived the same way so that two
  // episodes cannot share an attempt id.
  kidnap_attempt_id_ = makeUUID_(static_cast<uint32_t>(
    seedFor(navlearn::seed::Stream::ATTEMPT_ID, idx_)));
  kidnap_event_time_ = now;
  kidnap_target_pose_ = target;

  const double yaw = tf2::getYaw(target.orientation);
  if (!setEntityPose_(target.position.x, target.position.y, target.position.z, yaw)) {
    publishKidnapEvent_(false);
    kidnap_stage_ = KidnapStage::DONE;
    return;
  }

  kidnap_stage_ = KidnapStage::WAIT_SERVICE_RESPONSE;
}

void EpisodeManager::timerCallback() {
  if (!client_->wait_for_action_server(0s)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "NavigateToPose server not ready");
    return;
  }

  if (!have_spawn_pose_ && have_map_) {
    if (sampleStartPoseAt(this->get_clock()->now(), spawn_pose_)) {
      have_spawn_pose_ = true;
      RCLCPP_INFO(get_logger(), "Spawn pose captured (timer retry): (x=%.2f, y=%.2f)",
                  spawn_pose_.pose.position.x, spawn_pose_.pose.position.y);
    }
  }

  const auto now = this->get_clock()->now();

  if (active_) {
    maybeKidnap(now);
    return;
  }

  // Recovery-on-failure state machine (async)
  if (recovery_stage_ != RecoveryStage::IDLE) {
    advanceRecovery(now);
    return;
  }

  if (goal_request_inflight_) {
    return;
  }

  if (idx_ >= goal_poses_.size()) {
    return;
  }

  if (now < next_allowed_send_) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
      "Cooling down... %.2fs left", (next_allowed_send_ - now).seconds());
    return;
  }

  if (!bad_init_test_) {
    sendGoal();
    return;
  }

  if (pose_sequence_pending_) {
    advancePoseSequence(now);
    return;
  }

  startPoseSequence(now);
}

void EpisodeManager::sendGoal() {
  nav2_msgs::action::NavigateToPose::Goal goal_msg;
  goal_msg.pose = goal_poses_[idx_];

  auto opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  opts.goal_response_callback = std::bind(&EpisodeManager::onGoalResponse, this, _1);
  opts.feedback_callback      = std::bind(&EpisodeManager::onFeedback, this, _1, _2);
  opts.result_callback        = std::bind(&EpisodeManager::onResult, this, _1);

  goal_request_inflight_ = true;
  client_->async_send_goal(goal_msg, opts);

  RCLCPP_INFO(get_logger(), "Sent goal %zu: (x=%.2f, y=%.2f)",
              idx_, goal_poses_[idx_].pose.position.x, goal_poses_[idx_].pose.position.y);
}

void EpisodeManager::onGoalResponse(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & gh) {
  goal_request_inflight_ = false;

  if (!gh) {
    RCLCPP_ERROR(get_logger(), "Goal %zu rejected by NavigateToPose", idx_);
    idx_++;
    return;
  }

  RCLCPP_INFO(get_logger(), "Goal %zu accepted by NavigateToPose", idx_);
  active_ = true;

  const auto goal_id = gh->get_goal_id();
  std::copy(goal_id.begin(), goal_id.end(), goal_id_.uuid.begin());

  navlearn_msgs::msg::EpisodeEvent ev;
  ev.header.stamp = this->get_clock()->now();
  ev.state  = navlearn_msgs::msg::EpisodeEvent::START;
  ev.result = navlearn_msgs::msg::EpisodeEvent::RESULT_NA;
  ev.goal_id = goal_id_;
  ev.goal_pose  = goal_poses_[idx_];

  ev.stamp_received = ev.header.stamp;
  (void)sampleStartPoseAt(ev.header.stamp, start_pose_);
  ev.start_pose = start_pose_;

  ev.stamp_terminated.sec = 0;
  ev.stamp_terminated.nanosec = 0;
  ev.nav_time.sec = 0;
  ev.nav_time.nanosec = 0;

  // Compute Euclidean start-to-goal distance (optimal path approximation)
  const auto & goal_pose = goal_poses_[idx_];
  const double dx = goal_pose.pose.position.x - start_pose_.pose.position.x;
  const double dy = goal_pose.pose.position.y - start_pose_.pose.position.y;
  ev.optimal_path_m = std::hypot(dx, dy);

  stamp_received_ = rclcpp::Time(ev.stamp_received);

  episode_pub_->publish(ev);
  RCLCPP_INFO(get_logger(), "START episode for goal %zu (goal_id set)", idx_);

  goal_started_at_ = ev.header.stamp;

  have_start_gt_ = lookupPose(gt_error_frame_, rclcpp::Time(0,0,RCL_ROS_TIME), start_gt_pose_);

  double lo = kidnap_delay_min_sec_;
  double hi = kidnap_delay_max_sec_;
  if (hi < lo) std::swap(lo, hi);

  std::mt19937_64 gen(seedFor(navlearn::seed::Stream::KIDNAP_DELAY, idx_));
  std::uniform_real_distribution<double> dist_delay(lo, hi);
  kidnap_sampled_delay_sec_ = dist_delay(gen);

  kidnap_stage_ = KidnapStage::IDLE;
  have_kidnap_reference_ = false;
}

void EpisodeManager::onFeedback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &,
                const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  (void)feedback;
}

void EpisodeManager::clearCostmaps(const std::string & reason)
{
  auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  int dispatched = 0;

  for (auto & [name, client] :
       std::vector<std::pair<std::string, rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr>>{
         {"global", clear_global_costmap_client_}, {"local", clear_local_costmap_client_}})
  {
    if (!client) continue;
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(),
        "Cannot clear %s costmap (%s): service not ready. Phantom obstacles from any "
        "mislocalized period will persist into the next goal.",
        name.c_str(), reason.c_str());
      continue;
    }
    // Fire and forget: a clear is idempotent and the next goal must not be blocked waiting
    // on it. Failure is reported above rather than silently swallowed.
    client->async_send_request(request);
    ++dispatched;
  }

  RCLCPP_INFO(get_logger(), "Cleared %d/2 costmaps (%s).", dispatched, reason.c_str());
}

void EpisodeManager::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  last_amcl_pose_ = *msg;
  have_amcl_pose_ = true;
}

namespace {

/// Wrap an angle difference into [-pi, pi].
double wrapAngle(double angle)
{
  while (angle >  M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

/// Extract yaw from a quaternion, matching the convention used elsewhere in this file.
double yawOf(const geometry_msgs::msg::Quaternion & q_msg)
{
  tf2::Quaternion q;
  tf2::fromMsg(q_msg, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace

navlearn_msgs::msg::TerminalPoseReport EpisodeManager::captureTerminalPose(
  const geometry_msgs::msg::PoseStamped & goal_pose, const rclcpp::Time & terminated_at)
{
  // Every scalar starts at -1.0, the message's documented "not available" sentinel. Zero
  // is a legitimate and highly meaningful value for each of these distances, so it must
  // never be able to masquerade as a missing measurement.
  navlearn_msgs::msg::TerminalPoseReport report;
  report.gt_available = false;
  report.estimate_available = false;
  report.true_distance_to_goal_m = -1.0;
  report.estimated_distance_to_goal_m = -1.0;
  report.localization_error_m = -1.0;
  report.true_yaw_error_rad = -1.0;
  report.covariance_xx = -1.0;
  report.covariance_yy = -1.0;
  report.covariance_yaw = -1.0;
  report.filter_converged = false;
  report.convergence_threshold_m2 = terminal_converged_cov_threshold_m2_;
  report.estimate_age_sec = -1.0;

  // Ground truth, from the simulator's transform. Deliberately the latest available
  // rather than one at the termination stamp: the robot is stationary by now, so the
  // newest transform is the right answer, and an exact-stamp lookup could fail on
  // extrapolation and lose the single number this whole record exists to capture.
  geometry_msgs::msg::PoseStamped gt_pose;
  if (lookupPose(gt_error_frame_, rclcpp::Time(0, 0, RCL_ROS_TIME), gt_pose)) {
    report.gt_available = true;
    report.true_pose = gt_pose;
    report.true_distance_to_goal_m = std::hypot(
      gt_pose.pose.position.x - goal_pose.pose.position.x,
      gt_pose.pose.position.y - goal_pose.pose.position.y);
    report.true_yaw_error_rad = std::fabs(
      wrapAngle(yawOf(gt_pose.pose.orientation) - yawOf(goal_pose.pose.orientation)));
  } else {
    RCLCPP_ERROR(get_logger(),
                 "Terminal capture: no '%s' transform. True distance to goal is LOST for "
                 "goal %zu — this is the campaign's headline measurement.",
                 gt_error_frame_.c_str(), idx_);
  }

  if (have_amcl_pose_) {
    report.estimate_available = true;
    report.estimated_pose.header = last_amcl_pose_.header;
    report.estimated_pose.pose = last_amcl_pose_.pose.pose;
    report.estimated_distance_to_goal_m = std::hypot(
      last_amcl_pose_.pose.pose.position.x - goal_pose.pose.position.x,
      last_amcl_pose_.pose.pose.position.y - goal_pose.pose.position.y);

    // Row-major 6x6: xx at 0, yy at 7, yaw at 35.
    report.covariance_xx  = last_amcl_pose_.pose.covariance[0];
    report.covariance_yy  = last_amcl_pose_.pose.covariance[7];
    report.covariance_yaw = last_amcl_pose_.pose.covariance[35];
    report.filter_converged =
      (report.covariance_xx + report.covariance_yy) <= terminal_converged_cov_threshold_m2_;

    // AMCL publishes only when it updates (update_min_d / update_min_a), and the robot has
    // stopped, so the newest estimate may predate termination. Recording the age keeps
    // that visible in the data instead of leaving it to be assumed away in analysis.
    const rclcpp::Time estimate_stamp(last_amcl_pose_.header.stamp,
                                      terminated_at.get_clock_type());
    if (estimate_stamp.nanoseconds() > 0) {
      report.estimate_age_sec = (terminated_at - estimate_stamp).seconds();
    }

    if (report.gt_available) {
      report.localization_error_m = std::hypot(
        gt_pose.pose.position.x - last_amcl_pose_.pose.pose.position.x,
        gt_pose.pose.position.y - last_amcl_pose_.pose.pose.position.y);
    }
  } else {
    RCLCPP_WARN(get_logger(),
                "Terminal capture: no pose seen on '%s'; localization error unavailable "
                "for goal %zu.", amcl_pose_topic_.c_str(), idx_);
  }

  return report;
}

void EpisodeManager::onResult(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result) {
  navlearn_msgs::msg::EpisodeEvent ev;
  ev.header.stamp = this->get_clock()->now();
  ev.state = navlearn_msgs::msg::EpisodeEvent::END;
  ev.bad_init_commanded_magnitude_m = bad_init_commanded_magnitude_m_;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      ev.result = navlearn_msgs::msg::EpisodeEvent::RESULT_SUCCEEDED; break;
    case rclcpp_action::ResultCode::CANCELED:
      ev.result = navlearn_msgs::msg::EpisodeEvent::RESULT_CANCELED;  break;
    case rclcpp_action::ResultCode::ABORTED:
    default:
      ev.result = navlearn_msgs::msg::EpisodeEvent::RESULT_FAILED;    break;
  }

  ev.goal_id = goal_id_;
  ev.goal_pose  = goal_poses_[idx_];
  ev.start_pose = start_pose_;

  // Copy optimal_path_m to END event so MetricsCompiler can use it
  const double dx_end = goal_poses_[idx_].pose.position.x - start_pose_.pose.position.x;
  const double dy_end = goal_poses_[idx_].pose.position.y - start_pose_.pose.position.y;
  ev.optimal_path_m = std::hypot(dx_end, dy_end);

  ev.stamp_received = stamp_received_;
  ev.stamp_terminated = ev.header.stamp;

  if ((stamp_received_.nanoseconds() > 0) && (ev.stamp_terminated.sec != 0 || ev.stamp_terminated.nanosec != 0)) {
    const rclcpp::Time t_end(ev.stamp_terminated);
    const rclcpp::Duration dt = t_end - stamp_received_;
    const int64_t nsec_total = dt.nanoseconds();
    ev.nav_time.sec     = static_cast<int32_t>(nsec_total / 1000000000LL);
    ev.nav_time.nanosec = static_cast<uint32_t>(nsec_total % 1000000000LL);
  } else {
    ev.nav_time.sec = 0;
    ev.nav_time.nanosec = 0;
  }

  // Captured before publishing, while the robot is still standing where it stopped.
  ev.terminal = captureTerminalPose(goal_poses_[idx_], rclcpp::Time(ev.stamp_terminated));

  episode_pub_->publish(ev);

  const char *res_str =
    (ev.result == navlearn_msgs::msg::EpisodeEvent::RESULT_SUCCEEDED) ? "SUCCEEDED" :
    (ev.result == navlearn_msgs::msg::EpisodeEvent::RESULT_CANCELED)  ? "CANCELED"  : "FAILED";

  RCLCPP_INFO(get_logger(), "END goal %zu result=%s (%u) nav_time=%.3fs",
              idx_, res_str, static_cast<unsigned>(ev.result),
              (rclcpp::Time(ev.stamp_terminated) - stamp_received_).seconds());

  RCLCPP_INFO(get_logger(),
              "  terminal: true_dist=%.3fm est_dist=%.3fm loc_err=%.3fm converged=%s",
              ev.terminal.true_distance_to_goal_m,
              ev.terminal.estimated_distance_to_goal_m,
              ev.terminal.localization_error_m,
              ev.terminal.filter_converged ? "yes" : "no");

  // The finding this campaign exists to quantify: Nav2 declared arrival, but the robot is
  // measurably elsewhere. Logged at WARN so it is visible while a cell runs, not only in
  // post-hoc analysis. The threshold is the goal checker's own tolerance with a margin,
  // so this fires only when the discrepancy exceeds what success was supposed to mean.
  const bool false_success =
    ev.result == navlearn_msgs::msg::EpisodeEvent::RESULT_SUCCEEDED &&
    ev.terminal.gt_available &&
    ev.terminal.true_distance_to_goal_m > end_error_pos_threshold_m_;

  if (false_success) {
    RCLCPP_WARN(get_logger(),
                "  FALSE SUCCESS: reported SUCCEEDED but ground truth is %.3f m from the "
                "goal (threshold %.3f m); the filter believed it was %.3f m away.",
                ev.terminal.true_distance_to_goal_m, end_error_pos_threshold_m_,
                ev.terminal.estimated_distance_to_goal_m);
  }

  active_ = false;

  // A false success must take the recovery path, not the success path. Nav2 said
  // SUCCEEDED, but the robot is measurably elsewhere and the filter is still wrong;
  // advancing without correction hands that error to the next goal as its starting
  // state. Measured on 2026-07-29 (cmp_ttr run 1): goal 0 ended a false success, goal
  // 1's kidnap never applied, and goal 1 still terminated 11.15 m from its goal as a
  // second false success — pure carryover. Episodes are meant to be independent trials;
  // the costmap-clearing decision already says so, and the filter is subject to the
  // same rule.
  if ((ev.result != navlearn_msgs::msg::EpisodeEvent::RESULT_SUCCEEDED || false_success)
      && (bad_init_test_ || kidnap_enabled_)) {
    // Recovery: teleport to current goal, re-localize, then advance
    RCLCPP_WARN(get_logger(), "Goal %zu %s during TTC/TTR experiment. Initiating recovery.",
                idx_, false_success ? "was a FALSE SUCCESS" : "FAILED");
    recovery_stage_ = RecoveryStage::TELEPORT_PENDING;
    recovery_start_time_ = this->get_clock()->now();
    initiateRecoveryTeleport(goal_poses_[idx_]);
  } else {
    // Clear before advancing, during the inter-goal dwell, so the costmaps have the full
    // dwell to refill from live scans before the next goal is planned. Episodes are meant
    // to be independent trials; inheriting the previous episode's phantom obstacles is
    // contamination, and because cells beyond walls never raytrace clear it would
    // accumulate monotonically over a 72-cell campaign.
    if (clear_costmaps_between_goals_) {
      clearCostmaps("between goals");
    }

    idx_++;
    next_allowed_send_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(dwell_sec_);

    goal_id_.uuid.fill(0);
    start_pose_ = geometry_msgs::msg::PoseStamped();

    pose_sequence_pending_ = false;
    pose_seq_stage_ = PoseSeqStage::IDLE;

    kidnap_stage_ = KidnapStage::IDLE;
    have_start_gt_ = false;
    have_kidnap_reference_ = false;
    kidnap_target_pose_ = unsampledKidnapPose_();
    kidnap_sampled_delay_sec_ = 0.0;

    if (idx_ >= goal_poses_.size()) {
      RCLCPP_INFO(get_logger(), "All goals done");
      rclcpp::shutdown();
    }
  }
}

void EpisodeManager::callReinitGlobalLocalization()
{
  const auto now = this->now();

  if (reinit_in_flight_) {
    RCLCPP_WARN(this->get_logger(), "Reinit global localization already in flight; skipping.");
    return;
  }

  if ((now - last_reinit_time_) < reinit_cooldown_) {
    RCLCPP_WARN(this->get_logger(), "Reinit global localization cooldown active; skipping.");
    return;
  }

  if (!reinit_global_loc_client_->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(),
                "/reinitialize_global_localization not ready; skipping.");
    return;
  }

  reinit_in_flight_ = true;
  last_reinit_time_ = now;

  auto req = std::make_shared<std_srvs::srv::Empty::Request>();

  using SharedFuture = rclcpp::Client<std_srvs::srv::Empty>::SharedFuture;

  reinit_global_loc_client_->async_send_request(
    req,
    [this](SharedFuture /*future*/) {
      reinit_in_flight_ = false;
      RCLCPP_INFO(this->get_logger(), "Called /reinitialize_global_localization (with cooldown).");
    }
  );
}

void EpisodeManager::forceReinitGlobalLocalization()
{
  if (!reinit_global_loc_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(),
                "Recovery: /reinitialize_global_localization not ready; skipping reinit.");
    return;
  }

  auto req = std::make_shared<std_srvs::srv::Empty::Request>();

  using SharedFuture = rclcpp::Client<std_srvs::srv::Empty>::SharedFuture;

  reinit_global_loc_client_->async_send_request(
    req,
    [this](SharedFuture /*future*/) {
      RCLCPP_INFO(get_logger(), "Recovery: forced AMCL global reinit (cooldown bypassed).");
    }
  );

  last_reinit_time_ = this->get_clock()->now();
}

void EpisodeManager::initiateRecoveryTeleport(
    const geometry_msgs::msg::PoseStamped & goal_pose)
{
  RCLCPP_INFO(get_logger(),
              "Recovery: teleporting robot to goal (%.2f, %.2f) for re-localization.",
              goal_pose.pose.position.x, goal_pose.pose.position.y);

  const double yaw = tf2::getYaw(goal_pose.pose.orientation);
  if (!setEntityPose_(goal_pose.pose.position.x,
                      goal_pose.pose.position.y,
                      kidnap_z_,
                      yaw)) {
    // The teleport is unavailable -- in TTC cells the pose server is only launched when
    // kidnap is enabled, so this path is the NORM there, not an edge case. The teleport
    // is optional (it repositions the robot for convenience); the hygiene is not. Jumping
    // straight to DONE skipped both the corrective initialpose and the costmap clear, so
    // every failed TTC goal left phantom obstacles behind: the next goal's pose sequence
    // corrects the filter but nothing corrects the map, one early failure can ignite a
    // run-wide planning-failure spiral, and episodes stop being independent trials --
    // exactly what PROTOCOL.md forbids. Observed 2026-08-01: leg1 mppi v3 collapsed to
    // 0/120 while an identically-configured 20-goal probe scored 65%, the divergence
    // starting at the first goal after a skipped recovery.
    //
    // So: skip only the teleport. Correct the filter at the robot's ACTUAL pose (ground
    // truth, which the episode manager has), clear both costmaps, and wait for
    // convergence exactly as the teleport path does.
    RCLCPP_ERROR(get_logger(),
      "Recovery: setEntityPose_ unavailable. Skipping teleport but still correcting the "
      "filter at the robot's actual pose and clearing costmaps.");

    geometry_msgs::msg::PoseStamped gt_pose;
    if (lookupPose(gt_error_frame_, rclcpp::Time(0, 0, RCL_ROS_TIME), gt_pose)) {
      sendInitialPoseRequest(
        buildPoseWithCovariance(gt_pose, correct_cov_xy_, correct_cov_yaw_));
      clearCostmaps("recovery (teleport unavailable)");
      recovery_start_time_ = this->get_clock()->now();
      recovery_stage_ = RecoveryStage::WAIT_CONVERGENCE;
    } else {
      // No ground truth either: nothing can be corrected against. Advance rather than
      // hang, but say so loudly -- this goal's successor starts contaminated.
      RCLCPP_ERROR(get_logger(),
        "Recovery: no ground truth available; next goal starts CONTAMINATED "
        "(uncorrected filter, uncleared costmaps).");
      recovery_stage_ = RecoveryStage::DONE;
    }
  }
}

void EpisodeManager::advanceRecovery(const rclcpp::Time & now)
{
  switch (recovery_stage_) {
    case RecoveryStage::TELEPORT_PENDING:
    {
      // Wait for set_pose service future to complete.
      // setEntityPose_ is fire-and-forget via async; wait a fixed 0.5s for Gazebo.
      if ((now - recovery_start_time_).seconds() > 0.5) {
        RCLCPP_INFO(get_logger(), "Recovery: teleport assumed complete, reinitializing AMCL.");
        recovery_stage_ = RecoveryStage::REINIT_PENDING;
      }
      break;
    }

    case RecoveryStage::REINIT_PENDING:
    {
      // Deliberately no global reinit here.
      //
      // This used to call forceReinitGlobalLocalization() and then immediately publish the
      // correct pose — two contradictory operations racing each other. The robot has just
      // been teleported to a pose we chose, so its location is known exactly; scattering
      // particles across the map discards that and then asks the filter to rediscover it.
      // Setting the known pose with tight covariance is both correct and instant.
      geometry_msgs::msg::PoseWithCovarianceStamped correct_pose =
          buildPoseWithCovariance(goal_poses_[idx_], correct_cov_xy_, correct_cov_yaw_);
      sendInitialPoseRequest(correct_pose);

      // Clear the costmaps. While the filter was wrong, every scan was inserted at the
      // wrong map coordinates: phantom obstacles appeared in open floor and real walls
      // were raytraced away as free space. Nothing in Nav2 undoes that on its own — an
      // obstacle cell clears only when the robot later raytraces through it with correct
      // localization, and cells placed beyond walls can never be raytraced through at all,
      // so they persist for the lifetime of the node and their inflation keeps bleeding
      // into traversable space.
      //
      // Left uncleared, the corruption outlives the goal that produced it and accumulates
      // monotonically across a campaign, which would make late cells measurably harder
      // than early ones for reasons that have nothing to do with the stack under test.
      clearCostmaps("recovery");

      recovery_start_time_ = now;  // reset timer for convergence wait
      recovery_stage_ = RecoveryStage::WAIT_CONVERGENCE;
      RCLCPP_INFO(get_logger(), "Recovery: known pose set, costmaps cleared; waiting for "
                  "AMCL convergence (timeout %.1fs).", recovery_timeout_sec_);
      break;
    }

    case RecoveryStage::WAIT_CONVERGENCE:
    {
      geometry_msgs::msg::PoseStamped gt_pose, est_pose;
      bool have_gt = lookupPose(gt_error_frame_, now, gt_pose);
      bool have_est = lookupPose(est_error_frame_, now, est_pose);

      if (have_gt && have_est && !needCorrection(gt_pose, est_pose)) {
        RCLCPP_INFO(get_logger(), "Recovery: AMCL converged after %.2fs.",
                    (now - recovery_start_time_).seconds());
        recovery_stage_ = RecoveryStage::DONE;
        break;
      }

      if ((now - recovery_start_time_).seconds() > recovery_timeout_sec_) {
        RCLCPP_WARN(get_logger(),
                    "Recovery: convergence timed out after %.1fs. Proceeding anyway.",
                    recovery_timeout_sec_);
        recovery_stage_ = RecoveryStage::DONE;
        break;
      }
      break;
    }

    case RecoveryStage::DONE:
    {
      RCLCPP_INFO(get_logger(), "Recovery: complete. Advancing to next goal.");
      idx_++;
      recovery_stage_ = RecoveryStage::IDLE;

      // Reset state for next goal
      next_allowed_send_ = now + rclcpp::Duration::from_seconds(dwell_sec_);
      goal_id_.uuid.fill(0);
      start_pose_ = geometry_msgs::msg::PoseStamped();
      pose_sequence_pending_ = false;
      pose_seq_stage_ = PoseSeqStage::IDLE;
      kidnap_stage_ = KidnapStage::IDLE;
      have_start_gt_ = false;
      have_kidnap_reference_ = false;
      kidnap_target_pose_ = unsampledKidnapPose_();
      kidnap_sampled_delay_sec_ = 0.0;

      if (idx_ >= goal_poses_.size()) {
        RCLCPP_INFO(get_logger(), "All goals done");
        rclcpp::shutdown();
      }
      break;
    }

    case RecoveryStage::IDLE:
    default:
      break;
  }
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(navlearn_benchmarks::EpisodeManager)