#include <cmath>
#include <chrono>
#include <deque>
#include <limits>

#include "navlearn_benchmarks/trajectory_metric.hpp"

using namespace std::chrono_literals;

namespace navlearn_benchmarks{

TrajectoryMetric::TrajectoryMetric(const std::string &name) : rclcpp::Node(name)
    , have_last_odom_(false)
    , active_(false)
{
  odom_topic_     = declare_parameter<std::string>("odom_topic", "/bumperbot_controller/odom");
  episode_event_topic_  = declare_parameter<std::string>("episode_event_topic", "/navlearn/episode_event");
  trajectory_metric_topic_  = declare_parameter<std::string>("trajectory_metric_topic", "/navlearn/trajectory_metric");
  ds_thresh_m_    = declare_parameter<double>("jitter_guard", 0.002); // ignore micro-jitter
  max_gap_s_      = declare_parameter<double>("max_gap_dt", 0.5);        // skip giant dt gaps
  rpe_delta_s_    = declare_parameter<double>("rpe_delta", 1.0);      // placeholder only

  if(ds_thresh_m_ < 0)
  {
    RCLCPP_FATAL(get_logger(), "Bad param: Jitter guard (%f) (m) has to be > 0", ds_thresh_m_);
    rclcpp::shutdown();
  }
  if(max_gap_s_ < 0)
  {
    RCLCPP_FATAL(get_logger(), "Bad param: Jitter guard (%f) (sec) has to be > 0", max_gap_s_);
    rclcpp::shutdown();
  }

  RCLCPP_INFO(get_logger(), "TrajectoryMetric config: Jitter Guard (distance):%.2f, Jitter Guard (time):%.2f",
              ds_thresh_m_, max_gap_s_);

  // --- I/O ---
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SensorDataQoS(),
                std::bind(&TrajectoryMetric::odomCallback, this, std::placeholders::_1));

  episode_sub_ = create_subscription<navlearn_msgs::msg::EpisodeEvent>(episode_event_topic_, rclcpp::QoS(10),
                  std::bind(&TrajectoryMetric::episodeCallback, this, std::placeholders::_1));

  traj_pub_ = create_publisher<navlearn_msgs::msg::TrajectoryMetric>(trajectory_metric_topic_, rclcpp::QoS(10));

  timer_ = create_wall_timer(100ms, std::bind(&TrajectoryMetric::timerCallback, this));
}

// --- Callbacks ---
void TrajectoryMetric::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  if (!active_) return;

  // First valid sample in an episode
  if (!have_last_odom_) {
    last_odom_ = *odom;
    have_last_odom_ = true;
    ++msg_.samples;
    return;
  }

  // Time gap guard
  const double dt = (rclcpp::Time(odom->header.stamp) - rclcpp::Time(last_odom_.header.stamp)).seconds();
  if (dt <= 0.0 || dt > max_gap_s_) {
    last_odom_ = *odom;
    return;
  }

  // Incremental distance in odom
  const double dx = odom->pose.pose.position.x - last_odom_.pose.pose.position.x;
  const double dy = odom->pose.pose.position.y - last_odom_.pose.pose.position.y;
  const double ds = std::hypot(dx, dy);

  if (ds > ds_thresh_m_) {
    msg_.path_length_m += ds;
  }

  last_odom_ = *odom;
  ++msg_.samples;
}

void TrajectoryMetric::episodeCallback(navlearn_msgs::msg::EpisodeEvent::ConstSharedPtr ev)
{
  using E = navlearn_msgs::msg::EpisodeEvent;

  if (ev->state == E::START) {
    active_ = true;
    episode_id_ = ev->episode_id;
    t_start_ = ev->header.stamp;

    // reset message
    msg_ = navlearn_msgs::msg::TrajectoryMetric();
    msg_.episode_id = episode_id_;
    msg_.path_length_m = 0.0;
    msg_.samples = 0;

    // ensure we reseed odom integration
    have_last_odom_ = false;
    return;
  }

  if (ev->state == E::END) {
    if (!active_) return;
    publishReport(ev->header.stamp);
    active_ = false;
  }
}

void TrajectoryMetric::timerCallback() {
  // Not publishing rolling updates for now (only final snapshot on END).
  // Keep timer for future periodic reporting if needed.
}

void TrajectoryMetric::publishReport(const rclcpp::Time & t_end)
{
  // Header & frames
  msg_.header.stamp = t_end;
  msg_.header.frame_id = "odom";   // path length is integrated in odom frame
  msg_.episode_id = episode_id_;

  // Duration
  const rclcpp::Duration dur = t_end - t_start_;
  msg_.duration.sec     = static_cast<int32_t>(dur.seconds());
  msg_.duration.nanosec = static_cast<uint32_t>(dur.nanoseconds() % 1000000000LL);

  // Placeholders for ATE/RPE (not available yet)
  msg_.ate_rmse_m        = std::numeric_limits<double>::quiet_NaN();
  msg_.ate_count         = 0;
  msg_.rpe_delta_s       = rpe_delta_s_;
  msg_.rpe_trans_rmse_m  = std::numeric_limits<double>::quiet_NaN();
  msg_.rpe_count         = 0;
  // Use literal 0 to avoid enum name churn if your .msg doesn't define REF_* constants
  msg_.ref_source        = 0;         // REF_NONE
  msg_.ref_frame         = "";

  traj_pub_->publish(msg_);

  RCLCPP_INFO(
    get_logger(),
    "TrajectoryMetric: samples=%u path=%.3f m",
    msg_.samples, msg_.path_length_m
  );
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navlearn_benchmarks::TrajectoryMetric>("trajectory_metric");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
