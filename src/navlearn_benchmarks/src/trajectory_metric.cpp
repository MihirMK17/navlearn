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

#include <cmath>
#include <chrono>
#include <deque>
#include <limits>

#include "navlearn_benchmarks/trajectory_metric.hpp"
#include "navlearn_benchmarks/navlearn_utils.hpp"

using namespace std::chrono_literals;

namespace navlearn_benchmarks{

TrajectoryMetric::TrajectoryMetric(const std::string &name) : rclcpp::Node(name)
    , have_last_odom_(false)
    , active_(false)
    , collision_count_(0)
    , was_in_contact_(false)
{
  odom_topic_     = declare_parameter<std::string>("odom_topic", "/bumperbot_controller/odom");
  episode_event_topic_  = declare_parameter<std::string>("episode_event_topic", "/navlearn/episode_event");
  trajectory_metric_topic_  = declare_parameter<std::string>("trajectory_metric_topic", "/navlearn/trajectory_metric");
  scan_topic_     = declare_parameter<std::string>("scan_topic", "/scan");
  gt_topic_       = declare_parameter<std::string>("gt_topic", "/bumperbot/ground_truth_pose");
  ds_thresh_m_    = declare_parameter<double>("jitter_guard", 0.002); // ignore micro-jitter
  max_gap_s_      = declare_parameter<double>("max_gap_dt", 0.5);        // skip giant dt gaps
  rpe_delta_s_    = declare_parameter<double>("rpe_delta", 1.0);      // time interval for RPE
  collision_topic_ = declare_parameter<std::string>("collision_topic", "/navlearn/collision");
  collision_scan_threshold_m_ = declare_parameter<double>("collision_scan_threshold_m", 0.15);

  // Exactly one collision source may count.
  //
  // Two independent detectors both incremented collision_count_: the scan threshold below,
  // and Empty messages on collision_topic_ published by collision_monitor from the Gazebo
  // contact sensor. A single physical collision that trips both — which is what a
  // collision normally does — was counted twice. Nothing in the pipeline could distinguish
  // one double-counted collision from two real ones.
  //
  // "scan" is the campaign default per the execution spec: it needs no contact sensor, so
  // it behaves identically across simulators and on hardware, and it is the detector the
  // positive control exercises.
  collision_source_ = declare_parameter<std::string>("collision_source", "scan");
  if (collision_source_ != "scan" && collision_source_ != "topic") {
    RCLCPP_FATAL(get_logger(),
      "collision_source='%s' is invalid; expected 'scan' or 'topic'.",
      collision_source_.c_str());
    throw std::runtime_error("invalid collision_source");
  }
  RCLCPP_INFO(get_logger(), "Collision source: %s (the other detector will not count)",
              collision_source_.c_str());

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

  have_gt_ = false;
  min_clearance_m_ = -1.0;

  // --- I/O ---
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SensorDataQoS(),
                std::bind(&TrajectoryMetric::odomCallback, this, std::placeholders::_1));

  episode_sub_ = create_subscription<navlearn_msgs::msg::EpisodeEvent>(episode_event_topic_, rclcpp::QoS(10),
                  std::bind(&TrajectoryMetric::episodeCallback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TrajectoryMetric::scanCallback, this, std::placeholders::_1));

  gt_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
      gt_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TrajectoryMetric::gtCallback, this, std::placeholders::_1));

  collision_sub_ = create_subscription<std_msgs::msg::Empty>(
      collision_topic_, rclcpp::QoS(10),
      std::bind(&TrajectoryMetric::collisionCallback, this, std::placeholders::_1));

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

  // Accumulate pose sample for ATE/RPE computation
  PoseSample s;
  s.x = odom->pose.pose.position.x;
  s.y = odom->pose.pose.position.y;
  s.t = rclcpp::Time(odom->header.stamp).seconds();
  odom_path_.push_back(s);
}

void TrajectoryMetric::episodeCallback(navlearn_msgs::msg::EpisodeEvent::ConstSharedPtr ev)
{
  using E = navlearn_msgs::msg::EpisodeEvent;

  if (ev->state == E::START) {
    active_ = true;
    goal_id_ = ev->goal_id;
    t_start_ = ev->header.stamp;

    // reset message
    msg_ = navlearn_msgs::msg::TrajectoryMetric();
    msg_.goal_id = goal_id_;
    msg_.path_length_m = 0.0;
    msg_.samples = 0;

    // reset safety metric accumulators
    min_clearance_m_ = std::numeric_limits<double>::max();
    collision_count_ = 0;
    was_in_contact_ = false;

    // reset ATE/RPE accumulators
    gt_path_.clear();
    odom_path_.clear();
    have_gt_ = false;

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

void TrajectoryMetric::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  if (!active_) return;

  float min_range = std::numeric_limits<float>::max();
  for (const float r : scan->ranges) {
    if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) continue;
    if (static_cast<double>(r) < min_clearance_m_) {
      min_clearance_m_ = static_cast<double>(r);
    }
    if (r < min_range) min_range = r;
  }

  // Scan-based collision detection (rising-edge).
  // Fires when any valid scan range <= collision_scan_threshold_m_.
  // Set collision_scan_threshold_m: 0 to disable.
  if (collision_source_ == "scan" && collision_scan_threshold_m_ > 0.0 &&
      min_range < std::numeric_limits<float>::max())
  {
    const bool in_contact = (static_cast<double>(min_range) <= collision_scan_threshold_m_);
    if (in_contact && !was_in_contact_) {
      ++collision_count_;
      RCLCPP_DEBUG(get_logger(), "Scan collision: range=%.3f m, count=%u", min_range, collision_count_);
    }
    was_in_contact_ = in_contact;
  }
}

void TrajectoryMetric::publishReport(const rclcpp::Time & t_end)
{
  // Header & frames
  msg_.header.stamp = t_end;
  msg_.header.frame_id = "odom";   // path length is integrated in odom frame
  msg_.goal_id = goal_id_;

  // Duration
  const rclcpp::Duration dur = t_end - t_start_;
  msg_.duration.sec     = static_cast<int32_t>(dur.seconds());
  msg_.duration.nanosec = static_cast<uint32_t>(dur.nanoseconds() % 1000000000LL);

  // ATE/RPE: compute from accumulated paths if GT available
  msg_.rpe_delta_s = rpe_delta_s_;
  if (have_gt_ && gt_path_.size() >= 2 && odom_path_.size() >= 2) {
    msg_.ate_rmse_m       = computeAte();
    msg_.ate_count        = static_cast<uint32_t>(odom_path_.size());
    msg_.rpe_trans_rmse_m = computeRpe();
    msg_.rpe_count        = static_cast<uint32_t>(gt_path_.size());
    msg_.ref_source       = navlearn::REF_GROUND_TRUTH;
    msg_.ref_frame        = "world";
  } else {
    msg_.ate_rmse_m       = std::numeric_limits<double>::quiet_NaN();
    msg_.ate_count        = 0;
    msg_.rpe_trans_rmse_m = std::numeric_limits<double>::quiet_NaN();
    msg_.rpe_count        = 0;
    msg_.ref_source       = navlearn::REF_NONE;
    msg_.ref_frame        = "";
  }

  // Safety metric: minimum observed scan range; -1.0 if no valid scan was received
  msg_.min_clearance_m = (min_clearance_m_ < std::numeric_limits<double>::max())
      ? min_clearance_m_ : -1.0;

  // Safety metric: collision count (0 if collision_monitor not running)
  msg_.collision_count = collision_count_;

  traj_pub_->publish(msg_);

  RCLCPP_INFO(
    get_logger(),
    "TrajectoryMetric: samples=%u path=%.3f m  ATE=%.4f m  RPE=%.4f m",
    msg_.samples, msg_.path_length_m,
    std::isnan(msg_.ate_rmse_m) ? -1.0 : msg_.ate_rmse_m,
    std::isnan(msg_.rpe_trans_rmse_m) ? -1.0 : msg_.rpe_trans_rmse_m
  );
}

void TrajectoryMetric::collisionCallback(std_msgs::msg::Empty::ConstSharedPtr /*msg*/)
{
  // Each Empty message = one rising-edge collision event (published by collision_monitor
  // for Gazebo, or by the Isaac Sim OmniGraph extension for Isaac Sim).
  //
  // Ignored unless this is the selected source. Counting here while the scan detector is
  // also counting double-counts every collision that trips both, which is most of them.
  if (!active_ || collision_source_ != "topic") return;
  ++collision_count_;
  RCLCPP_DEBUG(get_logger(), "Collision count this episode: %u", collision_count_);
}

void TrajectoryMetric::gtCallback(geometry_msgs::msg::TransformStamped::ConstSharedPtr msg)
{
  /// Accumulate ground-truth pose samples while an episode is active.
  if (!active_) return;
  PoseSample s;
  s.x = msg->transform.translation.x;
  s.y = msg->transform.translation.y;
  s.t = rclcpp::Time(msg->header.stamp).seconds();
  gt_path_.push_back(s);
  have_gt_ = true;
}

double TrajectoryMetric::computeAte() const
{
  /// Match each odom sample to the nearest-in-time GT sample; return RMSE of position errors.
  if (gt_path_.size() < 2 || odom_path_.size() < 2) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double sum_sq = 0.0;
  int count = 0;

  for (const auto & os : odom_path_) {
    double best_dt = std::numeric_limits<double>::max();
    size_t best_idx = 0;
    for (size_t i = 0; i < gt_path_.size(); ++i) {
      double dt = std::fabs(gt_path_[i].t - os.t);
      if (dt < best_dt) { best_dt = dt; best_idx = i; }
    }
    if (best_dt > 0.5) continue;  // skip if no nearby GT sample

    double ex = os.x - gt_path_[best_idx].x;
    double ey = os.y - gt_path_[best_idx].y;
    sum_sq += ex * ex + ey * ey;
    ++count;
  }

  if (count < 2) return std::numeric_limits<double>::quiet_NaN();
  return std::sqrt(sum_sq / static_cast<double>(count));
}

double TrajectoryMetric::computeRpe() const
{
  /// Compute RPE: relative translation errors over rpe_delta_s_ time intervals.
  if (gt_path_.size() < 2) return std::numeric_limits<double>::quiet_NaN();

  double sum_sq = 0.0;
  int count = 0;
  const double delta = rpe_delta_s_;

  auto nearest_odom = [&](double t) -> PoseSample {
    size_t best = 0;
    double best_dt = std::numeric_limits<double>::max();
    for (size_t k = 0; k < odom_path_.size(); ++k) {
      double dt = std::fabs(odom_path_[k].t - t);
      if (dt < best_dt) { best_dt = dt; best = k; }
    }
    return odom_path_[best];
  };

  for (size_t i = 0; i < gt_path_.size(); ++i) {
    size_t j = i + 1;
    while (j < gt_path_.size() && (gt_path_[j].t - gt_path_[i].t) < delta) ++j;
    if (j >= gt_path_.size()) break;

    double gt_dx = gt_path_[j].x - gt_path_[i].x;
    double gt_dy = gt_path_[j].y - gt_path_[i].y;

    auto oi = nearest_odom(gt_path_[i].t);
    auto oj = nearest_odom(gt_path_[j].t);
    double odom_dx = oj.x - oi.x;
    double odom_dy = oj.y - oi.y;

    double ex = odom_dx - gt_dx;
    double ey = odom_dy - gt_dy;
    sum_sq += ex * ex + ey * ey;
    ++count;
  }

  if (count < 2) return std::numeric_limits<double>::quiet_NaN();
  return std::sqrt(sum_sq / static_cast<double>(count));
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
