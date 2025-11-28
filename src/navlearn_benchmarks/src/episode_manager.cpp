#include <chrono>
#include <vector>
#include <random>
#include <algorithm>

#include "navlearn_benchmarks/episode_manager.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace navlearn_benchmarks{

EpisodeManager::EpisodeManager(const rclcpp::NodeOptions & options)
  : rclcpp::Node("episode_manager", options)
  , idx_(0)
  , active_(false)
  , stamp_received_(0,0,RCL_ROS_TIME)
{
  dwell_sec_ = this->declare_parameter<double>("dwell_sec", 5.0);
  goal_poses_x_ = this->declare_parameter<std::vector<double>>("goal_poses_x", {});
  goal_poses_y_ = this->declare_parameter<std::vector<double>>("goal_poses_y", {});
  goal_poses_yaw_ = this->declare_parameter<std::vector<double>>("goal_poses_yaw", {});
  goals_num_ = this->declare_parameter<int>("goals_num", 4);
  episodes_num_ = this->declare_parameter<int>("episodes_num", 1);
  action_server_ = this->declare_parameter<std::string>("action_server", "/navigate_to_pose");
  episode_pub_topic_ = this->declare_parameter<std::string>("episode_pub_topic", "/navlearn/episode_event");
  fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "map");
  robot_frame_ = this->declare_parameter<std::string>("robot_frame", "base_link");

  next_allowed_send_ = this->get_clock()->now();

  client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, action_server_);
  episode_pub_ = create_publisher<navlearn_msgs::msg::EpisodeEvent>(episode_pub_topic_, rclcpp::QoS(10).reliable());
  timer_ = create_wall_timer(200ms, std::bind(&EpisodeManager::timerCallback, this));

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  loadGoals();
}
// ---------- helpers ----------
bool EpisodeManager::sampleStartPoseAt(const rclcpp::Time & t, geometry_msgs::msg::PoseStamped & out) {
  // Try TF at event time; fallback to latest if needed.
  try {
    const auto tr = tf_buffer_->lookupTransform(fixed_frame_, robot_frame_, t, 150ms);
    out.header = tr.header;
    out.pose.position.x = tr.transform.translation.x;
    out.pose.position.y = tr.transform.translation.y;
    out.pose.position.z = tr.transform.translation.z;
    out.pose.orientation = tr.transform.rotation;
    return true;
  } catch (...) {
    try {
      const auto tr_latest = tf_buffer_->lookupTransform(fixed_frame_, robot_frame_, rclcpp::Time(0,0,RCL_ROS_TIME));
      out.header = tr_latest.header;
      out.pose.position.x = tr_latest.transform.translation.x;
      out.pose.position.y = tr_latest.transform.translation.y;
      out.pose.position.z = tr_latest.transform.translation.z;
      out.pose.orientation = tr_latest.transform.rotation;
      return true; // fallback used
    } catch (...) {
      // Leave out zero-initialized if TF completely unavailable
      out = geometry_msgs::msg::PoseStamped();
      return false;
    }
  }
}

void EpisodeManager::loadGoals() {
  if(goal_poses_x_.size() != goal_poses_y_.size() || goal_poses_x_.size() != goal_poses_yaw_.size())
  {
    RCLCPP_FATAL(get_logger(), "Goal Poses length mismatch!");
    rclcpp::shutdown();
    return;
  }

  // Load goals from yaml file
  goal_poses_.reserve(goal_poses_x_.size());

  for (std::size_t i = 0; i < goal_poses_x_.size(); ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = fixed_frame_;
    p.header.stamp = this->get_clock()->now();
    p.pose.position.x = goal_poses_x_[i];
    p.pose.position.y = goal_poses_y_[i];
    p.pose.position.z = 0.0;
    tf2::Quaternion q; 
    q.setRPY(0,0, goal_poses_yaw_[i] * M_PI / 180.0);
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    goal_poses_.push_back(p);
  }
}

void EpisodeManager::timerCallback() {
  if (!client_->wait_for_action_server(0s)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "NavigateToPose server not ready");
    return;
  }

  const auto t = this->get_clock()->now();
  if (!active_ && idx_ < goal_poses_.size()) {
    if (t >= next_allowed_send_) {
      sendGoal();
    } else {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
        "Cooling down... %.2fs left", (next_allowed_send_ - t).seconds());
    }
  }
}

void EpisodeManager::sendGoal() {
  nav2_msgs::action::NavigateToPose::Goal goal_msg;
  goal_msg.pose = goal_poses_[idx_];

  auto opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  opts.goal_response_callback = std::bind(&EpisodeManager::onGoalResponse, this, _1);
  opts.feedback_callback      = std::bind(&EpisodeManager::onFeedback, this, _1, _2);
  opts.result_callback        = std::bind(&EpisodeManager::onResult, this, _1);

  client_->async_send_goal(goal_msg, opts);
  RCLCPP_INFO(get_logger(), "Sent goal %zu: (x=%.2f, y=%.2f)",
              idx_, goal_poses_[idx_].pose.position.x, goal_poses_[idx_].pose.position.y);
}

void EpisodeManager::onGoalResponse(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & gh) {
  if (!gh) {
    RCLCPP_ERROR(get_logger(), "Goal %zu rejected by NavigateToPose", idx_);
    idx_++; // Skip to next goal
    return;
  }

  RCLCPP_INFO(get_logger(), "Goal %zu accepted by NavigateToPose", idx_);
  active_ = true;

  // Store episode_id (copy from action goal id)
  const auto goal_id = gh->get_goal_id();
  std::copy(goal_id.begin(), goal_id.end(), episode_id_.uuid.begin());

  // Build START event
  navlearn_msgs::msg::EpisodeEvent ev;
  ev.header.stamp = this->get_clock()->now();  // authoritative start event time
  ev.state  = navlearn_msgs::msg::EpisodeEvent::START;
  ev.result = navlearn_msgs::msg::EpisodeEvent::RESULT_NA;
  ev.episode_id = episode_id_;
  ev.goal_pose  = goal_poses_[idx_];

  // Fill stamp_received and sample start_pose at accept time
  ev.stamp_received = ev.header.stamp;
  (void)sampleStartPoseAt(ev.header.stamp, start_pose_);
  ev.start_pose = start_pose_;  // even if zero-init (TF failed), we publish it

  // Clear terminate/nav_time for START
  ev.stamp_terminated.sec = 0;
  ev.stamp_terminated.nanosec = 0;
  ev.nav_time.sec = 0;
  ev.nav_time.nanosec = 0;

  // Remember t_start (for safety) even though you now carry it in the event
  stamp_received_ = rclcpp::Time(ev.stamp_received);

  episode_pub_->publish(ev);
  RCLCPP_INFO(get_logger(), "START episode for goal %zu (episode_id set)", idx_);
}

void EpisodeManager::onFeedback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &,
                const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  // Optional: track recoveries/distance_remaining internally if you later extend EpisodeEvent.
  (void)feedback;
}

void EpisodeManager::onResult(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result) {
  // Build END event
  navlearn_msgs::msg::EpisodeEvent ev;
  ev.header.stamp = this->get_clock()->now();        // authoritative end event time
  ev.state = navlearn_msgs::msg::EpisodeEvent::END;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      ev.result = navlearn_msgs::msg::EpisodeEvent::RESULT_SUCCEEDED; break;
    case rclcpp_action::ResultCode::CANCELED:
      ev.result = navlearn_msgs::msg::EpisodeEvent::RESULT_CANCELED;  break;
    case rclcpp_action::ResultCode::ABORTED:
    default:
      ev.result = navlearn_msgs::msg::EpisodeEvent::RESULT_FAILED;    break;
  }

  ev.episode_id = episode_id_;
  ev.goal_pose  = goal_poses_[idx_];

  // Carry forward the start pose captured at accept time (do NOT resample here)
  ev.start_pose = start_pose_;

  // Fill stamps + nav_time (Humble-safe conversion)
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

  episode_pub_->publish(ev);

  const char *res_str =
    (ev.result == navlearn_msgs::msg::EpisodeEvent::RESULT_SUCCEEDED) ? "SUCCEEDED" :
    (ev.result == navlearn_msgs::msg::EpisodeEvent::RESULT_CANCELED)  ? "CANCELED"  : "FAILED";

  RCLCPP_INFO(get_logger(), "END goal %zu result=%s (%u) nav_time=%.3fs\n",
              idx_, res_str, static_cast<unsigned>(ev.result),
              (rclcpp::Time(ev.stamp_terminated) - stamp_received_).seconds());

  // Prepare next goal
  active_ = false;
  idx_++;
  next_allowed_send_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(dwell_sec_);

  // Reset per-episode state
  episode_id_.uuid.fill(0);
  // stamp_received_ = rclcpp::Time(0,0,RCL_ROS_TIME);
  start_pose_ = geometry_msgs::msg::PoseStamped();

  if (idx_ >= goal_poses_.size()) {
    RCLCPP_INFO(get_logger(), "All goals done");
    rclcpp::shutdown();
  }
}
}

RCLCPP_COMPONENTS_REGISTER_NODE(navlearn_benchmarks::EpisodeManager)
