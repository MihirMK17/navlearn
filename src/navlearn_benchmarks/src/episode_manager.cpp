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

#include "navlearn_benchmarks/episode_manager.hpp"
#include "navlearn_benchmarks/navlearn_utils.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace navlearn_benchmarks{

EpisodeManager::EpisodeManager(const rclcpp::NodeOptions & options)
  : rclcpp::Node("episode_manager", options)
  , idx_(0)
  , active_(false)
  , goal_request_inflight_(false)
  , stamp_received_(0,0,RCL_ROS_TIME)
  , qos_profile_sub(10)
  , is_first_bad_init_(false)
  , pose_sequence_pending_(false)
  , pose_seq_stage_(PoseSeqStage::IDLE)
  , kidnap_stage_(KidnapStage::IDLE)
  , kidnap_sampled_delay_sec_(0.0)
  , have_start_gt_(false)
  , have_amcl_pose_(false)
  , reinit_in_flight_(false)
  , last_reinit_time_(0,0,RCL_ROS_TIME)
{
  dwell_sec_ = this->declare_parameter<double>("dwell_sec", 5.0);
  goal_source_ = this->declare_parameter<std::string>("goal_source", "map_random");
  goal_seed_ = static_cast<unsigned int>(this->declare_parameter<int>("goal_seed", 42));
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

  initial_pose_seed_ = declare_parameter<int>("initial_pose_seed", 1337);

  set_pose_service_ = this->declare_parameter<std::string>("set_pose_service", "/gz_set_pose_server/set_entity_pose");
  entity_name_ = this->declare_parameter<std::string>("entity_name", "bumperbot");

  kidnap_enabled_ = this->declare_parameter<bool>("kidnap_enabled", false);

  // deprecated (kept only so old configs don’t crash). Not used in logic.
  kidnap_delay_sec_ = this->declare_parameter<double>("kidnap_delay_sec", 0.0);

  kidnap_delay_min_sec_ = this->declare_parameter<double>("kidnap_delay_min_sec", 1.0);
  kidnap_delay_max_sec_ = this->declare_parameter<double>("kidnap_delay_max_sec", 5.0);
  kidnap_min_travel_m_  = this->declare_parameter<double>("kidnap_min_travel_m", 0.25);
  kidnap_distance_m_    = this->declare_parameter<double>("kidnap_distance_m", 0.20);
  kidnap_max_distance_m_ = this->declare_parameter<double>("kidnap_max_distance_m", 3.0);
  kidnap_z_             = this->declare_parameter<double>("kidnap_z", 0.01);
  kidnap_seed_          = this->declare_parameter<int>("kidnap_seed", 4242);
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

    std::mt19937 gen(static_cast<uint32_t>(goal_seed_));
    std::uniform_int_distribution<int> dist(0, width * height - 1);

    const int max_tries_per_goal = 1000;

    for (int i = 0; i < goals_num_; ++i) {
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

      std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);
      double yaw = dist_theta(gen);

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
  std::mt19937 gen(static_cast<uint32_t>(initial_pose_seed_ + static_cast<int>(idx_)));
  std::uniform_real_distribution<double> dist_lin(-bad_init_lin_range_m_, bad_init_lin_range_m_);
  std::uniform_real_distribution<double> dist_yaw(-bad_init_yaw_range_rad_, bad_init_yaw_range_rad_);

  const double dx = dist_lin(gen);
  const double dy = dist_lin(gen);
  const double dyaw = dist_yaw(gen);

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

  const bool need_correct =
    (idx_ > 0) && have_est && needCorrection(gt_pose, est_pose);

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

bool EpisodeManager::sampleKidnapPoseFromMap_(const geometry_msgs::msg::PoseStamped & ref, geometry_msgs::msg::Pose & out_pose)
{
  if (!have_map_) return false;

  const auto & info = latest_map_.info;
  const int width   = static_cast<int>(info.width);
  const int height  = static_cast<int>(info.height);
  const double res  = info.resolution;
  const auto & data = latest_map_.data;

  if (width <= 0 || height <= 0 || data.empty()) return false;

  std::mt19937 gen(static_cast<uint32_t>(kidnap_seed_ + static_cast<int>(idx_)));
  std::uniform_int_distribution<int> dist_cell(0, width * height - 1);
  std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);

  for (int attempt = 0; attempt < kidnap_max_sample_tries_; ++attempt)
  {
    int idx = dist_cell(gen);
    if (idx < 0 || idx >= static_cast<int>(data.size())) continue;

    if (data[idx] != 0) continue;

    int row = idx / width;
    int col = idx % width;

    const double x = info.origin.position.x + (col + 0.5) * res;
    const double y = info.origin.position.y + (row + 0.5) * res;

    if (inExclusionZone(x, y)) continue;
    if (!hasClearanceCell(col, row, width, height, res, data)) continue;

    const double d = std::hypot(x - ref.pose.position.x, y - ref.pose.position.y);
    if (kidnap_distance_m_ > 0.0 && d < kidnap_distance_m_) continue;
    if (kidnap_max_distance_m_ > 0.0 && d > kidnap_max_distance_m_) continue;

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
    callReinitGlobalLocalization();
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

  geometry_msgs::msg::Pose target;
  if (!sampleKidnapPoseFromMap_(ref_pose, target)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Kidnap sample failed (no valid pose found)");
    return;
  }

  kidnap_attempt_id_ = makeUUID_(static_cast<uint32_t>(kidnap_seed_ + static_cast<int>(idx_) * 101));
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

  std::mt19937 gen(static_cast<uint32_t>(kidnap_seed_ + static_cast<int>(idx_) * 17));
  std::uniform_real_distribution<double> dist_delay(lo, hi);
  kidnap_sampled_delay_sec_ = dist_delay(gen);

  kidnap_stage_ = KidnapStage::IDLE;
}

void EpisodeManager::onFeedback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &,
                const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  (void)feedback;
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
  if (ev.result == navlearn_msgs::msg::EpisodeEvent::RESULT_SUCCEEDED &&
      ev.terminal.gt_available &&
      ev.terminal.true_distance_to_goal_m > end_error_pos_threshold_m_) {
    RCLCPP_WARN(get_logger(),
                "  FALSE SUCCESS: reported SUCCEEDED but ground truth is %.3f m from the "
                "goal (threshold %.3f m); the filter believed it was %.3f m away.",
                ev.terminal.true_distance_to_goal_m, end_error_pos_threshold_m_,
                ev.terminal.estimated_distance_to_goal_m);
  }

  active_ = false;

  if (ev.result != navlearn_msgs::msg::EpisodeEvent::RESULT_SUCCEEDED &&
      (bad_init_test_ || kidnap_enabled_)) {
    // Recovery: teleport to current goal, re-localize, then advance
    RCLCPP_WARN(get_logger(), "Goal %zu FAILED during TTC/TTR experiment. Initiating recovery.", idx_);
    recovery_stage_ = RecoveryStage::TELEPORT_PENDING;
    recovery_start_time_ = this->get_clock()->now();
    initiateRecoveryTeleport(goal_poses_[idx_]);
  } else {
    idx_++;
    next_allowed_send_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(dwell_sec_);

    goal_id_.uuid.fill(0);
    start_pose_ = geometry_msgs::msg::PoseStamped();

    pose_sequence_pending_ = false;
    pose_seq_stage_ = PoseSeqStage::IDLE;

    kidnap_stage_ = KidnapStage::IDLE;
    have_start_gt_ = false;
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
    RCLCPP_ERROR(get_logger(), "Recovery: setEntityPose_ failed. Skipping recovery.");
    recovery_stage_ = RecoveryStage::DONE;
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
      forceReinitGlobalLocalization();

      // Publish correct initial pose at teleport location
      geometry_msgs::msg::PoseWithCovarianceStamped correct_pose =
          buildPoseWithCovariance(goal_poses_[idx_], correct_cov_xy_, correct_cov_yaw_);
      sendInitialPoseRequest(correct_pose);

      recovery_start_time_ = now;  // reset timer for convergence wait
      recovery_stage_ = RecoveryStage::WAIT_CONVERGENCE;
      RCLCPP_INFO(get_logger(), "Recovery: waiting for AMCL convergence (timeout %.1fs).",
                  recovery_timeout_sec_);
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