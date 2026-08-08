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

#ifndef LOCALIZATION_METRICS_HPP
#define LOCALIZATION_METRICS_HPP

#include <deque>
#include <fstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <cstdint>
#include <memory>
#include <iomanip>
#include <chrono>
#include <filesystem>
#include <sstream>
#include <cmath>
#include <boost/unordered_map.hpp>

#include <rclcpp/rclcpp.hpp>
#include <navlearn_msgs/msg/episode_event.hpp>                  // Topic: /navlearn/episode_event
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>   // Topic: /amcl_pose and /initialpose
#include <navlearn_msgs/msg/kidnap_event.hpp>                   // Topic: /navlearn/kidnap_event
#include <geometry_msgs/msg/transform_stamped.hpp>              // Topic: /bumperbot/ground_truth
#include <tf2_msgs/msg/tf_message.hpp>                          // Topics: /tf
#include <sensor_msgs/msg/laser_scan.hpp>                       // Topic: /scan
#include <nav_msgs/msg/odometry.hpp>                            // Topic: /bumperbot_controller/odom

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <unique_identifier_msgs/msg/uuid.hpp>

namespace navlearn_localization_metrics
{
class LocalizationMetrics : public rclcpp::Node
{
public:
  LocalizationMetrics(const std::string & name);
  // ~LocalizationMetrics();

private:
  // Miscellaneous Helpers
  std::string csv_path_;
  std::ofstream csv_;
  bool header_written_;

  static double getYaw(const geometry_msgs::msg::Quaternion & q);
  std::string uuid_to_string(const unique_identifier_msgs::msg::UUID & id);
  void trim(std::deque<rclcpp::Time> & q, const rclcpp::Time & now, double max_buffer);
  void write_csv(
    std::string_view metric_class, std::string_view metric_source,
    const boost::unordered_map<std::string, double> & metrics, const rclcpp::Time & t);

  void reset_episode_state();
  void reset_ttc_state();
  void reset_ttr_state();

  // Mission Outcome Helpers
  bool is_active_;
  std::string episode_topic;
  std::string goal_id_;
  uint8_t goal_result_;
  double nav_time_;
  int collision_count_;
  int recoveries_count_;
  boost::unordered_map<std::string, double> mission_outcome_;

  // Localization Quality Helpers
  struct PoseStatistics
  {
    rclcpp::Time t;
    double mean_x;
    double mean_y;
    double mean_yaw;
    double cov_xx;
    double cov_yy;
    double cov_yawyaw;
  };

  bool is_first_pose_;
  std::string amcl_topic_;
  PoseStatistics prev_pose_;
  PoseStatistics curr_pose_;
  uint32_t pose_monotonicity_violations_;

  PoseStatistics fillPoseStatistics(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const rclcpp::Time & t);

  // Localization Stability Helpers
  struct TFStatistics
  {
    rclcpp::Time t;
    double transform_x;
    double transform_y;
    double transform_yaw;
  };

  bool is_first_tf_;
  std::string tf_topic_;
  TFStatistics prev_tf_;
  TFStatistics curr_tf_;

  TFStatistics fillTFStatistics(
    const geometry_msgs::msg::TransformStamped & transform,
    const rclcpp::Time & t);
  boost::unordered_map<std::string, double> getLocalizationStability(
    const TFStatistics prev,
    const TFStatistics curr);

  // Localization Responsiveness Helpers
  bool is_first_gt_;
  std::string ground_truth_tf_topic_;
  TFStatistics ground_truth_statistic_;
  std::deque<TFStatistics> gt_stats_;
  TFStatistics gt_matched_;

  // Time-To-Converge
  bool bad_init_test_;
  std::string initial_pose_topic;
  bool ttc_arm_for_next_goal_;
  rclcpp::Time last_initialpose_time_;

  rclcpp::Time ttc_start_time_;
  rclcpp::Time ttc_converged_time_;
  rclcpp::Time ttc_complete_time_;
  double TTC;
  uint8_t ttc_outcome_;

  double ttc_hold_sec_;
  double ttc_timeout_sec_;

  enum class TtcState
  {
    IDLE,           // no episode active
    ARMED,          // initial pose received; waiting for episode START
    MEASURING,      // measuring time-to-converge
    CONVERGED,      // within thresholds for hold duration
    TIMED_OUT       // ttc_timeout_sec_ exceeded
  };
  TtcState ttc_state_;
  bool ttc_done_;       // set when measurement concluded (success or timeout)

  double error_x_threshold_;
  double error_y_threshold_;
  double error_yaw_threshold_;

  bool linear_interpolate(
    const std::deque<TFStatistics> & q, TFStatistics & out,
    rclcpp::Time t_req, double max_gap_s, double max_stale_s);
  void trim(std::deque<TFStatistics> & q, const rclcpp::Time & now, double max_buffer);

  // Time-To-Recover
  struct TTRStatistics
  {
    double goal_start_x_;
    double goal_start_y_;
    double goal_start_yaw_;
    bool have_goal_start_pose_;
  };

  bool kidnap_test_;
  std::string kidnap_topic_;

  rclcpp::Time ttr_start_time_;
  rclcpp::Time ttr_recovered_time_;
  rclcpp::Time ttr_complete_time_;
  double TTR;
  uint8_t ttr_outcome_;

  double kidnap_distance_;
  double kidnap_angle_;

  double ttr_hold_sec_;
  double ttr_timeout_sec_;

  enum class TtrState
  {
    IDLE,            // no kidnap attempted
    KIDNAPPED,       // kidnap teleport confirmed; recovery measuring started
    RECOVERING,      // robot recovering, not yet within thresholds
    RECOVERED,       // within thresholds for hold duration
    TIMED_OUT        // ttr_timeout_sec_ exceeded
  };
  TtrState ttr_state_;
  bool ttr_done_;           // set when measurement concluded (success or timeout)
  bool ttr_attempted_;      // guards against duplicate kidnap events per goal

  double ttr_error_pos_threshold_m_;
  double ttr_error_yaw_threshold_rad_;

  TTRStatistics ttr_stats_;

  // Input Validity Helpers
  // Lidar Health
  bool is_first_scan_;
  std::string scan_topic_;
  std::string scan_child_frame_;
  rclcpp::Time prev_scan_;
  rclcpp::Time curr_scan_;
  uint32_t scan_monotonicity_violations_;
  uint32_t scan_frame_mismatch_count_;
  std::deque<rclcpp::Time> scan_stamp_;
  double last_valid_beam_fraction_;  // fraction of finite, in-range beams in the most recent scan

  // Odometry  Health
  bool is_first_odom_;
  std::string odom_topic_;
  std::string odom_parent_frame_;
  std::string odom_child_frame_;
  rclcpp::Time prev_odom_;
  rclcpp::Time curr_odom_;
  uint32_t odom_monotonicity_violations_;
  uint32_t odom_frame_mismatch_count_;
  std::deque<rclcpp::Time> odom_stamp_;

  // TF Health
  std::string tf_parent_frame_;
  std::string tf_child_frame_;
  uint32_t tf_monotonicity_violations_;
  uint32_t tf_frame_mismatch_count_;
  uint32_t tf_gap_count_;
  std::deque<rclcpp::Time> tf_stamp_;

  // Subscriptions and Callbacks
  rclcpp::Subscription<navlearn_msgs::msg::EpisodeEvent>::SharedPtr episode_event_sub_;                 // For mission outcome and per goal metrics
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;        // For Localization Quality
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;                                    // For Localization Stability and Input Validity - TF
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr ground_truth_tf_sub_;           // For Localization Responsiveness
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;     // For Localization Responsiveness
  rclcpp::Subscription<navlearn_msgs::msg::KidnapEvent>::SharedPtr kidnap_event_sub_;                   // For Localization Responsiveness
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;                               // For Input Validity - Lidar
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;                                   // For Input Validity - Odometry
  rclcpp::TimerBase::SharedPtr timer_;                                                                  // For Input Validity - Rate

  void onEpisodeEvent(const navlearn_msgs::msg::EpisodeEvent & episode_event);
  void onAMCLPose(const geometry_msgs::msg::PoseWithCovarianceStamped & amcl_pose);
  void onTF(const tf2_msgs::msg::TFMessage::SharedPtr transforms);
  void onGroundTruthTF(const geometry_msgs::msg::TransformStamped gt_transform);
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose);
  void onKidnapEvent(const navlearn_msgs::msg::KidnapEvent & kidnap_event);
  void onScan(const sensor_msgs::msg::LaserScan & scan);
  void onOdometry(const nav_msgs::msg::Odometry & odom);
  void timerCallback();
};
}

#endif
