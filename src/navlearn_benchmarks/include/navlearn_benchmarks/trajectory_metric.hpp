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

/**
 * Node: TrajectoryMetric
 * Role:
 *  - Measures path length, ATE, RPE, min clearance, and collision count per episode.
 *
 * Subscribes:
 *  - odom_topic          : nav_msgs::msg::Odometry
 *  - scan_topic          : sensor_msgs::msg::LaserScan
 *  - gt_topic            : geometry_msgs::msg::TransformStamped (ground truth)
 *  - collision_topic     : std_msgs::msg::Empty (one msg = one collision event)
 *  - episode_event_topic : navlearn_msgs::msg::EpisodeEvent
 *
 * Publishes:
 *  - trajectory_metric_topic : navlearn_msgs::msg::TrajectoryMetric
 *
 * Parameters:
 *  - odom_topic (string)       : [default: /bumperbot_controller/odom]
 *  - scan_topic (string)       : [default: /scan]
 *  - gt_topic (string)         : [default: /bumperbot/ground_truth]
 *  - collision_topic (string)  : [default: /navlearn/collision]
 *  - jitter_guard (double)     : Minimum distance step to count [m]
 *  - max_gap_dt (double)       : Skip odom samples with dt > this [s]
 *  - rpe_delta (double)        : RPE time interval [s]
 */

#ifndef TRAJECTORY_METRIC_HPP
#define TRAJECTORY_METRIC_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <navlearn_msgs/msg/trajectory_metric.hpp>
#include <navlearn_msgs/msg/episode_event.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <vector>

namespace navlearn_benchmarks{

struct PoseSample {
    double x;
    double y;
    double t;  // seconds since epoch
};

class TrajectoryMetric : public rclcpp::Node
{
public:
    TrajectoryMetric(const std::string &name);

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<navlearn_msgs::msg::EpisodeEvent>::SharedPtr episode_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr gt_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr collision_sub_;
    rclcpp::Publisher<navlearn_msgs::msg::TrajectoryMetric>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Params ---
    std::string odom_topic_;
    std::string episode_event_topic_;
    std::string trajectory_metric_topic_;
    std::string scan_topic_;
    std::string gt_topic_;
    std::string collision_topic_;
    double collision_scan_threshold_m_;
    double ds_thresh_m_;
    double max_gap_s_;
    double rpe_delta_s_;

    // --- State ---
    nav_msgs::msg::Odometry last_odom_;
    bool have_last_odom_;

    bool active_;
    unique_identifier_msgs::msg::UUID goal_id_;
    rclcpp::Time t_start_;
    double min_clearance_m_;
    uint32_t collision_count_;
    bool was_in_contact_;

    navlearn_msgs::msg::TrajectoryMetric msg_;

    // --- Ground truth ATE/RPE ---
    std::vector<PoseSample> gt_path_;   // GT poses accumulated per episode
    std::vector<PoseSample> odom_path_; // Odom poses accumulated per episode (in odom frame)
    bool have_gt_;                       // any GT received this episode

    void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr odom);

    void episodeCallback(navlearn_msgs::msg::EpisodeEvent::ConstSharedPtr ev);

    void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

    void gtCallback(geometry_msgs::msg::TransformStamped::ConstSharedPtr msg);

    void collisionCallback(std_msgs::msg::Empty::ConstSharedPtr msg);

    void timerCallback();

    void publishReport(const rclcpp::Time & t_end);

    double computeAte() const;
    double computeRpe() const;
};
}

#endif