/**
 * Node: TrajectoryMetric
 * Role:
 *  - Measures the path length for each trajectory from start pose --> goal pose
 *  - Measures the Absolute Path Error over each trajectory   (planned)
 *  - Measures the Relative Pose Error i.e. RMS drift over each trajectory (planned)
 * 
 * Subscribes:
 *  - /bumperbot_controller/odom :  nav_msgs::msg::Odometry
 *  - /ground_truth/pose_topic  :  Motion Capture / Any Other ground truth (planned)
 * 
 * Publishes:
 *  - /navlearn/trajectory_metric : navlearn_msgs::msg::TrajectoryMetric
 * 
 * Parameters (planned):
 *  - odom_sub_ (topic) : Configurable for different odom models
 *  - ds_thresh_m_ (double) : Micro movement jitter guard
 *  - max_gap_s_ (double)   : Skipping giant dt gaps
 *  - rpe_delta_s_ (double) : [Placeholder] (planned)
 */

#ifndef TRAJECTORY_METRIC_HPP
#define TRAJECTORY_METRIC_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <navlearn_msgs/msg/trajectory_metric.hpp>
#include <navlearn_msgs/msg/episode_event.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <builtin_interfaces/msg/duration.hpp>

namespace navlearn_benchmarks{

class TrajectoryMetric : public rclcpp::Node
{
public:
    TrajectoryMetric(const std::string &name);

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<navlearn_msgs::msg::EpisodeEvent>::SharedPtr episode_sub_;
    rclcpp::Publisher<navlearn_msgs::msg::TrajectoryMetric>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Params ---
    std::string odom_topic_;
    std::string episode_event_topic_;
    std::string trajectory_metric_topic_;
    double ds_thresh_m_;
    double max_gap_s_;
    double rpe_delta_s_;

    // --- State ---
    nav_msgs::msg::Odometry last_odom_;
    bool have_last_odom_;

    bool active_;
    unique_identifier_msgs::msg::UUID episode_id_;
    rclcpp::Time t_start_;

    navlearn_msgs::msg::TrajectoryMetric msg_;

    void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr odom);

    void episodeCallback(navlearn_msgs::msg::EpisodeEvent::ConstSharedPtr ev);

    void timerCallback();

    void publishReport(const rclcpp::Time & t_end);
};
}

#endif