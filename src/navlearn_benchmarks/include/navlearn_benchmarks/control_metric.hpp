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
 * Node: ControlMetric
 * Role:
 *  - Stores 2 sec buffer data for robot, wheel and command velocities
 *  - Computes control metrics using by matching current timestamp and corresponding velocity using zero-order hold / linear interpolation
 *  - Publishes control metrics
 * 
 * Subscribes:
 *  - /cmd_vel  : geometry_msgs::msg::Twist
 *  - /joint_states : sensor_msgs::msg::JointState
 *  - /bumperbot_controller/odom  : nav_msgs::msg::Odometry
 *  - /navlearn/episode_event : navlearn_msgs::msg::EpisodeEvent
 * 
 * Publishes:
 *  - /navlearn/control_metric  : navlearn_msgs::msg::ControlMetric
 * 
 * Parameters (planned):
 *  - odom_sub_ (topic) : Configurable for different odom models
 *  - wheel_radius_ (double)  : Robot wheel radius
 *  - wheel_separation_ (double)  : Robot wheel separation
 *  - buffer_span_s_ (double) : Buffer span to store incomming velocities
 *  - v_max_ (double) : Robot maximum linear velocity
 *  - w_max_ (double) : Robot maximum angular velocity
 *  - sat_tol_ (double) : % saturation limit
 *  - eps_v_ (double) : Non-zero Robot base_link linear velocity (NaN guard)
 *  - eps_w_ (double) : Non-zero Robot base_link angular velocity (NaN guard)
 */

#ifndef CONTROL_METRIC_HPP
#define CONTROL_METRIC_HPP

#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <navlearn_msgs/msg/control_metric.hpp>
#include <navlearn_msgs/msg/episode_event.hpp>

#include <memory>

namespace navlearn_benchmarks{

struct Sample {
  rclcpp::Time t;
  double x;
};

class ControlMetric : public rclcpp::Node
{
public:
    ControlMetric(const std::string &name);

private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<navlearn_msgs::msg::EpisodeEvent>::SharedPtr episode_sub_;
    rclcpp::Publisher<navlearn_msgs::msg::ControlMetric>::SharedPtr control_metric_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::deque<Sample> buf_v_cmd_;
    std::deque<Sample> buf_w_cmd_;
    std::deque<Sample> buf_v_base_;
    std::deque<Sample> buf_w_base_;
    std::deque<Sample> buf_v_wheel_;
    std::deque<Sample> buf_w_wheel_;

    // Config
    double buffer_span_s_;
    double wheel_radius_;
    double wheel_separation_;

    double v_max_;
    double w_max_;
    double sat_tol_;

    double eps_v_;
    double eps_w_;

    double lambda_;
    double control_energy_;

    int slip_buffer_max_size_;
    double max_stale_dt_s_;

    std::string controller_topic_;
    std::string odom_topic_;
    std::string joint_states_topic_;
    std::string episode_event_topic_;
    std::string control_metric_topic_;

    std::string wheel_left_joint_;
    std::string wheel_right_joint_;

    // State
    bool active_;
    unique_identifier_msgs::msg::UUID goal_id_;
    rclcpp::Time t_start_;
    uint32_t count_;
    uint32_t sat_v_count_;
    uint32_t sat_w_count_;
    double sum_ev2_, sum_ew2_;
    double sum_s_, sum_s2_;
    std::deque<double> slip_abs_buf_;

    void resetAccumulators();

    void trim(std::deque<Sample>& q, const rclcpp::Time &now);

    void velCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr vel);

    void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr joint);

    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom);

    bool zero_order_hold(const std::deque<Sample> &q,
                       const rclcpp::Time &tk,
                       double max_stale_s,
                       double &out);

    bool sample_interp_or_hold(const std::deque<Sample> &q,
                             rclcpp::Time tk,
                             double &out,
                             double max_gap_s = 0.25,
                             double max_stale_s = 0.25);

    void timerCallback();

    void episodeCallback(const navlearn_msgs::msg::EpisodeEvent::ConstSharedPtr episode);

    void publishReport(const rclcpp::Time& t_end);
};
}

#endif