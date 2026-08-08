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

#ifndef GROUND_TRUTH_PUBLISHER_HPP
#define GROUND_TRUTH_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>

class GroundTruthTF : public rclcpp::Node
{
public:
  explicit GroundTruthTF(const std::string & name);

private:
  std::string fixed_frame_id_;
  std::string robot_frame_id_;
  std::string gazebo_groudnd_truth_topic_;
  std::string robot_ground_truth_topic_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr gazebo_gt_tf_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr robot_gt_tf_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void onGazeboGT(const tf2_msgs::msg::TFMessage::SharedPtr gazebo_gt);
};

#endif
