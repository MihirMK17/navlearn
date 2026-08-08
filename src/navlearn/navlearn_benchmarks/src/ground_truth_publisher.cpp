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

#include <navlearn_benchmarks/ground_truth_publisher.hpp>

using namespace std::placeholders;

GroundTruthTF::GroundTruthTF(const std::string & name)
: Node(name)
{
  fixed_frame_id_ = this->declare_parameter<std::string>("fixed_frame", "map");
  robot_frame_id_ = this->declare_parameter<std::string>("robot_frame", "bumperbot");
  gazebo_groudnd_truth_topic_ = this->declare_parameter<std::string>(
    "gazebo_ground_truth_topic",
    "/tf_gt");
  robot_ground_truth_topic_ = this->declare_parameter<std::string>(
    "robot_ground_truth_topic",
    "/" + robot_frame_id_ +
    "/ground_truth_pose");

  gazebo_gt_tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
    gazebo_groudnd_truth_topic_, 10,
    std::bind(&GroundTruthTF::onGazeboGT, this, _1));
  robot_gt_tf_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>(
    robot_ground_truth_topic_, 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  RCLCPP_INFO(
    get_logger(), "Publishing Ground Truth Robot Pose to topic: %s",
    robot_ground_truth_topic_.c_str());
}

void GroundTruthTF::onGazeboGT(const tf2_msgs::msg::TFMessage::SharedPtr gazebo_gt)
{
  geometry_msgs::msg::TransformStamped robot_gt_tf_msg_;

  for (const auto & transform : gazebo_gt->transforms) {
    if (transform.child_frame_id == robot_frame_id_) {
      auto stamp = transform.header.stamp;
      if (stamp.sec == 0 && stamp.nanosec == 0) {
        stamp = this->get_clock()->now();
      }
      robot_gt_tf_msg_.header.stamp = stamp;
      robot_gt_tf_msg_.header.frame_id = fixed_frame_id_;
      robot_gt_tf_msg_.child_frame_id = "base_footprint_gt";
      robot_gt_tf_msg_.transform = transform.transform;

      robot_gt_tf_pub_->publish(robot_gt_tf_msg_);
      tf_broadcaster_->sendTransform(robot_gt_tf_msg_);
      break;
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GroundTruthTF>("ground_truth_publisher");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
