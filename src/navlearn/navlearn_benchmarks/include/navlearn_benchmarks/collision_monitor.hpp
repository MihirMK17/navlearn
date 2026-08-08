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
 * Node: CollisionMonitor
 * Role:
 *  - Bridges Gazebo Ignition contact sensor events to the sim-agnostic
 *    /navlearn/collision ROS 2 topic used by trajectory_metric.
 *  - Performs rising-edge detection: one std_msgs/Empty is published per
 *    contact event (False->True transition), not per physics step.
 *  - For Isaac Sim: this node is NOT used. The Isaac Sim OmniGraph extension
 *    script publishes directly to collision_topic with the same semantics.
 *
 * Subscribes:
 *  - contacts_topic (ros_gz_interfaces/msg/Contacts) — bridged from Ignition
 *    via ros_gz_bridge/parameter_bridge
 *
 * Publishes:
 *  - collision_topic (std_msgs/Empty) — one message per collision event
 *
 * Parameters:
 *  - contacts_topic (string): ROS 2 topic from ros_gz_bridge
 *      [default: /world/small_house/model/bumperbot/link/base_link/sensor/contact_sensor/contact]
 *  - collision_topic (string): abstracted output topic
 *      [default: /navlearn/collision]
 */

#ifndef COLLISION_MONITOR_HPP
#define COLLISION_MONITOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <std_msgs/msg/empty.hpp>

namespace navlearn_benchmarks {

class CollisionMonitor : public rclcpp::Node
{
public:
  explicit CollisionMonitor(const std::string & name);

private:
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contacts_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr collision_pub_;

  std::string contacts_topic_;
  std::string collision_topic_;

  bool was_in_contact_;

  void contactsCallback(ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg);
};

}  // namespace navlearn_benchmarks

#endif  // COLLISION_MONITOR_HPP
