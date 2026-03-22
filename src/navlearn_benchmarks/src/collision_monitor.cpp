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

#include "navlearn_benchmarks/collision_monitor.hpp"

namespace navlearn_benchmarks {

CollisionMonitor::CollisionMonitor(const std::string & name)
: rclcpp::Node(name)
, was_in_contact_(false)
{
  contacts_topic_ = declare_parameter<std::string>(
    "contacts_topic",
    "/world/small_house/model/bumperbot/link/base_link/sensor/contact_sensor/contact");

  collision_topic_ = declare_parameter<std::string>(
    "collision_topic",
    "/navlearn/collision");

  contacts_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
    contacts_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&CollisionMonitor::contactsCallback, this, std::placeholders::_1));

  collision_pub_ = create_publisher<std_msgs::msg::Empty>(
    collision_topic_,
    rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(),
    "CollisionMonitor: listening on '%s', publishing to '%s'",
    contacts_topic_.c_str(), collision_topic_.c_str());
}

void CollisionMonitor::contactsCallback(
  ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg)
{
  const bool in_contact = !msg->contacts.empty();

  // Rising-edge only: publish once per contact event start (False -> True)
  if (in_contact && !was_in_contact_) {
    collision_pub_->publish(std_msgs::msg::Empty{});
    RCLCPP_DEBUG(get_logger(), "Collision event detected");
  }

  was_in_contact_ = in_contact;
}

}  // namespace navlearn_benchmarks

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navlearn_benchmarks::CollisionMonitor>("collision_monitor");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
