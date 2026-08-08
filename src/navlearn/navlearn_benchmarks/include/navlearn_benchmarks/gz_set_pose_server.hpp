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

#ifndef GZ_SET_POSE_SERVER_HPP
#define GZ_SET_POSE_SERVER_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <navlearn_msgs/srv/set_entity_pose.hpp>

#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/boolean.pb.h>

class GzSetPoseServer : public rclcpp::Node
{
public:
  GzSetPoseServer(const std::string & name);

private:
  std::string world_name_;
  std::string ign_service_;
  int timeout_ms_;
  ignition::transport::Node ign_node_;

  rclcpp::Service<navlearn_msgs::srv::SetEntityPose>::SharedPtr srv_;

  void onRequest(
    const std::shared_ptr<navlearn_msgs::srv::SetEntityPose::Request> req,
    std::shared_ptr<navlearn_msgs::srv::SetEntityPose::Response> res);
};

#endif
