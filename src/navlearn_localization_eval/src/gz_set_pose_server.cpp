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

#include <navlearn_localization_eval/gz_set_pose_server.hpp>

using namespace std::placeholders;

GzSetPoseServer::GzSetPoseServer(const std::string & name)
: Node(name)
{
  world_name_ = this->declare_parameter<std::string>("world_name", "default");
  ign_service_ = this->declare_parameter<std::string>(
    "ign_service",
    "/world/" + world_name_ + "/set_pose");
  timeout_ms_ = this->declare_parameter<int>("timeout_ms", 2000);

  srv_ = create_service<navlearn_msgs::srv::SetEntityPose>(
    "/gz_set_pose_server/set_entity_pose",
    std::bind(&GzSetPoseServer::onRequest, this, _1, _2));

  RCLCPP_INFO(
    get_logger(),
    "ROS service: %s/set_entity_pose -> IGN service: %s (timeout=%d ms)",
    this->get_fully_qualified_name(), ign_service_.c_str(), timeout_ms_);
}

void GzSetPoseServer::onRequest(
  const std::shared_ptr<navlearn_msgs::srv::SetEntityPose::Request> req,
  std::shared_ptr<navlearn_msgs::srv::SetEntityPose::Response> res)
{
  ignition::msgs::Pose ign_req;
  ign_req.set_name(req->name);

  auto * p = ign_req.mutable_position();
  p->set_x(req->pose.position.x);
  p->set_y(req->pose.position.y);
  p->set_z(req->pose.position.z);

  auto * q = ign_req.mutable_orientation();
  q->set_x(req->pose.orientation.x);
  q->set_y(req->pose.orientation.y);
  q->set_z(req->pose.orientation.z);
  q->set_w(req->pose.orientation.w);

  ignition::msgs::Boolean ign_res;
  bool result = false;
  bool executed = ign_node_.Request(
    ign_service_,
    ign_req,
    static_cast<unsigned int>(timeout_ms_),
    ign_res,
    result
  );

  res->success = (executed && result && ign_res.data());

  if (!executed) {
    res->message = "IGN request failed / timed out";
  } else if (!result) {
    res->message = "IGN transport returned result=false";
  } else if (!ign_res.data()) {
    res->message = "IGN service returned Boolean=false";
  } else {
    res->message = "OK";
  }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GzSetPoseServer>("gz_set_pose_server");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
