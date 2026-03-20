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
    GzSetPoseServer(const std::string &name);

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