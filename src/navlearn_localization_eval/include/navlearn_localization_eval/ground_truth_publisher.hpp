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
    explicit GroundTruthTF(const std::string &name);

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
