#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

using std::placeholders::_1;

class TrajectoryDrawer : public rclcpp::Node
{
public:
    TrajectoryDrawer() : Node("trajectory_drawer")
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/bumperbot_controller/odom", 10, 
                    std::bind(&TrajectoryDrawer::OdomCallback, this, _1));
        trajectory_pub_ = create_publisher<nav_msgs::msg::Path>("/bumperbot_controller/trajectory", 10);
    } 

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;

     void OdomCallback(const nav_msgs::msg::Odometry &odom_msg)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Received odometry message:\n"
                                         << "Position: ["
                                         << odom_msg.pose.pose.position.x << ", "
                                         << odom_msg.pose.pose.position.y << ", "
                                         << odom_msg.pose.pose.position.z << "]\n"
                                         << "Orientation: ["
                                         << odom_msg.pose.pose.orientation.x << ", "
                                         << odom_msg.pose.pose.orientation.y << ", "
                                         << odom_msg.pose.pose.orientation.z << ", "
                                         << odom_msg.pose.pose.orientation.w << "]\n"
                                         << "Drawing bumperbot trajectory...");
        // Create a PoseStamped message
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = odom_msg.header.stamp;
    pose_stamped.header.frame_id = odom_msg.header.frame_id;

    pose_stamped.pose = odom_msg.pose.pose; // Copy position and orientation from odometry

    // Add the new pose to the path message
    static nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = odom_msg.header.frame_id;
    path_msg.header.stamp = odom_msg.header.stamp;
    path_msg.poses.push_back(pose_stamped); // Add the new pose

    // Publish the path
    trajectory_pub_->publish(path_msg);

    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryDrawer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}