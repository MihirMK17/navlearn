/**
 * Node: EpisodeManager
 * Role:
 *  - Loads navigation goals and sends them to the Robot sequentially
 *  - Tracks episode state, navigation time, start pose, goal pose (and collisions --> planned) for each goal
 *  - Publishes them as EpisodeEvent messages
 *  - Accuracy in goal_xy and goal_yaw is decided by controller_server (0.25m and 0.25 rad/s)
 *  - Accuracy in path length measurements is decided by trajectory metric node's parameters
 * 
 * Subscribes / Client to:
 *  - /navigate_to_pose :  nav2_msgs::action::NavigateToPose
 *  - /map  :  nav_msgs::msg::OccupancyGrid
 *  - /global_costmap/map   :  nav_msgs::msg::OccupancyGrid (planned)
 *  - /local_costmap/map    :  nav_msgs::msg::OccupancyGrid (planned)
 * 
 * Publishes:
 *  - /navlearn/episode_event  : navlearn_msgs::msg::EpisodeEvent
 * 
 * Parameters (planned):
 *  - dwell_sec_ (double)   :   Wait time at goal before marking success and proceeding to new goal
 *  - N_goals_ (int) :   Number of goals per episode
 *  - M_episodes_ (int) :   Number of episodes
 */

#ifndef EPISODE_MANAGER_HPP
#define EPISODE_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <navlearn_msgs/msg/episode_event.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

namespace navlearn_benchmarks{

class EpisodeManager : public rclcpp::Node
{
public:
    EpisodeManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<navlearn_msgs::msg::EpisodeEvent>::SharedPtr episode_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<geometry_msgs::msg::PoseStamped> goal_poses_;

    size_t idx_;
    bool active_;
    unique_identifier_msgs::msg::UUID episode_id_;
    
    rclcpp::Time stamp_received_;
    geometry_msgs::msg::PoseStamped start_pose_;

    double dwell_sec_;
    rclcpp::Time next_allowed_send_;
    std::string goal_source_;

    std::vector<double> goal_poses_x_;
    std::vector<double> goal_poses_y_;
    std::vector<double> goal_poses_yaw_;

    int goals_num_;
    int episodes_num_;

    rclcpp::QoS qos_profile_sub;

    std::string action_server_;
    std::string episode_pub_topic_;
    std::string fixed_frame_;
    std::string robot_frame_;

    bool have_map_;
    nav_msgs::msg::OccupancyGrid latest_map_;

    void loadGoals();

    bool sampleStartPoseAt(const rclcpp::Time & t, geometry_msgs::msg::PoseStamped & pose);

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

    void timerCallback();

    void sendGoal();

    void onGoalResponse(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & gh);

    void onFeedback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &,
                  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    
    void onResult(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result);
};
}

#endif