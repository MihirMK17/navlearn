#include <chrono>
#include <limits>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <navlearn_msgs/msg/goal_metric.hpp>
#include <navlearn_msgs/msg/episode_event.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

class GoalMetric : public rclcpp::Node
{
public:
  GoalMetric() : Node("goal_metric")
  {
    event_sub_ = create_subscription<navlearn_msgs::msg::EpisodeEvent>("/navlearn/episode_event", 10,
                  std::bind(&GoalMetric::onEvent, this, _1));
    goal_metric_pub_ = create_publisher<navlearn_msgs::msg::GoalMetric>("/navlearn/goal_metric", 10);

    timer_ = create_wall_timer(10ms, std::bind(&GoalMetric::timerCallback, this));
  }

private:
  rclcpp::Subscription<navlearn_msgs::msg::EpisodeEvent>::SharedPtr event_sub_;
  rclcpp::Publisher<navlearn_msgs::msg::GoalMetric>::SharedPtr goal_metric_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  navlearn_msgs::msg::GoalMetric goal_metric_msg_;

  void onEvent(const navlearn_msgs::msg::EpisodeEvent::SharedPtr & msg)
  {
    
  }
};