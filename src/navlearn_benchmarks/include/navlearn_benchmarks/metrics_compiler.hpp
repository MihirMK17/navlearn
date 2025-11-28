/**
 * Node: MetricCompiler
 * Role:
 *  - Collects all the navigation metrics from individual metric nodes
 *  - Writes them to a csv file for each goal with an episode id, once it recieves all the required metrics
 * 
 * Subscribes:
 *  - /navlearn/episode_event : navlearn_msgs::msg::EpisodeEvent
 *  - /navlearn/control_metric  : navlearn_msgs::msg::ControlMetric
 *  - /navlearn/trajectory_metric : navlearn_msgs::msg::TrajectoryMetric
 * 
 * Publishes:
 * 
 * Parameters (planned):
 *  - csv_path_ (std::string) : Directory path where the benchmark report is stored
 */

#ifndef METRICS_COMPILER_HPP
#define METRICS_COMPILER_HPP

#include <rclcpp/rclcpp.hpp>
#include <navlearn_msgs/msg/control_metric.hpp>
#include <navlearn_msgs/msg/trajectory_metric.hpp>
#include <navlearn_msgs/msg/episode_event.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.hpp>

namespace navlearn_benchmarks{

class MetricsCompiler : public rclcpp::Node
{
public:
    MetricsCompiler(const std::string &name);

    ~MetricsCompiler();

private:
  struct EpisodeAggregate
  {
    bool have_end_event  = false;
    bool have_control    = false;
    bool have_trajectory = false;

    navlearn_msgs::msg::EpisodeEvent   end_event;
    navlearn_msgs::msg::ControlMetric  control;
    navlearn_msgs::msg::TrajectoryMetric traj;
  };

  struct RunAccumulator
  {
    unsigned int episodes_total = 0;
    unsigned int episodes_succeeded = 0;
    unsigned int episodes_failed = 0;
    unsigned int episodes_canceled = 0;

    double total_nav_time = 0.0;
    double total_nav_time_sq = 0.0;

    double total_path_traveled = 0.0;
    double total_path_traveled_sq = 0.0;

    double total_control_energy = 0.0;
    double total_control_energy_sq = 0.0;
  };

  RunAccumulator run_acc_;

  std::string episode_event_topic_;
  std::string control_metric_topic_;
  std::string trajectory_metric_topic_;

  std::string csv_path_;
  std::string json_path_;
  std::ofstream csv_;
  bool header_written_;

  rclcpp::Subscription<navlearn_msgs::msg::EpisodeEvent>::SharedPtr episode_sub_;
  rclcpp::Subscription<navlearn_msgs::msg::ControlMetric>::SharedPtr control_sub_;
  rclcpp::Subscription<navlearn_msgs::msg::TrajectoryMetric>::SharedPtr traj_sub_;

  std::unordered_map<std::string, EpisodeAggregate> episodes_;

  int success_count_;

  static std::string uuid_to_string(const unique_identifier_msgs::msg::UUID & id);

  EpisodeAggregate & get_or_create(const unique_identifier_msgs::msg::UUID & id);

  std::string result_to_string(uint8_t result) const;

  void episodeCallback(navlearn_msgs::msg::EpisodeEvent::SharedPtr msg);

  void controlCallback(navlearn_msgs::msg::ControlMetric::SharedPtr msg);

  void trajCallback(navlearn_msgs::msg::TrajectoryMetric::SharedPtr msg);

  void maybe_flush_episode(const std::string & key, EpisodeAggregate & episode);

  void update_run_accumulator(const navlearn_msgs::msg::EpisodeEvent &ev, const navlearn_msgs::msg::ControlMetric &cm, 
                              const navlearn_msgs::msg::TrajectoryMetric &tm);
  
  void write_run_json();
};
}

#endif