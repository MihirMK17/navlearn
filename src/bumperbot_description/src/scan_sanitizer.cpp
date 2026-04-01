#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanSanitizer : public rclcpp::Node
{
public:
  ScanSanitizer() : Node("scan_sanitizer")
  {
    input_topic_  = this->declare_parameter<std::string>("input_topic", "scan_unfiltered");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "scan");

    min_valid_range_ = this->declare_parameter<double>("min_valid_range", 0.12);
    max_valid_range_ = this->declare_parameter<double>("max_valid_range", 12.0);

    mode_ = this->declare_parameter<std::string>("mode", "to_inf");

    if (mode_ != "to_inf") {
      RCLCPP_WARN(this->get_logger(),
                  "Unsupported mode '%s'. Falling back to 'to_inf'.",
                  mode_.c_str());
      mode_ = "to_inf";
    }

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable());

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_,
      rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&ScanSanitizer::onScan, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(),
                "ScanSanitizer running. input='%s' output='%s' min=%.3f max=%.3f mode='%s'",
                input_topic_.c_str(), output_topic_.c_str(),
                min_valid_range_, max_valid_range_, mode_.c_str());
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto out = *msg;

    const float inf = std::numeric_limits<float>::infinity();

    const size_t n = out.ranges.size();
    bool intensities_match = (out.intensities.size() == n);

    for (size_t i = 0; i < n; ++i) {
      const float r = out.ranges[i];

      const bool bad =
        std::isnan(r) ||
        !std::isfinite(r) ||
        (r < static_cast<float>(min_valid_range_)) ||
        (r > static_cast<float>(max_valid_range_));

      if (bad) {
        out.ranges[i] = inf;
        if (intensities_match) {
          out.intensities[i] = 0.0f;
        }
      }
    }

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  double min_valid_range_;
  double max_valid_range_;
  std::string mode_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanSanitizer>());
  rclcpp::shutdown();
  return 0;
}