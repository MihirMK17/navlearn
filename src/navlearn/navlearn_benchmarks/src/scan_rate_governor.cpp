// Copyright 2026 Mihir Kulkarni
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

/// \file
/// Starves the LiDAR stream to a commanded rate, for the sensor-rate leg of the campaign.
///
/// Purpose
///     Leg 6 asks whether recovery from a kidnap degrades faster under a starved sensor
///     than tracking does. The mechanism under test is that recovery works by climbing a
///     likelihood gradient and every climb step is a measurement update, so removing
///     updates should hurt recovery more than it hurts control. Answering that needs the
///     starving to be exact, reproducible, and recorded.
///
/// Why a node rather than lowering the simulated sensor's update rate
///     Changing the LiDAR's rate in the robot description also changes what Gazebo
///     simulates and how the bridge is loaded, so a rate cell would differ from the
///     baseline in more than one variable. Decimating the published stream leaves the
///     simulation identical and varies exactly one thing: how many scans reach the stack.
///     It also puts the commanded rate in a parameter, where the run record can capture
///     it, instead of in an edited description file that no provenance check would see.
///
/// Why C++
///     This sits in the live sensor path. A relay in Python would add latency and jitter
///     to the measurements under study, and would inflate the ROBOT-class compute figure
///     the campaign reports.
///
/// Subscribers
///     input_topic (sensor_msgs/LaserScan) - the unstarved stream.
/// Publishers
///     output_topic (sensor_msgs/LaserScan) - the surviving scans, forwarded unmodified.
/// Parameters
///     input_topic (string, default "scan_unfiltered") - source stream.
///     output_topic (string, default "scan_governed") - starved stream.
///     native_rate_hz (double, default 10.0) - the source's true rate. RPLidar A1 is 10 Hz.
///     target_rate_hz (double, default 10.0) - commanded rate. Equal to native is a no-op
///         pass-through, so the node can stay in the graph for unstarved cells.
///     allow_approximate_rate (bool, default false) - permit a target the native rate
///         cannot divide. Off by default: recording a cell at a rate it never ran at puts
///         a wrong number in a results table.
///     report_period_sec (double, default 10.0) - interval for the delivered-rate log.

#include <exception>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "navlearn_benchmarks/scan_decimator.hpp"

namespace navlearn_benchmarks
{

class ScanRateGovernor : public rclcpp::Node
{
public:
  explicit ScanRateGovernor(const rclcpp::NodeOptions & options)
  : rclcpp::Node("scan_rate_governor", options)
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "scan_unfiltered");
    const auto output_topic = declare_parameter<std::string>("output_topic", "scan_governed");
    const double native_rate_hz = declare_parameter<double>("native_rate_hz", 10.0);
    const double target_rate_hz = declare_parameter<double>("target_rate_hz", 10.0);
    const bool allow_approximate = declare_parameter<bool>("allow_approximate_rate", false);
    report_period_sec_ = declare_parameter<double>("report_period_sec", 10.0);

    // Throws for a non-positive rate or a target above native. Allowed to escape to
    // main: a misconfigured rate cell must fail to start, not start unstarved.
    decimator_ = std::make_unique<navlearn::ScanDecimator>(native_rate_hz, target_rate_hz);

    if (decimator_->rate_is_approximate() && !allow_approximate) {
      RCLCPP_FATAL(get_logger(),
        "target_rate_hz %.4f Hz is not achievable from a %.4f Hz source by integer "
        "decimation; the nearest achievable rate is %.4f Hz (keep every %lu). Recording "
        "this cell at the requested rate would put a number in the results that the run "
        "never ran at. Set allow_approximate_rate:=true to run at %.4f Hz deliberately.",
        target_rate_hz, native_rate_hz, decimator_->achieved_rate_hz(),
        static_cast<unsigned long>(decimator_->keep_every()),
        decimator_->achieved_rate_hz());
      throw std::runtime_error("target_rate_hz is not achievable; see log");
    }

    // Matches the scan sanitizer this node sits beside. A depth-5 reliable queue on a
    // 10 Hz stream: deep enough to absorb a scheduling hiccup, shallow enough that a
    // stalled consumer surfaces as a dropped scan the rate monitor sees, rather than as
    // a growing backlog of stale measurements delivered late.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, qos);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, qos,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { onScan(msg); });

    last_report_ = now();

    RCLCPP_INFO(get_logger(),
      "Scan rate governor: %s -> %s | native %.3f Hz, commanded %.3f Hz, delivering "
      "%.3f Hz (keep every %lu scan%s)%s",
      input_topic.c_str(), output_topic.c_str(), native_rate_hz, target_rate_hz,
      decimator_->achieved_rate_hz(),
      static_cast<unsigned long>(decimator_->keep_every()),
      decimator_->keep_every() == 1u ? "" : "s",
      decimator_->keep_every() == 1u ? " - pass-through, nothing dropped" : "");

    if (decimator_->rate_is_approximate()) {
      RCLCPP_WARN(get_logger(),
        "Delivered rate %.4f Hz differs from the commanded %.4f Hz by %+.4f Hz. The run "
        "record must carry the delivered rate.",
        decimator_->achieved_rate_hz(), target_rate_hz, decimator_->rate_error_hz());
    }
  }

  /// The rate actually being delivered, for the run record.
  double achieved_rate_hz() const { return decimator_->achieved_rate_hz(); }

private:
  void onScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    // The keep decision is taken on the arrival index, never on a clock. A time-based
    // resampler would drop a different set of scans on every run, and a starved cell
    // would stop being reproducible from its seed.
    const bool keep = decimator_->shouldKeep(received_);
    ++received_;

    if (keep) {
      // Forwarded exactly as received. In particular the header stamp is untouched:
      // AMCL looks up the sensor-to-map transform at that stamp, so restamping on
      // forward would shift every measurement in time and change the experiment.
      pub_->publish(*msg);
      ++forwarded_;
    }

    maybeReport();
  }

  void maybeReport()
  {
    if (report_period_sec_ <= 0.0) { return; }
    const auto t = now();
    if ((t - last_report_).seconds() < report_period_sec_) { return; }

    RCLCPP_INFO(get_logger(),
      "Scan governor: %lu received, %lu forwarded (%.1f%%), target %.3f Hz",
      static_cast<unsigned long>(received_), static_cast<unsigned long>(forwarded_),
      received_ > 0 ? 100.0 * static_cast<double>(forwarded_) /
        static_cast<double>(received_) : 0.0,
      decimator_->achieved_rate_hz());
    last_report_ = t;
  }

  std::unique_ptr<navlearn::ScanDecimator> decimator_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;

  uint64_t received_ = 0;
  uint64_t forwarded_ = 0;
  double report_period_sec_ = 10.0;
  rclcpp::Time last_report_{0, 0, RCL_ROS_TIME};
};

}  // namespace navlearn_benchmarks

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<navlearn_benchmarks::ScanRateGovernor>(
      rclcpp::NodeOptions());
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    // Non-zero exit, so the harness's fail-loud gate stops the cell instead of letting it
    // run at a rate that was never requested.
    RCLCPP_FATAL(rclcpp::get_logger("scan_rate_governor"),
                 "Refusing to start: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
