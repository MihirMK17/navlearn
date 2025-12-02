#include <chrono>
#include <algorithm>
#include <limits>
#include <cmath>

#include "navlearn_benchmarks/control_metric.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace navlearn_benchmarks{

ControlMetric::ControlMetric(const std::string &name) : Node(name)
  , active_(false)
  , count_(0)
  , sat_v_count_(0)
  , sat_w_count_(0)
  , sum_ev2_(0)
  , sum_ew2_(0)
  , sum_s_(0)
  , sum_s2_(0)
  , control_energy_(0.0)
{
  controller_topic_ = this->declare_parameter<std::string>("controller_topic", "/bumperbot_controller/cmd_vel");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/bumperbot_controller/odom");
  episode_event_topic_ = this->declare_parameter<std::string>("episode_event_topic", "/navlearn/episode_event");
  joint_states_topic_ = this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
  control_metric_topic_ = this->declare_parameter<std::string>("controller_metric_topic", "/navlearn/control_metric");

  wheel_left_joint_ = this->declare_parameter<std::string>("wheel_left_joint", "wheel_left_joint");
  wheel_right_joint_ = this->declare_parameter<std::string>("wheel_right_joint", "wheel_right_joint");

  wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.033);
  wheel_separation_ = this->declare_parameter<double>("wheel_separation", 0.16);
  v_max_ = this->declare_parameter<double>("v_max", 0.25);
  w_max_ = this->declare_parameter<double>("w_max", 1.0);
  sat_tol_ = this->declare_parameter<double>("saturation_tolerance", 0.98);
  eps_v_ = this->declare_parameter<double>("eps_v", 0.05);
  eps_w_ = this->declare_parameter<double>("eps_w", 0.05);
  buffer_span_s_ = this->declare_parameter<double>("buffer_span", 2.0);
  lambda_ = this->declare_parameter<double>("lambda", 0.5);

  if(wheel_radius_ < 0)
  {
    RCLCPP_FATAL(get_logger(), "Bad param: wheel_radius (%f) has to be > 0", wheel_radius_);
    rclcpp::shutdown();
  }
  if(wheel_separation_ < 0)
  {
    RCLCPP_FATAL(get_logger(), "Bad param: wheel_separation (%f) has to be > 0", wheel_separation_);
    rclcpp::shutdown();
  }

  RCLCPP_INFO(get_logger(), "ControlMetric config: wheel_radius:%.2f, wheel_separation:%.2f, v_max:%.2f, w_max:%.2f, buffer_span:%.2f, lambda:%.2f",
              wheel_radius_, wheel_separation_, v_max_, w_max_, buffer_span_s_, lambda_);

  // QoS
  auto q_sensor = rclcpp::SensorDataQoS{};
  auto q_cmd    = rclcpp::QoS{10};

  // /cmd_vel (Twist, no header) — stamp with now()
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(controller_topic_, q_cmd, 
                  std::bind(&ControlMetric::velCallback, this, _1));

  // High-rate sensors
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(joint_states_topic_, q_sensor, 
                      std::bind(&ControlMetric::jointStateCallback, this, _1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, q_sensor,
              std::bind(&ControlMetric::odomCallback, this, _1));

  // Episodes
  episode_sub_ = create_subscription<navlearn_msgs::msg::EpisodeEvent>(episode_event_topic_, rclcpp::QoS{10},
                  std::bind(&ControlMetric::episodeCallback, this, _1));

  control_metric_pub_ = create_publisher<navlearn_msgs::msg::ControlMetric>(control_metric_topic_, 10);

  timer_ = create_wall_timer(10ms, std::bind(&ControlMetric::timerCallback, this));
}

// Helpers
void ControlMetric::resetAccumulators()
{
  count_ = sat_v_count_ = sat_w_count_ = 0;
  sum_ev2_ = sum_ew2_ = sum_s_ = sum_s2_ = 0.0;
  slip_abs_buf_.clear();
  control_energy_ = 0.0;
}

void ControlMetric::trim(std::deque<Sample>& q, const rclcpp::Time &now)
{
  while (!q.empty() && (now - q.front().t).seconds() > buffer_span_s_) {
    q.pop_front();
  }
}

// NOTE: Twist (no header) => use this->now()
void ControlMetric::velCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr vel)
{
  const rclcpp::Time t = this->now();
  buf_v_cmd_.push_back({t, vel->twist.linear.x});
  buf_w_cmd_.push_back({t, vel->twist.angular.z});
  trim(buf_v_cmd_, t);
  trim(buf_w_cmd_, t);
}

void ControlMetric::jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr joint)
{
  const rclcpp::Time t = joint->header.stamp;

  double wL = std::numeric_limits<double>::quiet_NaN();
  double wR = std::numeric_limits<double>::quiet_NaN();

  // Prefer explicit names if present; else first two velocities
  for (std::size_t i = 0; i < joint->name.size() && i < joint->velocity.size(); ++i) {
    if (joint->name[i].find(wheel_left_joint_) != std::string::npos)  wL = joint->velocity[i];
    if (joint->name[i].find(wheel_right_joint_) != std::string::npos) wR = joint->velocity[i];
  }
  if (!std::isfinite(wL) || !std::isfinite(wR)) {
    // Fallback: if exactly 2 velocities, assume [L, R]
    if (joint->velocity.size() >= 2) {
      wL = std::isfinite(wL) ? wL : joint->velocity[0];
      wR = std::isfinite(wR) ? wR : joint->velocity[1];
    }
  }
  if (!std::isfinite(wL) || !std::isfinite(wR)) return;

  const double v_wheel = 0.5 * wheel_radius_ * (wR + wL);
  const double w_wheel = wheel_radius_ * (wR - wL) / wheel_separation_;

  buf_v_wheel_.push_back({t, v_wheel});
  buf_w_wheel_.push_back({t, w_wheel});
  trim(buf_v_wheel_, t);
  trim(buf_w_wheel_, t);
}

void ControlMetric::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  const rclcpp::Time t = odom->header.stamp;
  buf_v_base_.push_back({t, odom->twist.twist.linear.x});
  buf_w_base_.push_back({t, odom->twist.twist.angular.z});
  trim(buf_v_base_, t);
  trim(buf_w_base_, t);
}

// Zero-Order Hold from latest <= tk (bounded staleness)
bool ControlMetric::zero_order_hold(const std::deque<Sample> &q,
                      const rclcpp::Time &tk,
                      double max_stale_s,
                      double &out)
{
  if (q.empty()) return false;
  for (std::size_t k = q.size(); k-- > 0; ) {
    if (q[k].t <= tk) {
      const double stale = (tk - q[k].t).seconds();
      if (stale > max_stale_s) return false;
      out = q[k].x;
      return true;
    }
  }
  return false;
}

// Interpolate if we have [left,right] around tk and gap is small; else ZOH
bool ControlMetric::sample_interp_or_hold(const std::deque<Sample> &q,
                            rclcpp::Time tk,
                            double &out,
                            double max_gap_s,
                            double max_stale_s)
{
  const size_t n = q.size();
  if (n == 0) return false;

  // Safety lag: pull tk back by half of the latest interval (prevents "future" edge)
  if (n >= 2) {
    const double dt_latest = (q.back().t - q[n-2].t).seconds();
    if (dt_latest > 0.0 && dt_latest <= max_gap_s) {
      tk = tk - rclcpp::Duration::from_seconds(0.5 * dt_latest);
    }
  }

  // Find last sample with t <= tk
  size_t i = n;
  while (i-- > 0) {
    if (q[i].t <= tk) {
      const double stale = (tk - q[i].t).seconds();
      if (stale > max_stale_s) return false;

      // Try linear interpolation using (i,i+1)
      if (i + 1 < n) {
        const double dt = (q[i+1].t - q[i].t).seconds();
        if (dt > 0.0 && dt <= max_gap_s) {
          const double alpha = (tk - q[i].t).seconds() / dt;
          out = q[i].x + alpha * (q[i+1].x - q[i].x);
          return true;
        }
      }
      // Fallback: ZOH
      out = q[i].x;
      return true;
    }
  }
  // All samples are > tk
  return false;
}

void ControlMetric::timerCallback()
{
  if (!active_) return;

  // If using sim time and it hasn't started yet, bail quietly
  if (this->get_clock()->get_clock_type() == RCL_ROS_TIME &&
      this->now().nanoseconds() == 0) {
    return;
  }

  const rclcpp::Time tk = this->now();

  double v_cmd, w_cmd;
  if (!zero_order_hold(buf_v_cmd_, tk, 0.30, v_cmd)) return;
  if (!zero_order_hold(buf_w_cmd_, tk, 0.30, w_cmd)) return;

  double v_base, w_base, v_wheel;
  if (!sample_interp_or_hold(buf_v_base_,  tk, v_base))  return;
  if (!sample_interp_or_hold(buf_w_base_,  tk, w_base))  return;
  if (!sample_interp_or_hold(buf_v_wheel_, tk, v_wheel)) return;

  const double ev = v_cmd - v_base;
  const double ew = w_cmd - w_base;

  sum_ev2_ += ev * ev;
  sum_ew2_ += ew * ew;

  sat_v_count_ += (std::abs(v_cmd) >= sat_tol_ * v_max_) ? 1 : 0;
  sat_w_count_ += (std::abs(w_cmd) >= sat_tol_ * w_max_) ? 1 : 0;

  const bool standstill = (std::abs(v_cmd) < 0.02) && (std::abs(v_base) < eps_v_);
  const double denom = std::max(std::abs(v_base), eps_v_);
  const double slip = standstill ? 0.0 : (v_wheel - v_base) / denom;

  sum_s_  += slip;
  sum_s2_ += slip * slip;
  if (slip_abs_buf_.size() >= 1000) slip_abs_buf_.pop_front();
  slip_abs_buf_.push_back(std::abs(slip));

  control_energy_ += (v_base*v_base) + (lambda_)*(w_base*w_base);

  count_ += 1;
}

void ControlMetric::episodeCallback(const navlearn_msgs::msg::EpisodeEvent::ConstSharedPtr episode)
{
  using E = navlearn_msgs::msg::EpisodeEvent;
  if (episode->state == E::START) {
    active_ = true;
    episode_id_ = episode->episode_id;
    t_start_ = episode->header.stamp;
    resetAccumulators();
  } else if (episode->state == E::END) {
    if (!active_) return;
    publishReport(episode->header.stamp);
    active_ = false;
  }
}

void ControlMetric::publishReport(const rclcpp::Time& t_end)
{
  navlearn_msgs::msg::ControlMetric msg;
  msg.header.stamp = t_end;
  msg.header.frame_id = "base_link";
  msg.episode_id = episode_id_;
  msg.samples = count_;

  // Duration
  const rclcpp::Duration dur = t_end - t_start_;
  builtin_interfaces::msg::Duration dmsg;
  const int64_t nsec = dur.nanoseconds();
  dmsg.sec     = static_cast<int32_t>(nsec / 1000000000LL);
  dmsg.nanosec = static_cast<uint32_t>(nsec % 1000000000LL);
  msg.duration = dmsg;

  if (count_ > 0) {
    msg.tracking_rms_v = std::sqrt(sum_ev2_ / count_);
    msg.tracking_rms_w = std::sqrt(sum_ew2_ / count_);
    msg.saturation_frac_v = static_cast<double>(sat_v_count_) / count_;
    msg.saturation_frac_w = static_cast<double>(sat_w_count_) / count_;

    const double mean = sum_s_ / count_;
    msg.slip_mean = mean;
    msg.slip_std = (count_ > 1)
      ? std::sqrt(std::max(0.0, (sum_s2_ - count_ * mean * mean) / (count_ - 1)))
      : 0.0;

    std::vector<double> tmp(slip_abs_buf_.begin(), slip_abs_buf_.end());
    if (!tmp.empty()) {
      std::sort(tmp.begin(), tmp.end());
      const std::size_t k = static_cast<std::size_t>(std::floor(0.95 * (tmp.size() - 1)));
      msg.slip_p95 = tmp[k];
    } else {
      msg.slip_p95 = 0.0;
    }
  } else {
    msg.tracking_rms_v = msg.tracking_rms_w = 0.0;
    msg.saturation_frac_v = msg.saturation_frac_w = 0.0;
    msg.slip_mean = msg.slip_std = msg.slip_p95 = 0.0;
  }

  msg.control_energy = static_cast<double>(control_energy_);
  msg.v_max = v_max_;
  msg.w_max = w_max_;
  msg.epsilon_v = eps_v_;
  msg.epsilon_w = eps_w_;

  control_metric_pub_->publish(msg);
  RCLCPP_INFO(get_logger(),
    "ControlMetric: N=%u RMSv=%.3f RMSw=%.3f satV=%.2f satW=%.2f slip(mu=%.3f, p95=%.3f)",
    msg.samples, msg.tracking_rms_v, msg.tracking_rms_w,
    msg.saturation_frac_v, msg.saturation_frac_w,
    msg.slip_mean, msg.slip_p95);
}
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navlearn_benchmarks::ControlMetric>("control_metric");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
