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

#include <unordered_map>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <cmath>

#include "navlearn_benchmarks/metrics_compiler.hpp"
#include "navlearn_benchmarks/navlearn_utils.hpp"

namespace navlearn_benchmarks{

MetricsCompiler::MetricsCompiler(const std::string &name): Node(name)
, header_written_(false)
, success_count_(0)
{
  // Parameter: where to write the CSV
  csv_path_ = this->declare_parameter<std::string>("csv_path", "navlearn_metrics.csv");
  json_path_ = this->declare_parameter<std::string>("json_path", "navlearn_run_report.json");
  episode_event_topic_ = this->declare_parameter<std::string>("episode_event_topic", "/navlearn/episode_event");
  control_metric_topic_ = this->declare_parameter<std::string>("control_metric_topic", "/navlearn/control_metric");
  trajectory_metric_topic_ = this->declare_parameter<std::string>("trajectory_metric_topic", "/navlearn/trajectory_metric");
  kidnap_event_topic_ = this->declare_parameter<std::string>("kidnap_event_topic", "/navlearn/kidnap_event");
  // Distance beyond which a SUCCEEDED goal is recorded as a false success. Defaults to
  // the episode manager's end_error_pos_threshold_m so the two nodes agree; both are
  // reported in the data, so analysis is never left inferring which value applied.
  false_success_threshold_m_ = this->declare_parameter<double>("false_success_threshold_m", 0.10);

  csv_.open(csv_path_, std::ios::app);
  if (!csv_.is_open()) {
    RCLCPP_FATAL(get_logger(), "Failed to open CSV file: %s", csv_path_.c_str());
    throw std::runtime_error("cannot open metrics CSV");
  }

  // Subscriptions
  episode_sub_ = create_subscription<navlearn_msgs::msg::EpisodeEvent>(episode_event_topic_, 10,
                  std::bind(&MetricsCompiler::episodeCallback, this, std::placeholders::_1));

  control_sub_ = create_subscription<navlearn_msgs::msg::ControlMetric>(control_metric_topic_, 10,
                  std::bind(&MetricsCompiler::controlCallback, this, std::placeholders::_1));

  kidnap_sub_ = create_subscription<navlearn_msgs::msg::KidnapEvent>(kidnap_event_topic_, 10,
      [this](navlearn_msgs::msg::KidnapEvent::ConstSharedPtr m){ onKidnap(*m); });
  traj_sub_ = create_subscription<navlearn_msgs::msg::TrajectoryMetric>(trajectory_metric_topic_, 10,
                std::bind(&MetricsCompiler::trajCallback, this, std::placeholders::_1));

  run_acc_ = RunAccumulator{};

  RCLCPP_INFO(get_logger(), "MetricsCompiler writing to %s", csv_path_.c_str());
}

MetricsCompiler::EpisodeAggregate & MetricsCompiler::get_or_create(const unique_identifier_msgs::msg::UUID & id)
{
  const auto key = navlearn::uuid_to_string(id);
  return episodes_[key];  // default-construct if missing
}

namespace {

/// Yaw of a quaternion in degrees. A default-constructed (all-zero) quaternion is not a
/// valid rotation and means the pose was never populated, so it reports -1.0 rather than
/// a plausible-looking angle derived from nothing.
double yaw_degrees(const geometry_msgs::msg::Quaternion & q_msg)
{
  if (q_msg.x == 0.0 && q_msg.y == 0.0 && q_msg.z == 0.0 && q_msg.w == 0.0) {
    return -1.0;
  }
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw * 180.0 / M_PI;
}

}  // namespace

std::string MetricsCompiler::result_to_string(uint8_t result) const
{
  using E = navlearn_msgs::msg::EpisodeEvent;
  switch (result) {
    case E::RESULT_SUCCEEDED: return "SUCCEEDED";
    case E::RESULT_FAILED:    return "FAILED";
    case E::RESULT_CANCELED:  return "CANCELED";
    case E::RESULT_NA:        return "NA";
    default:                  return "UNKNOWN";
  }
}

// ---------- Callbacks ----------

void MetricsCompiler::episodeCallback(navlearn_msgs::msg::EpisodeEvent::SharedPtr msg)
{
  using E = navlearn_msgs::msg::EpisodeEvent;

  if (msg->state != E::END) {
    return;
  }

  auto & episode = get_or_create(msg->goal_id);
  episode.end_event = *msg;
  episode.have_end_event = true;

  const auto key= navlearn::uuid_to_string(msg->goal_id);
  maybe_flush_episode(key, episode);
}

void MetricsCompiler::controlCallback(navlearn_msgs::msg::ControlMetric::SharedPtr msg)
{
  auto & episode = get_or_create(msg->goal_id);
  episode.control = *msg;
  episode.have_control = true;

  const auto key = navlearn::uuid_to_string(msg->goal_id);
  maybe_flush_episode(key, episode);
}

void MetricsCompiler::trajCallback(navlearn_msgs::msg::TrajectoryMetric::SharedPtr msg)
{
  auto & episode = get_or_create(msg->goal_id);
  episode.traj = *msg;
  episode.have_trajectory = true;

  const auto key = navlearn::uuid_to_string(msg->goal_id);
  maybe_flush_episode(key, episode);
}

void MetricsCompiler::onKidnap(const navlearn_msgs::msg::KidnapEvent & ev)
{
  // Not part of the flush gate: clean and TTC cells produce no kidnap events at all, so
  // requiring one would stall every non-TTR episode forever. Recorded if it arrives.
  auto & episode = get_or_create(ev.goal_id);
  episode.kidnap = ev;
  episode.have_kidnap = true;

  if (!ev.success) {
    RCLCPP_WARN(get_logger(),
      "Kidnap attempt FAILED for this goal — no perturbation was applied. The row will "
      "carry Kidnap Applied=0 and must not be pooled with kidnapped goals.");
  }
}

// ---------- Flush to CSV ----------

void MetricsCompiler::maybe_flush_episode(const std::string & key, EpisodeAggregate & episode)
{
  auto t0 = std::chrono::steady_clock::now();

  // Wait until we have END event + both metrics
  if (!(episode.have_end_event && episode.have_control && episode.have_trajectory)) {
    return;
  }

  const auto & ev = episode.end_event;
  const auto & cm = episode.control;
  const auto & tm = episode.traj;

  update_run_accumulator(ev, cm, tm);

  // Header (once)
  if (!header_written_) {
    csv_ << "Goal_ID,Reference Frame,"
         << "Start Pose_X (m),Start Pose_Y (m),Start Pose_Yaw (deg),Goal Pose_X (m),Goal Pose_Y (m),Goal Pose_Yaw (deg),Goal Result Code,Goal Result,Success Count,Nav Time (sec),Nav Time Start (sec),Nav Time End (sec),"
         << "Tracking RMS_V (m/s),Tracking RMS_W (rad/s),Saturation Frac_V,Saturation Frac_W,Slip Mean,Slip Std Deviation,Slip 95_Percentile,Control Energy,Control Samples,"
         << "Path Length (m),Absolute Path Error RMS (m),Relative Pose Error (Drift),Min Clearance (m),Collision Count,Trajectory Samples,"
         << "SPL,"
         // Terminal pose block. "True Distance To Goal" is the campaign's headline
         // measurement: Nav2 judges success against the localization estimate, so a
         // SUCCEEDED row with a large value here is a false success. Missing values are
         // -1.0, never 0.0 — see TerminalPoseReport.msg.
         << "GT Available,Estimate Available,"
         << "True Pose_X (m),True Pose_Y (m),True Pose_Yaw (deg),"
         << "Estimated Pose_X (m),Estimated Pose_Y (m),Estimated Pose_Yaw (deg),"
         << "True Distance To Goal (m),Estimated Distance To Goal (m),Localization Error (m),"
         << "True Yaw Error (deg),"
         << "Covariance_XX (m2),Covariance_YY (m2),Covariance_Yaw (rad2),"
         << "Filter Converged,Convergence Threshold (m2),Estimate Age (sec),"
         << "False Success,"
         // Was the perturbation actually applied? A TTR goal whose kidnap never fired is
         // not a TTR trial. Nothing recorded this before, so a cell could silently contain
         // unperturbed episodes pooled with perturbed ones.
         << "Kidnap Attempted,Kidnap Applied,"
         << "Kidnap Target_X (m),Kidnap Target_Y (m),Kidnap Target_Yaw (deg),"
         // The teleport's "before". Displacement is derived from it here for convenience
         // and is -1 whenever it is not a measurement: no kidnap, no reference, or an
         // attempt that never moved the robot. Never 0, which would read as a teleport
         // that happened to land where it started.
         << "Kidnap Reference Available,"
         << "Kidnap Reference_X (m),Kidnap Reference_Y (m),Kidnap Reference_Yaw (deg),"
         << "Kidnap Displacement (m),Kidnap Yaw Change (deg),"
         // What the design asked for, as opposed to what the geometry allowed. The curve
         // is fitted against this; the predictor comparison uses the realised
         // displacement above. -1 when severity was not drawn from a continuous range.
         << "Kidnap Commanded Magnitude (m)"
         << "\n";
    header_written_ = true;
  }

  // Convert Start Pose message types to doubles
  const auto start_pose_x = ev.start_pose.pose.position.x;
  const auto start_pose_y = ev.start_pose.pose.position.y;
  tf2::Quaternion q_start(ev.start_pose.pose.orientation.x,
                    ev.start_pose.pose.orientation.y,
                    ev.start_pose.pose.orientation.z,
                    ev.start_pose.pose.orientation.w);
  
  tf2::Matrix3x3 m_start(q_start);
  double roll_start, pitch_start, yaw_start;
  m_start.getRPY(roll_start, pitch_start, yaw_start);
  
  // Convert Start Pose message types to doubles
  const auto goal_pose_x = ev.goal_pose.pose.position.x;
  const auto goal_pose_y = ev.goal_pose.pose.position.y;
  tf2::Quaternion q_goal(ev.goal_pose.pose.orientation.x,
                    ev.goal_pose.pose.orientation.y,
                    ev.goal_pose.pose.orientation.z,
                    ev.goal_pose.pose.orientation.w);
  
  tf2::Matrix3x3 m_goal(q_goal);
  double roll_goal, pitch_goal, yaw_goal;
  m_goal.getRPY(roll_goal, pitch_goal, yaw_goal);


  // Convert nav_time (Duration) to seconds
  const auto & nt = ev.nav_time;
  const auto & ts = ev.stamp_received;
  const auto & tterm = ev.stamp_terminated;
  const double nav_time_sec = static_cast<double>(nt.sec) + 1e-9 * static_cast<double>(nt.nanosec);
  const double t_start_sec = static_cast<double>(ts.sec) + 1e-9 * static_cast<double>(ts.nanosec);
  const double t_terminated_sec = static_cast<double>(tterm.sec) + 1e-9 * static_cast<double>(tterm.nanosec);

  const std::string result_str = result_to_string(ev.result);

  if(ev.result == 1) success_count_ ++;

  // To implement  - max goal_counter in EpisodeEvent message, set from the config file / parameter during execution

  // One row per goal
  csv_ << key << ","
       << "map" << ","
       << start_pose_x << ","
       << start_pose_y << ","
       << yaw_start * 180 / M_PI << ","
       << goal_pose_x << ","
       << goal_pose_y << ","
       << yaw_goal * 180 / M_PI << ","
       << static_cast<int>(ev.result) << ","
       << result_str << ","
       << success_count_ << ","
       << nav_time_sec << ","
       << t_start_sec << ","
       << t_terminated_sec << ","

       << cm.tracking_rms_v << ","
       << cm.tracking_rms_w << ","
       << cm.saturation_frac_v << ","
       << cm.saturation_frac_w << ","
       << cm.slip_mean << ","
       << cm.slip_std << ","
       << cm.slip_p95 << ","
       << cm.control_energy << ","
       << cm.samples << ","

       << tm.path_length_m << ","
       << tm.ate_rmse_m << ","
       << tm.rpe_trans_rmse_m << ","
       << tm.min_clearance_m << ","
       << tm.collision_count << ","
       << tm.samples << ","

       // SPL: success * optimal_path / max(actual_path, optimal_path)
       << [&]() -> double {
            if (ev.result == navlearn_msgs::msg::EpisodeEvent::RESULT_SUCCEEDED
                && ev.optimal_path_m > 0.0) {
              return ev.optimal_path_m / std::max(tm.path_length_m, ev.optimal_path_m);
            }
            return 0.0;
          }()
       << ","

       // --- terminal pose block ---
       << (ev.terminal.gt_available ? 1 : 0) << ","
       << (ev.terminal.estimate_available ? 1 : 0) << ","
       << ev.terminal.true_pose.pose.position.x << ","
       << ev.terminal.true_pose.pose.position.y << ","
       << yaw_degrees(ev.terminal.true_pose.pose.orientation) << ","
       << ev.terminal.estimated_pose.pose.position.x << ","
       << ev.terminal.estimated_pose.pose.position.y << ","
       << yaw_degrees(ev.terminal.estimated_pose.pose.orientation) << ","
       << ev.terminal.true_distance_to_goal_m << ","
       << ev.terminal.estimated_distance_to_goal_m << ","
       << ev.terminal.localization_error_m << ","
       << (ev.terminal.true_yaw_error_rad < 0.0
             ? -1.0 : ev.terminal.true_yaw_error_rad * 180.0 / M_PI) << ","
       << ev.terminal.covariance_xx << ","
       << ev.terminal.covariance_yy << ","
       << ev.terminal.covariance_yaw << ","
       << (ev.terminal.filter_converged ? 1 : 0) << ","
       << ev.terminal.convergence_threshold_m2 << ","
       << ev.terminal.estimate_age_sec << ","

       // Derived for convenience; recomputable from the columns above. A goal Nav2
       // reported as reached, where ground truth says the robot is further away than the
       // success radius was ever meant to permit. -1 when ground truth was unavailable,
       // so an unmeasured goal is never silently counted as a clean success.
       << [&]() -> int {
            if (!ev.terminal.gt_available) return -1;
            if (ev.result != navlearn_msgs::msg::EpisodeEvent::RESULT_SUCCEEDED) return 0;
            return ev.terminal.true_distance_to_goal_m > false_success_threshold_m_ ? 1 : 0;
          }()
       << ","

       // --- perturbation actually applied? ---
       << (episode.have_kidnap ? 1 : 0) << ","
       << (episode.have_kidnap && episode.kidnap.success ? 1 : 0) << ","
       << (episode.have_kidnap ? episode.kidnap.kidnap_pose.position.x : -1.0) << ","
       << (episode.have_kidnap ? episode.kidnap.kidnap_pose.position.y : -1.0) << ","
       << (episode.have_kidnap
             ? yaw_degrees(episode.kidnap.kidnap_pose.orientation) : -1.0) << ",";

  {
    const bool have_ref = episode.have_kidnap && episode.kidnap.reference_available;

    // Displacement requires both a "before" and a teleport that actually happened. A
    // failed attempt left the robot at the reference, so writing the commanded target's
    // distance would record a perturbation the robot never experienced.
    const bool measurable = have_ref && episode.kidnap.success;

    const auto & ref = episode.kidnap.reference_pose;
    const auto & tgt = episode.kidnap.kidnap_pose;

    csv_ << (have_ref ? 1 : 0) << ","
         << (have_ref ? ref.position.x : -1.0) << ","
         << (have_ref ? ref.position.y : -1.0) << ","
         << (have_ref ? yaw_degrees(ref.orientation) : -1.0) << ","
         << (measurable
               ? std::hypot(tgt.position.x - ref.position.x,
                            tgt.position.y - ref.position.y)
               : -1.0) << ","
         // Shorter arc: a rotation across the pi boundary is a small turn, not a large
         // one, and severity should not jump by 340 degrees at the wrap point.
         << (measurable
               ? std::fabs(std::remainder(yaw_degrees(tgt.orientation)
                                          - yaw_degrees(ref.orientation), 360.0))
               : -1.0) << ","
         << (episode.have_kidnap ? episode.kidnap.commanded_magnitude_m : -1.0);
  }

  csv_ << "\n";

  csv_.flush();

  RCLCPP_INFO(get_logger(), "Wrote episode %s: result=%s, nav_time=%.3fs, path=%.3fm, RMSv=%.3f, RMSw=%.3f", key.c_str(), result_str.c_str(), nav_time_sec,
              tm.path_length_m, cm.tracking_rms_v, cm.tracking_rms_w);

  // Drop from map so memory doesn't grow forever
  episodes_.erase(key);

  auto t1 = std::chrono::steady_clock::now();
  double dt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  RCLCPP_INFO(get_logger(), "maybe_flush_episode took %.3f ms (episode %s)", dt_ms, key.c_str());
}

void MetricsCompiler::update_run_accumulator(const navlearn_msgs::msg::EpisodeEvent &ev, const navlearn_msgs::msg::ControlMetric &cm, 
                              const navlearn_msgs::msg::TrajectoryMetric &tm)
{
  using E = navlearn_msgs::msg::EpisodeEvent;

  run_acc_.goals_total++;

  switch (ev.result) {
    case E::RESULT_SUCCEEDED: run_acc_.goals_succeeded++; break;
    case E::RESULT_FAILED:    run_acc_.goals_failed++;    break;
    case E::RESULT_CANCELED:  run_acc_.goals_canceled++;  break;
    default: break;
  }

  // nav time in seconds
  const auto & nt = ev.nav_time;
  const double nav_t = static_cast<double>(nt.sec) + 1e-9 * static_cast<double>(nt.nanosec);
  run_acc_.total_nav_time += nav_t;

  // path length
  const double L = tm.path_length_m;
  run_acc_.total_path_traveled += L;

  // control energy
  const double Ectrl = cm.control_energy;
  run_acc_.total_control_energy += Ectrl;

  // SPL: success * optimal_path / max(actual_path, optimal_path)
  if (ev.result == E::RESULT_SUCCEEDED && ev.optimal_path_m > 0.0) {
    run_acc_.total_spl += ev.optimal_path_m / std::max(L, ev.optimal_path_m);
  }

  run_acc_.total_collisions += tm.collision_count;
}

void MetricsCompiler::write_run_json()
{
  const unsigned int N = run_acc_.goals_total;
  if (N == 0) {
    RCLCPP_WARN(get_logger(), "No goals seen; skipping run JSON.");
    return;
  }

  std::ofstream js(json_path_, std::ios::out | std::ios::trunc);
  if (!js.is_open()) {
    throw std::runtime_error("Cannot open json_path for write: " + json_path_);
  }

  js << "{\n";
  js << "  \"goals_total\": "      << N << ",\n";
  js << "  \"goals_succeeded\": "  << run_acc_.goals_succeeded << ",\n";
  js << "  \"goals_failed\": "     << run_acc_.goals_failed << ",\n";
  js << "  \"goals_canceled\": "   << run_acc_.goals_canceled << ",\n";

  js << "  \"nav_time_mean\": "       << run_acc_.total_nav_time / static_cast<double>(N) << ",\n";
  js << "  \"total_nav_time\": "      << run_acc_.total_nav_time << ",\n";

  js << "  \"path_length_mean\": "    << run_acc_.total_path_traveled / static_cast<double>(N) << ",\n";
  js << "  \"total_path_traveled\": " << run_acc_.total_path_traveled << ",\n";

  js << "  \"control_energy_mean\": " << run_acc_.total_control_energy / static_cast<double>(N) << ",\n";
  js << "  \"total_control_energy\": "  << run_acc_.total_control_energy << ",\n";

  js << "  \"spl_mean\": "            << run_acc_.total_spl / static_cast<double>(N) << ",\n";
  js << "  \"total_spl\": "           << run_acc_.total_spl << ",\n";

  js << "  \"total_collisions\": "    << run_acc_.total_collisions << ",\n";
  js << "  \"collisions_per_goal\": " << run_acc_.total_collisions / static_cast<double>(N) << ",\n";

  js << "  \"csv_path\": "            << "\"" << csv_path_ << "\"\n";
  js << "}\n";

  js.close();

  RCLCPP_INFO(
    get_logger(),
    "Run summary JSON written to %s (N=%u, success=%u)",
    json_path_.c_str(), N, run_acc_.goals_succeeded);
}

MetricsCompiler::~MetricsCompiler()
{
  try{
    write_run_json();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to write run JSON: %s", e.what());
  }

  if(csv_.is_open()){
    csv_.close();
  }
}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navlearn_benchmarks::MetricsCompiler>("metrics_compiler");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
