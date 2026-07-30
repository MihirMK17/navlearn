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

// episode_manager.hpp

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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <navlearn_msgs/msg/episode_event.hpp>
#include <navlearn_msgs/msg/kidnap_event.hpp>
#include <navlearn_msgs/msg/terminal_pose_report.hpp>
#include <navlearn_benchmarks/navlearn_seed.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav2_msgs/srv/set_initial_pose.hpp>
#include <navlearn_msgs/srv/set_entity_pose.hpp>
#include <std_srvs/srv/empty.hpp>

namespace navlearn_benchmarks{

class EpisodeManager : public rclcpp::Node
{
public:
    EpisodeManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // Core ROS interfaces
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    // Subscribed for its covariance, which TF does not carry. The estimated pose itself
    // could be read from TF, but a filter's confidence is as much a part of the terminal
    // record as its position: a converged-but-wrong filter and an unconverged one are
    // different failures and must be distinguishable in the data.
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Publisher<navlearn_msgs::msg::EpisodeEvent>::SharedPtr episode_pub_;
    rclcpp::Publisher<navlearn_msgs::msg::KidnapEvent>::SharedPtr kidnap_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedPtr set_initial_pose_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr bad_init_pose_pub_;
    rclcpp::Client<navlearn_msgs::srv::SetEntityPose>::SharedPtr set_pose_entity_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reinit_global_loc_client_;
    bool reinit_in_flight_;
    rclcpp::Time last_reinit_time_;
    rclcpp::Duration reinit_cooldown_{0, 0};

    // Params / topics
    bool bad_init_test_;
    bool is_first_bad_init_;
    std::string bad_init_pose_topic_;
    std::string set_initial_pose_srv_;

    std::string fixed_frame_;
    std::string robot_frame_;

    std::string est_error_frame_;
    std::string gt_error_frame_;

    // Terminal pose capture. The convergence threshold is a methodological choice that
    // reaches the paper, so it is a declared parameter and is recorded in every event
    // rather than being implicit in the code.
    std::string amcl_pose_topic_;
    double terminal_converged_cov_threshold_m2_;
    geometry_msgs::msg::PoseWithCovarianceStamped last_amcl_pose_;
    bool have_amcl_pose_;

    double end_error_pos_threshold_m_;
    double end_error_yaw_threshold_rad_;

    double bad_init_lin_range_m_;
    double bad_init_yaw_range_rad_;

    double pose_apply_pos_tol_m_;
    double pose_apply_yaw_tol_rad_;
    double pose_apply_timeout_sec_;
    double pose_sequence_timeout_sec_;

    double correct_cov_xy_;
    double correct_cov_yaw_;
    double bad_cov_xy_;
    double bad_cov_yaw_;

    std::string set_pose_service_;
    std::string entity_name_;

    bool kidnap_enabled_;

    /// Whether a kidnap also forces an AMCL global reinit. False = true kidnapped-robot
    /// condition: the filter gets no hint and must recover from scan mismatch alone.
    bool kidnap_notify_localizer_;
    double kidnap_delay_sec_;                 // deprecated (kept for compatibility, not used)
    double kidnap_delay_min_sec_;
    double kidnap_delay_max_sec_;
    double kidnap_min_travel_m_;
    double kidnap_distance_m_;
    double kidnap_max_distance_m_;
    double kidnap_z_;
    int kidnap_max_sample_tries_;

    double kidnap_verify_pos_tol_m_;
    double kidnap_verify_yaw_tol_rad_;
    double kidnap_verify_timeout_sec_;

    std::string kidnap_event_topic_;

    // Kidnap state machine (one attempt per goal)
    enum class KidnapStage : int
    {
        IDLE = 0,
        WAIT_SERVICE_RESPONSE,
        VERIFY_GT,
        DONE
    };

    KidnapStage kidnap_stage_;
    rclcpp::Time goal_started_at_;
    double kidnap_sampled_delay_sec_;
    geometry_msgs::msg::PoseStamped start_gt_pose_;
    bool have_start_gt_;

    geometry_msgs::msg::Pose kidnap_target_pose_;
    unique_identifier_msgs::msg::UUID kidnap_attempt_id_;
    rclcpp::Time kidnap_event_time_;
    rclcpp::Time kidnap_verify_started_at_;

    rclcpp::Client<navlearn_msgs::srv::SetEntityPose>::SharedFuture kidnap_future_;

    // Recovery-on-failure state machine (async, driven by timerCallback)
    enum class RecoveryStage : int
    {
        IDLE = 0,
        TELEPORT_PENDING,
        REINIT_PENDING,
        WAIT_CONVERGENCE,
        DONE
    };

    RecoveryStage recovery_stage_;
    rclcpp::Time recovery_start_time_;
    double recovery_timeout_sec_;

    // Goals / episode state
    std::vector<geometry_msgs::msg::PoseStamped> goal_poses_;

    size_t idx_;
    bool active_;
    bool goal_request_inflight_;
    unique_identifier_msgs::msg::UUID goal_id_;

    rclcpp::Time stamp_received_;
    geometry_msgs::msg::PoseStamped start_pose_;

    double dwell_sec_;
    rclcpp::Time next_allowed_send_;
    std::string goal_source_;

    double goal_min_clearance_m_;
    int goal_occ_thresh_;
    bool goal_reject_unknown_;
    double goal_min_distance_m_;

    geometry_msgs::msg::PoseStamped spawn_pose_;
    bool have_spawn_pose_;

    std::vector<double> goal_poses_x_;
    std::vector<double> goal_poses_y_;
    std::vector<double> goal_poses_yaw_;

    int goals_num_;
    // Campaign-wide seed and this episode's index. Every random draw is derived from
    // these plus the goal index, via navlearn::seed::derive. The controller is
    // deliberately not an input, so all arms see identical conditions.
    int64_t campaign_seed_;
    int64_t run_index_;

    /// Seed for one draw in this episode. Wraps navlearn::seed::derive with the node's
    /// campaign seed and run index so call sites cannot accidentally omit either.
    uint64_t seedFor(navlearn::seed::Stream stream, uint64_t goal_index) const
    {
      return navlearn::seed::derive(
        static_cast<uint64_t>(campaign_seed_), stream,
        static_cast<uint64_t>(run_index_), goal_index);
    }

    rclcpp::QoS qos_profile_sub;

    std::string action_server_;
    std::string episode_pub_topic_;

    bool have_map_;
    nav_msgs::msg::OccupancyGrid latest_map_;

    // Pose injection sequence state machine
    enum class PoseSeqStage : int
    {
        IDLE = 0,
        SEND_CORRECT,
        WAIT_CORRECT_APPLIED,
        SEND_BAD,
        WAIT_BAD_APPLIED
    };

    bool pose_sequence_pending_;
    PoseSeqStage pose_seq_stage_;
    rclcpp::Time pose_seq_started_at_;
    rclcpp::Time pose_stage_started_at_;

    geometry_msgs::msg::PoseWithCovarianceStamped correct_pose_msg_;
    geometry_msgs::msg::PoseWithCovarianceStamped bad_pose_msg_;
    geometry_msgs::msg::PoseWithCovarianceStamped pose_wait_target_;

    // Map helpers
    bool hasClearanceCell(int col, int row, int width, int height, double res,
                const std::vector<int8_t> & data) const;

    bool inExclusionZone(double x, double y) const;

    void loadGoals();

    // TF helpers
    bool sampleStartPoseAt(const rclcpp::Time & t, geometry_msgs::msg::PoseStamped & pose);
    bool lookupPose(const std::string & child_frame, const rclcpp::Time & t, geometry_msgs::msg::PoseStamped & out);

    // Terminal ground truth
    navlearn_msgs::msg::TerminalPoseReport captureTerminalPose(
      const geometry_msgs::msg::PoseStamped & goal_pose, const rclcpp::Time & terminated_at);

    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void timerCallback();

    // Nav actions
    void sendGoal();
    void onGoalResponse(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & gh);
    void onFeedback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &,
                  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
    void onResult(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result);

    // Initial pose injection
    void onSetInitialPose(rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedFuture future);

    bool needCorrection(const geometry_msgs::msg::PoseStamped &ground_truth_pose, const geometry_msgs::msg::PoseStamped &estimated_pose);

    geometry_msgs::msg::PoseWithCovarianceStamped buildPoseWithCovariance(
        const geometry_msgs::msg::PoseStamped & base,
        double cov_xy,
        double cov_yaw);

    geometry_msgs::msg::PoseWithCovarianceStamped buildPerturbedPose(
        const geometry_msgs::msg::PoseStamped & base);

    bool sendInitialPoseRequest(const geometry_msgs::msg::PoseWithCovarianceStamped & pose_msg);

    bool isInitialPoseApplied(const geometry_msgs::msg::PoseWithCovarianceStamped & target);

    void startPoseSequence(const rclcpp::Time & t);
    void advancePoseSequence(const rclcpp::Time & t);

    // Kidnap helpers
    unique_identifier_msgs::msg::UUID makeUUID_(uint32_t seed);
    bool sampleKidnapPoseFromMap_(const geometry_msgs::msg::PoseStamped & ref, geometry_msgs::msg::Pose & out_pose);
    void publishKidnapEvent_(bool success);
    bool isKidnapGTVerified_();
    bool setEntityPose_(double x, double y, double z, double yaw);
    void maybeKidnap(const rclcpp::Time & now);
    void callReinitGlobalLocalization();
    void forceReinitGlobalLocalization();
    void initiateRecoveryTeleport(const geometry_msgs::msg::PoseStamped & goal_pose);
    void advanceRecovery(const rclcpp::Time & now);
};
}

#endif