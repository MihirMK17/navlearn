# TTC/TTR Integration + AMCL Tuning — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate TTC/TTR metrics into the NavLearn benchmark pipeline with recovery-on-failure, perturbation presets, and AMCL tuning to produce publishable degradation curves.

**Architecture:** Async RecoveryStage state machine in episode_manager (following existing KidnapStage/PoseSeqStage patterns), conditional launch of localization_metrics + gz_set_pose_server in benchmarks.launch.py via PythonExpression gates, perturbation level presets mapped to parameter values, and configurable TTC thresholds in localization_metrics.

**Tech Stack:** C++17 (ROS 2 Humble), Python 3 (launch files, analysis scripts), colcon/CMake/ament

**Spec:** `docs/superpowers/specs/2026-03-26-ttc-ttr-integration-design.md`

---

### Task 1: Add `goal_min_distance_m` parameter and filter in `loadGoals()`

**Files:**
- Modify: `src/navlearn_benchmarks/include/navlearn_benchmarks/episode_manager.hpp:181` (add member)
- Modify: `src/navlearn_benchmarks/src/episode_manager.cpp:63` (declare param)
- Modify: `src/navlearn_benchmarks/src/episode_manager.cpp:290-340` (add filter in loadGoals)
- Modify: `src/navlearn_benchmarks/config/episode_manager_defaults.yaml:21` (add param docs)

- [ ] **Step 1: Add `goal_min_distance_m_` member to hpp**

In `episode_manager.hpp`, after `goal_reject_unknown_` (line 183):

```cpp
double goal_min_distance_m_;
```

- [ ] **Step 2: Add `spawn_pose_` member to hpp**

In `episode_manager.hpp`, after `goal_min_distance_m_`:

```cpp
geometry_msgs::msg::PoseStamped spawn_pose_;
bool have_spawn_pose_;
```

- [ ] **Step 3: Declare parameter in constructor (episode_manager.cpp)**

After `goal_reject_unknown_` declaration (~line 65 in constructor):

```cpp
goal_min_distance_m_ = declare_parameter<double>("goal_min_distance_m", 0.0);
have_spawn_pose_ = false;
```

- [ ] **Step 4: Capture spawn pose in mapCallback**

In `mapCallback()`, after setting `have_map_ = true`, add spawn pose capture:

```cpp
if (!have_spawn_pose_) {
  if (sampleStartPoseAt(this->get_clock()->now(), spawn_pose_)) {
    have_spawn_pose_ = true;
    RCLCPP_INFO(get_logger(), "Captured spawn pose: (%.2f, %.2f)",
                spawn_pose_.pose.position.x, spawn_pose_.pose.position.y);
  }
}
```

- [ ] **Step 5: Add spawn pose retry in `timerCallback()`**

In `timerCallback()`, after the action server readiness check (~line 807), add:

```cpp
// Retry spawn pose capture if not yet available (TF may not be ready at map receipt)
if (!have_spawn_pose_ && have_map_) {
  if (sampleStartPoseAt(now, spawn_pose_)) {
    have_spawn_pose_ = true;
    RCLCPP_INFO(get_logger(), "Captured spawn pose (retry): (%.2f, %.2f)",
                spawn_pose_.pose.position.x, spawn_pose_.pose.position.y);
  }
}
```

- [ ] **Step 6: Add minimum distance filter in `loadGoals()` map_random branch**

In `episode_manager.cpp`, inside the `loadGoals()` map_random branch, after the `hasClearanceCell` check (line 307) and before `cell_index = idx; break;` (line 309), add:

```cpp
// Minimum inter-goal distance check
if (goal_min_distance_m_ > 0.0) {
  double ref_x, ref_y;
  if (goal_poses_.empty()) {
    if (have_spawn_pose_) {
      ref_x = spawn_pose_.pose.position.x;
      ref_y = spawn_pose_.pose.position.y;
    } else {
      ref_x = 0.0;
      ref_y = 0.0;
    }
  } else {
    ref_x = goal_poses_.back().pose.position.x;
    ref_y = goal_poses_.back().pose.position.y;
  }
  double d = std::hypot(x - ref_x, y - ref_y);
  if (d < goal_min_distance_m_) continue;
}
```

- [ ] **Step 7: Add parameter to defaults yaml**

In `episode_manager_defaults.yaml`, after `goal_reject_unknown: true` (line 21):

```yaml
goal_min_distance_m: 0.0   # min Euclidean distance between consecutive goals [m]; 0.0 = no filter; set to 4.0 for TTC/TTR
```

- [ ] **Step 8: Build and verify compilation**

Run: `cd /home/mihirmk/robot_ws && colcon build --packages-select navlearn_benchmarks --symlink-install 2>&1 | tail -5`
Expected: `navlearn_benchmarks` builds successfully with no errors.

- [ ] **Step 9: Commit**

```bash
git add src/navlearn_benchmarks/include/navlearn_benchmarks/episode_manager.hpp \
        src/navlearn_benchmarks/src/episode_manager.cpp \
        src/navlearn_benchmarks/config/episode_manager_defaults.yaml
git commit -m "feat: add goal_min_distance_m parameter for TTC/TTR experiments

Ensures consecutive goals are at least N meters apart so AMCL has enough
travel time to converge after perturbation/kidnap. Uses spawn_pose_ from
TF rather than hardcoded origin."
```

---

### Task 2: Add `--extra-arg` flag to `multi_run_harness.py`

**Files:**
- Modify: `src/navlearn_benchmarks/scripts/multi_run_harness.py:38-83` (add arg)
- Modify: `src/navlearn_benchmarks/scripts/multi_run_harness.py:106-117` (append to cmd)

- [ ] **Step 1: Add `--extra-arg` argument to parser**

In `multi_run_harness.py`, after the `--dry-run` argument (line 82), add:

```python
parser.add_argument(
    "--extra-arg",
    action="append",
    default=[],
    help="Additional launch arg passed to benchmarks.launch.py (repeatable), "
         "e.g. --extra-arg bad_init_test:=true --extra-arg perturbation_level:=easy",
)
```

- [ ] **Step 2: Append extra args to launch command**

In `run_benchmark()`, after line 117 (`f"json_path:={json_path}",`), add:

```python
    cmd.extend(args.extra_arg)
```

- [ ] **Step 3: Verify with dry-run**

Run: `python3 src/navlearn_benchmarks/scripts/multi_run_harness.py --episodes 1 --goals 3 --seed 42 --output-dir /tmp/test --profile aggressive --dry-run --extra-arg bad_init_test:=true --extra-arg perturbation_level:=easy`

Expected: Output shows command with `bad_init_test:=true` and `perturbation_level:=easy` appended.

- [ ] **Step 4: Commit**

```bash
git add src/navlearn_benchmarks/scripts/multi_run_harness.py
git commit -m "feat: add --extra-arg flag to multi_run_harness.py

Repeatable flag that appends additional launch args to benchmarks.launch.py
invocations. Each --extra-arg is a separate element in the subprocess cmd
list, avoiding whitespace-splitting ambiguity."
```

---

### Task 3: Make TTC error thresholds configurable in `localization_metrics`

**Files:**
- Modify: `src/navlearn_localization_eval/src/localization_metrics.cpp:39-41` (hardcoded → params)
- Modify: `src/navlearn_localization_eval/config/localization_metrics.yaml` (add params)

- [ ] **Step 1: Replace hardcoded TTC thresholds with `declare_parameter` calls**

In `localization_metrics.cpp`, replace the hardcoded initializer list values (lines 39-41):

Old:
```cpp
, error_x_threshold_(0.2)
, error_y_threshold_(0.2)
, error_yaw_threshold_(0.1)
```

New (move to body after other `declare_parameter` calls, after line 65):
```cpp
error_x_threshold_ = this->declare_parameter<double>("ttc_error_x_threshold_m", 0.2);
error_y_threshold_ = this->declare_parameter<double>("ttc_error_y_threshold_m", 0.2);
error_yaw_threshold_ = this->declare_parameter<double>("ttc_error_yaw_threshold_rad", 0.1);
```

And remove them from the initializer list (replace with default values that get overwritten):
```cpp
, error_x_threshold_(0.0)
, error_y_threshold_(0.0)
, error_yaw_threshold_(0.0)
```

- [ ] **Step 2: Add parameters to localization_metrics.yaml**

In `localization_metrics.yaml`, after `ttr_error_yaw_threshold_rad: 0.10` (line 17):

```yaml
    ttc_error_x_threshold_m: 0.20
    ttc_error_y_threshold_m: 0.20
    ttc_error_yaw_threshold_rad: 0.10
```

- [ ] **Step 3: Build and verify**

Run: `cd /home/mihirmk/robot_ws && colcon build --packages-select navlearn_localization_eval --symlink-install 2>&1 | tail -5`
Expected: Builds successfully.

- [ ] **Step 4: Commit**

```bash
git add src/navlearn_localization_eval/src/localization_metrics.cpp \
        src/navlearn_localization_eval/config/localization_metrics.yaml
git commit -m "feat: make TTC error thresholds configurable via parameters

Previously hardcoded to 0.2/0.2/0.1 in the constructor. Now read from
ttc_error_x_threshold_m, ttc_error_y_threshold_m, ttc_error_yaw_threshold_rad
parameters, matching how TTR thresholds are already configurable."
```

---

### Task 4: Add `RecoveryStage` state machine and `forceReinitGlobalLocalization()`

This is the largest task — the core recovery-on-failure logic.

**Files:**
- Modify: `src/navlearn_benchmarks/include/navlearn_benchmarks/episode_manager.hpp:144` (add enum + members)
- Modify: `src/navlearn_benchmarks/src/episode_manager.cpp:60-90` (declare params)
- Modify: `src/navlearn_benchmarks/src/episode_manager.cpp:803-839` (timerCallback recovery poll)
- Modify: `src/navlearn_benchmarks/src/episode_manager.cpp:922-988` (onResult recovery trigger)
- Modify: `src/navlearn_benchmarks/src/episode_manager.cpp:990-1024` (add forceReinit)

- [ ] **Step 1: Add `RecoveryStage` enum and members to hpp**

In `episode_manager.hpp`, after the `KidnapStage` enum block (after line 163), add:

```cpp
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
```

- [ ] **Step 2: Add recovery method declarations to hpp**

In `episode_manager.hpp`, after `callReinitGlobalLocalization()` (line 269), add:

```cpp
void forceReinitGlobalLocalization();
void initiateRecoveryTeleport(const geometry_msgs::msg::PoseStamped & goal_pose);
void advanceRecovery(const rclcpp::Time & now);
```

- [ ] **Step 3: Declare `recovery_timeout_sec_` parameter in constructor**

In `episode_manager.cpp`, after `kidnap_event_topic_` declaration (~line 113), add:

```cpp
recovery_timeout_sec_ = declare_parameter<double>("recovery_timeout_sec", 15.0);
recovery_stage_ = RecoveryStage::IDLE;
```

- [ ] **Step 4: Implement `forceReinitGlobalLocalization()`**

In `episode_manager.cpp`, after `callReinitGlobalLocalization()` (after line 1024), add:

```cpp
void EpisodeManager::forceReinitGlobalLocalization()
{
  if (!reinit_global_loc_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(),
                "Recovery: /reinitialize_global_localization not ready; skipping reinit.");
    return;
  }

  auto req = std::make_shared<std_srvs::srv::Empty::Request>();

  using SharedFuture = rclcpp::Client<std_srvs::srv::Empty>::SharedFuture;

  reinit_global_loc_client_->async_send_request(
    req,
    [this](SharedFuture /*future*/) {
      RCLCPP_INFO(get_logger(), "Recovery: forced AMCL global reinit (cooldown bypassed).");
    }
  );

  last_reinit_time_ = this->get_clock()->now();
}
```

- [ ] **Step 5: Implement `initiateRecoveryTeleport()`**

After `forceReinitGlobalLocalization()`, add:

```cpp
void EpisodeManager::initiateRecoveryTeleport(
    const geometry_msgs::msg::PoseStamped & goal_pose)
{
  RCLCPP_INFO(get_logger(),
              "Recovery: teleporting robot to goal (%.2f, %.2f) for re-localization.",
              goal_pose.pose.position.x, goal_pose.pose.position.y);

  const double yaw = tf2::getYaw(goal_pose.pose.orientation);
  if (!setEntityPose_(goal_pose.pose.position.x,
                      goal_pose.pose.position.y,
                      kidnap_z_,
                      yaw)) {
    RCLCPP_ERROR(get_logger(), "Recovery: setEntityPose_ failed. Skipping recovery.");
    recovery_stage_ = RecoveryStage::DONE;
  }
}
```

- [ ] **Step 6: Implement `advanceRecovery()` state machine**

After `initiateRecoveryTeleport()`, add:

```cpp
void EpisodeManager::advanceRecovery(const rclcpp::Time & now)
{
  switch (recovery_stage_) {
    case RecoveryStage::TELEPORT_PENDING:
    {
      // Wait for set_pose service future to complete.
      // setEntityPose_ is fire-and-forget via async; wait a fixed 0.5s for Gazebo.
      if ((now - recovery_start_time_).seconds() > 0.5) {
        RCLCPP_INFO(get_logger(), "Recovery: teleport assumed complete, reinitializing AMCL.");
        recovery_stage_ = RecoveryStage::REINIT_PENDING;
      }
      break;
    }

    case RecoveryStage::REINIT_PENDING:
    {
      forceReinitGlobalLocalization();

      // Publish correct initial pose at teleport location
      geometry_msgs::msg::PoseWithCovarianceStamped correct_pose =
          buildPoseWithCovariance(goal_poses_[idx_], correct_cov_xy_, correct_cov_yaw_);
      sendInitialPoseRequest(correct_pose);

      recovery_start_time_ = now;  // reset timer for convergence wait
      recovery_stage_ = RecoveryStage::WAIT_CONVERGENCE;
      RCLCPP_INFO(get_logger(), "Recovery: waiting for AMCL convergence (timeout %.1fs).",
                  recovery_timeout_sec_);
      break;
    }

    case RecoveryStage::WAIT_CONVERGENCE:
    {
      geometry_msgs::msg::PoseStamped gt_pose, est_pose;
      bool have_gt = lookupPose(gt_error_frame_, now, gt_pose);
      bool have_est = lookupPose(est_error_frame_, now, est_pose);

      if (have_gt && have_est && !needCorrection(gt_pose, est_pose)) {
        RCLCPP_INFO(get_logger(), "Recovery: AMCL converged after %.2fs.",
                    (now - recovery_start_time_).seconds());
        recovery_stage_ = RecoveryStage::DONE;
        break;
      }

      if ((now - recovery_start_time_).seconds() > recovery_timeout_sec_) {
        RCLCPP_WARN(get_logger(),
                    "Recovery: convergence timed out after %.1fs. Proceeding anyway.",
                    recovery_timeout_sec_);
        recovery_stage_ = RecoveryStage::DONE;
        break;
      }
      break;
    }

    case RecoveryStage::DONE:
    {
      RCLCPP_INFO(get_logger(), "Recovery: complete. Advancing to next goal.");
      idx_++;
      recovery_stage_ = RecoveryStage::IDLE;

      // Reset state for next goal
      next_allowed_send_ = now + rclcpp::Duration::from_seconds(dwell_sec_);
      goal_id_.uuid.fill(0);
      start_pose_ = geometry_msgs::msg::PoseStamped();
      pose_sequence_pending_ = false;
      pose_seq_stage_ = PoseSeqStage::IDLE;
      kidnap_stage_ = KidnapStage::IDLE;
      have_start_gt_ = false;
      kidnap_sampled_delay_sec_ = 0.0;

      if (idx_ >= goal_poses_.size()) {
        RCLCPP_INFO(get_logger(), "All goals done");
        rclcpp::shutdown();
      }
      break;
    }

    case RecoveryStage::IDLE:
    default:
      break;
  }
}
```

- [ ] **Step 7: Hook recovery into `timerCallback()`**

In `timerCallback()` (line 803), after the `active_` check (line 811-814), add a recovery check:

```cpp
// Recovery-on-failure state machine (async)
if (recovery_stage_ != RecoveryStage::IDLE) {
  advanceRecovery(now);
  return;
}
```

This goes right after:
```cpp
if (active_) {
  maybeKidnap(now);
  return;
}
```

- [ ] **Step 8: Modify `onResult()` to trigger recovery on failure**

In `onResult()`, replace the unconditional `idx_++` block (lines 970-987):

Old (lines 970-987):
```cpp
active_ = false;
idx_++;
next_allowed_send_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(dwell_sec_);

goal_id_.uuid.fill(0);
start_pose_ = geometry_msgs::msg::PoseStamped();

pose_sequence_pending_ = false;
pose_seq_stage_ = PoseSeqStage::IDLE;

kidnap_stage_ = KidnapStage::IDLE;
have_start_gt_ = false;
kidnap_sampled_delay_sec_ = 0.0;

if (idx_ >= goal_poses_.size()) {
  RCLCPP_INFO(get_logger(), "All goals done");
  rclcpp::shutdown();
}
```

New:
```cpp
active_ = false;

if (ev.result != navlearn_msgs::msg::EpisodeEvent::RESULT_SUCCEEDED &&
    (bad_init_test_ || kidnap_enabled_)) {
  // Recovery: teleport to current goal, re-localize, then advance
  RCLCPP_WARN(get_logger(), "Goal %zu FAILED during TTC/TTR experiment. Initiating recovery.", idx_);
  recovery_stage_ = RecoveryStage::TELEPORT_PENDING;
  recovery_start_time_ = this->get_clock()->now();
  initiateRecoveryTeleport(goal_poses_[idx_]);
} else {
  idx_++;
  next_allowed_send_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(dwell_sec_);

  goal_id_.uuid.fill(0);
  start_pose_ = geometry_msgs::msg::PoseStamped();

  pose_sequence_pending_ = false;
  pose_seq_stage_ = PoseSeqStage::IDLE;

  kidnap_stage_ = KidnapStage::IDLE;
  have_start_gt_ = false;
  kidnap_sampled_delay_sec_ = 0.0;

  if (idx_ >= goal_poses_.size()) {
    RCLCPP_INFO(get_logger(), "All goals done");
    rclcpp::shutdown();
  }
}
```

- [ ] **Step 9: Add `recovery_timeout_sec` to defaults yaml**

In `episode_manager_defaults.yaml`, after the kidnap section (after line 78):

```yaml

    # --- Recovery on Failure (TTC/TTR) ---
    recovery_timeout_sec: 15.0   # max time to wait for AMCL convergence during recovery [s]; [5.0, 60.0]
```

- [ ] **Step 10: Build and verify compilation**

Run: `cd /home/mihirmk/robot_ws && colcon build --packages-select navlearn_benchmarks --symlink-install 2>&1 | tail -10`
Expected: Builds successfully with no errors.

- [ ] **Step 11: Commit**

```bash
git add src/navlearn_benchmarks/include/navlearn_benchmarks/episode_manager.hpp \
        src/navlearn_benchmarks/src/episode_manager.cpp \
        src/navlearn_benchmarks/config/episode_manager_defaults.yaml
git commit -m "feat: add RecoveryStage state machine for TTC/TTR failure recovery

When a goal fails during bad_init_test or kidnap experiments, the robot is
teleported to the current goal pose and AMCL is re-initialized (bypassing
cooldown) before advancing to the next goal. Prevents cascading failures
from corrupt localization state. Async design avoids executor deadlock."
```

---

### Task 5: Launch integration — add `localization_metrics` + `gz_set_pose_server` to `benchmarks.launch.py`

**Files:**
- Modify: `src/navlearn_benchmarks/launch/benchmarks.launch.py`

- [ ] **Step 1: Add new imports**

At the top of `benchmarks.launch.py`, add to the existing imports:

```python
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
```

- [ ] **Step 2: Add `perturbation_level` launch argument**

After `collision_scan_threshold_m_arg` (~line 105), add:

```python
perturbation_level_arg = DeclareLaunchArgument(
    "perturbation_level",
    default_value="medium",
    description="Perturbation preset for TTC/TTR: easy | medium | hard | extreme")
perturbation_level = LaunchConfiguration("perturbation_level")

goal_min_distance_m_arg = DeclareLaunchArgument(
    "goal_min_distance_m",
    default_value="0.0",
    description="Min Euclidean distance between consecutive goals [m]. "
                "Set to 4.0 for TTC/TTR experiments.")
goal_min_distance_m = LaunchConfiguration("goal_min_distance_m")

ttc_timeout_sec_arg = DeclareLaunchArgument(
    "ttc_timeout_sec", default_value="10.0",
    description="TTC convergence timeout passed to localization_metrics [s]")
ttc_timeout_sec = LaunchConfiguration("ttc_timeout_sec")

ttr_timeout_sec_arg = DeclareLaunchArgument(
    "ttr_timeout_sec", default_value="10.0",
    description="TTR recovery timeout passed to localization_metrics [s]")
ttr_timeout_sec = LaunchConfiguration("ttr_timeout_sec")

recovery_timeout_sec_arg = DeclareLaunchArgument(
    "recovery_timeout_sec", default_value="15.0",
    description="Recovery convergence timeout for episode_manager [s]")
recovery_timeout_sec = LaunchConfiguration("recovery_timeout_sec")
```

- [ ] **Step 3: Add `goal_min_distance_m` and `recovery_timeout_sec` to episode_manager node params**

In the `episode_manager` Node definition (~line 161-177), add to the parameters dict:

```python
"goal_min_distance_m": goal_min_distance_m,
"recovery_timeout_sec": recovery_timeout_sec,
```

- [ ] **Step 4: Add localization_metrics YAML path**

After the `episode_manager_config_path` PathJoinSubstitution (~line 111):

```python
localization_metrics_yaml = PathJoinSubstitution([
    FindPackageShare("navlearn_localization_eval"),
    "config",
    "localization_metrics.yaml"
])

gz_set_pose_yaml = PathJoinSubstitution([
    FindPackageShare("navlearn_localization_eval"),
    "config",
    "gz_set_pose_server.yaml"
])
```

- [ ] **Step 5: Add `localization_metrics` node (conditionally launched)**

After the `ground_truth_publisher` Node (~line 233):

```python
localization_metrics_node = Node(
    condition=IfCondition(
        PythonExpression([
            "'", bad_init_test, "' == 'true' or '",
            kidnap_test, "' == 'true'"
        ])
    ),
    package='navlearn_localization_eval',
    executable='localization_metrics',
    name='localization_metrics',
    output='log',
    parameters=[
        localization_metrics_yaml,
        {'use_sim_time': use_sim_time,
         'bad_init_test': bad_init_test,
         'kidnap_test': kidnap_test,
         'ttc_timeout_sec': ttc_timeout_sec,
         'ttr_timeout_sec': ttr_timeout_sec},
    ],
)
```

- [ ] **Step 6: Add `gz_set_pose_server` node (conditionally launched)**

After `localization_metrics_node`:

```python
gz_set_pose_server_node = Node(
    condition=IfCondition(
        PythonExpression(["'", kidnap_test, "' == 'true'"])
    ),
    package='navlearn_localization_eval',
    executable='gz_set_pose_server',
    name='gz_set_pose_server',
    output='log',
    parameters=[
        gz_set_pose_yaml,
        {'use_sim_time': use_sim_time,
         'world_name': world_name},
    ],
)
```

- [ ] **Step 7: Add new args and nodes to LaunchDescription return**

In the `LaunchDescription` return list (~line 247-272), add the new args and nodes:

After `collision_scan_threshold_m_arg,`:
```python
perturbation_level_arg,
goal_min_distance_m_arg,
ttc_timeout_sec_arg,
ttr_timeout_sec_arg,
recovery_timeout_sec_arg,
```

After `ground_truth_publisher,`:
```python
localization_metrics_node,
gz_set_pose_server_node,
```

- [ ] **Step 8: Verify launch file syntax**

Run: `python3 -c "import importlib.util; spec = importlib.util.spec_from_file_location('m', 'src/navlearn_benchmarks/launch/benchmarks.launch.py'); mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod); ld = mod.generate_launch_description(); print(f'OK: {len(ld.entities)} entities')"`

Expected: `OK: N entities` (no import or syntax errors).

- [ ] **Step 9: Commit**

```bash
git add src/navlearn_benchmarks/launch/benchmarks.launch.py
git commit -m "feat: add localization_metrics + gz_set_pose_server to benchmarks.launch.py

Both nodes are conditionally launched via PythonExpression when bad_init_test
or kidnap_enabled is 'true'. Adds perturbation_level, goal_min_distance_m,
ttc/ttr_timeout_sec, and recovery_timeout_sec launch arguments."
```

---

### Task 6: Add perturbation level preset mapping to `benchmarks.launch.py`

**Files:**
- Modify: `src/navlearn_benchmarks/launch/benchmarks.launch.py`

**IMPORTANT — Existing launch arg conflict:** `kidnap_distance_m` and `kidnap_max_distance_m`
are already declared at lines 76-82 of `benchmarks.launch.py` with defaults `0.20` and `1.1`.
`bad_init_lin_range_m` and `bad_init_yaw_range_rad` are NOT declared as launch args (they come
from the episode_manager YAML config). The OpaqueFunction must unconditionally set all four
preset values in `context.launch_configurations`, overriding existing defaults. The existing
`kidnap_distance_m_arg` and `kidnap_max_distance_m_arg` declarations remain as the
fallback defaults when `perturbation_level` is not set.

- [ ] **Step 1: Add OpaqueFunction import and preset mapping**

Add to imports:
```python
from launch.actions import OpaqueFunction
```

Add the preset mapping function before `generate_launch_description()`:

```python
def apply_perturbation_presets(context, *args, **kwargs):
    """Map perturbation_level string to concrete parameter values.

    Always overrides kidnap_distance_m, kidnap_max_distance_m,
    bad_init_lin_range_m, and bad_init_yaw_range_rad based on the
    perturbation_level. If a user wants custom values, they should
    NOT set perturbation_level and instead pass params directly.
    """
    level = context.launch_configurations.get('perturbation_level', 'medium')

    presets = {
        'easy':    {'bad_init_lin_range_m': '0.25', 'bad_init_yaw_range_rad': '0.087',
                    'kidnap_distance_m': '0.3',  'kidnap_max_distance_m': '0.5'},
        'medium':  {'bad_init_lin_range_m': '0.50', 'bad_init_yaw_range_rad': '0.262',
                    'kidnap_distance_m': '0.8',  'kidnap_max_distance_m': '1.2'},
        'hard':    {'bad_init_lin_range_m': '1.00', 'bad_init_yaw_range_rad': '0.524',
                    'kidnap_distance_m': '1.5',  'kidnap_max_distance_m': '2.5'},
        'extreme': {'bad_init_lin_range_m': '2.00', 'bad_init_yaw_range_rad': '0.785',
                    'kidnap_distance_m': '2.5',  'kidnap_max_distance_m': '4.0'},
    }

    if level in presets:
        for key, val in presets[level].items():
            context.launch_configurations[key] = val

    return []
```

- [ ] **Step 2: Add `bad_init_lin_range_m` and `bad_init_yaw_range_rad` launch arg declarations**

These two are NOT currently declared as launch args (they come from episode_manager YAML).
Add them BEFORE the OpaqueFunction so they exist as launch configurations:

```python
bad_init_lin_range_m_arg = DeclareLaunchArgument(
    "bad_init_lin_range_m", default_value="0.50",
    description="TTC position perturbation half-range [m]")
bad_init_lin_range_m = LaunchConfiguration("bad_init_lin_range_m")

bad_init_yaw_range_rad_arg = DeclareLaunchArgument(
    "bad_init_yaw_range_rad", default_value="0.262",
    description="TTC yaw perturbation half-range [rad]")
bad_init_yaw_range_rad = LaunchConfiguration("bad_init_yaw_range_rad")
```

Do NOT re-declare `kidnap_distance_m` or `kidnap_max_distance_m` — they already exist
at lines 76-82. Use `LaunchConfiguration` references to the existing declarations:

```python
kidnap_distance_m = LaunchConfiguration("kidnap_distance_m")
kidnap_max_distance_m_lc = LaunchConfiguration("kidnap_max_distance_m")
```

- [ ] **Step 3: Add OpaqueFunction to LaunchDescription**

In the `LaunchDescription` return, add in this order:

1. After the existing `kidnap_max_distance_m_arg,` — add:
```python
bad_init_lin_range_m_arg,
bad_init_yaw_range_rad_arg,
```

2. After `perturbation_level_arg,` — add:
```python
OpaqueFunction(function=apply_perturbation_presets),
```

The OpaqueFunction MUST come AFTER all four `DeclareLaunchArgument` entries AND after
`perturbation_level_arg`, but BEFORE any Node that reads these configurations.

- [ ] **Step 4: Update episode_manager node to pass perturbation params**

Add to the `episode_manager` Node parameters dict:

```python
"bad_init_lin_range_m": bad_init_lin_range_m,
"bad_init_yaw_range_rad": bad_init_yaw_range_rad,
"kidnap_distance_m": kidnap_distance_m,
"kidnap_max_distance_m": kidnap_max_distance_m_lc,
```

- [ ] **Step 5: Verify launch file syntax**

Run: `python3 -c "import importlib.util; spec = importlib.util.spec_from_file_location('m', 'src/navlearn_benchmarks/launch/benchmarks.launch.py'); mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod); ld = mod.generate_launch_description(); print(f'OK: {len(ld.entities)} entities')"`

Expected: `OK: N entities` (no errors).

- [ ] **Step 6: Commit**

```bash
git add src/navlearn_benchmarks/launch/benchmarks.launch.py
git commit -m "feat: add perturbation level presets to benchmarks.launch.py

Maps perturbation_level (easy/medium/hard/extreme) to concrete parameter
values for bad_init_lin_range_m, bad_init_yaw_range_rad, kidnap_distance_m,
and kidnap_max_distance_m via OpaqueFunction."
```

---

### Task 7: AMCL + LiDAR tuning (Phase 2 parameter changes)

**Files:**
- Modify: `src/bumperbot_localization/config/amcl.yaml`
- Modify: `src/bumperbot_description/urdf/bumperbot_gazebo.xacro:138-166`
- Modify: `src/navlearn_localization_eval/config/localization_metrics.yaml`

**Note:** bumperbot_* packages are normally read-only. Exception granted for this localization research per user approval. Changes limited to parameter values only.

- [ ] **Step 1: Read current AMCL config**

Run: Read `src/bumperbot_localization/config/amcl.yaml` — verify current values match spec expectations.

- [ ] **Step 2: Update AMCL parameters**

Change these values in `amcl.yaml`:
- `max_particles`: 8000 → 15000
- `max_beams`: 240 → 480
- `recovery_alpha_slow`: 0.001 → 0.005
- `z_rand`: 0.05 → 0.10

- [ ] **Step 3: Read current LiDAR config in xacro**

Run: Read `src/bumperbot_description/urdf/bumperbot_gazebo.xacro` lines 138-166 — verify LiDAR `samples` value.

- [ ] **Step 4: Update LiDAR samples**

Change `<samples>360</samples>` → `<samples>720</samples>` in `bumperbot_gazebo.xacro`.

- [ ] **Step 5: Update localization_metrics timeouts**

In `localization_metrics.yaml`, change:
- `ttc_timeout_sec`: 10.0 → 30.0
- `ttr_timeout_sec`: 10.0 → 30.0

- [ ] **Step 6: Build affected packages**

Run: `cd /home/mihirmk/robot_ws && colcon build --packages-select bumperbot_localization bumperbot_description navlearn_localization_eval --symlink-install 2>&1 | tail -5`
Expected: All three build successfully.

- [ ] **Step 7: Commit**

```bash
git add src/bumperbot_localization/config/amcl.yaml \
        src/bumperbot_description/urdf/bumperbot_gazebo.xacro \
        src/navlearn_localization_eval/config/localization_metrics.yaml
git commit -m "feat: tune AMCL + LiDAR params for localization recovery (Phase 2)

AMCL: max_particles 8k→15k, max_beams 240→480, recovery_alpha_slow
0.001→0.005, z_rand 0.05→0.10.
LiDAR: samples 360→720.
Timeouts: ttc/ttr 10s→30s.
These changes improve AMCL's ability to recover from perturbation/kidnap
with 2D LiDAR in simulation."
```

---

### Task 8: Create `analyze_ttc_ttr.py` analysis script

**Files:**
- Create: `src/navlearn_benchmarks/scripts/analyze_ttc_ttr.py`

- [ ] **Step 1: Create the analysis script**

Create `src/navlearn_benchmarks/scripts/analyze_ttc_ttr.py` with:

```python
#!/usr/bin/env python3
# Copyright 2026 NavLearn Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License")
# ...
"""
Analyze TTC/TTR experiment results and produce degradation curves.

Reads localization_evaluation.csv files from structured experiment directories
and produces:
  1. Degradation curves (success rate vs perturbation level)
  2. Recovery time box plots
  3. Before/after comparison table (Phase 1 vs Phase 2)
  4. Aggregated tidy CSV

Usage:
    python3 analyze_ttc_ttr.py \
        --phase1-dir benchmark_reports/runs/ttc_ttr/phase1 \
        --phase2-dir benchmark_reports/runs/ttc_ttr/phase2 \
        --output-dir benchmark_reports/analysis/ttc_ttr
"""

import argparse
import csv
import pathlib
import sys
from collections import defaultdict

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


LEVELS = ['easy', 'medium', 'hard', 'extreme']
MODES = ['ttc', 'ttr']


def parse_args():
    parser = argparse.ArgumentParser(
        description="Analyze TTC/TTR experiment results",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--phase1-dir", type=str, required=True,
                        help="Root directory for Phase 1 results")
    parser.add_argument("--phase2-dir", type=str, default=None,
                        help="Root directory for Phase 2 results (optional)")
    parser.add_argument("--output-dir", type=str, required=True,
                        help="Directory for output plots and CSV")
    return parser.parse_args()


def find_csv_files(base_dir, mode, level):
    """Find localization CSV files for a given mode/level combination."""
    pattern = f"{mode}_{level}"
    search_dir = pathlib.Path(base_dir) / pattern
    if not search_dir.exists():
        return []
    return sorted(search_dir.glob("*localization*.csv"))


def parse_localization_csv(csv_path):
    """Parse a single localization_evaluation.csv and extract TTC/TTR rows."""
    results = []
    try:
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                results.append(row)
    except Exception as e:
        print(f"Warning: Could not read {csv_path}: {e}", file=sys.stderr)
    return results


def compute_success_rate(rows, metric_key):
    """Compute success rate for a given metric (ttc or ttr)."""
    if not rows:
        return 0.0, 0, 0
    # Look for outcome code column
    outcome_col = f"{metric_key}_outcome"
    time_col = f"{metric_key}_time_sec"

    successes = 0
    total = 0
    for row in rows:
        if outcome_col in row and row[outcome_col]:
            total += 1
            try:
                outcome = int(float(row[outcome_col]))
                if outcome == 0:  # 0 = success
                    successes += 1
            except (ValueError, KeyError):
                pass

    rate = (successes / total * 100.0) if total > 0 else 0.0
    return rate, successes, total


def collect_recovery_times(rows, metric_key):
    """Collect successful recovery times for box plots."""
    time_col = f"{metric_key}_time_sec"
    outcome_col = f"{metric_key}_outcome"
    times = []
    for row in rows:
        if outcome_col in row and time_col in row:
            try:
                outcome = int(float(row[outcome_col]))
                if outcome == 0:
                    times.append(float(row[time_col]))
            except (ValueError, KeyError):
                pass
    return times


def aggregate_phase(base_dir):
    """Aggregate all results for a phase into a structured dict."""
    data = {}
    for mode in MODES:
        data[mode] = {}
        for level in LEVELS:
            csvs = find_csv_files(base_dir, mode, level)
            all_rows = []
            for csv_path in csvs:
                all_rows.extend(parse_localization_csv(csv_path))
            rate, succ, total = compute_success_rate(all_rows, mode)
            times = collect_recovery_times(all_rows, mode)
            data[mode][level] = {
                'success_rate': rate,
                'successes': succ,
                'total': total,
                'recovery_times': times,
                'rows': all_rows,
            }
    return data


def plot_degradation_curves(phase1_data, phase2_data, output_dir):
    """Plot success rate vs perturbation level."""
    if not HAS_MATPLOTLIB:
        print("matplotlib not available — skipping plots.", file=sys.stderr)
        return

    fig, ax = plt.subplots(figsize=(10, 6))

    x = range(len(LEVELS))

    for mode in MODES:
        label_prefix = mode.upper()
        p1_rates = [phase1_data[mode][lv]['success_rate'] for lv in LEVELS]
        ax.plot(x, p1_rates, marker='o', linestyle='-',
                label=f'{label_prefix} Phase 1 (current params)')

        if phase2_data:
            p2_rates = [phase2_data[mode][lv]['success_rate'] for lv in LEVELS]
            ax.plot(x, p2_rates, marker='s', linestyle='--',
                    label=f'{label_prefix} Phase 2 (tuned params)')

    ax.set_xticks(x)
    ax.set_xticklabels(LEVELS)
    ax.set_xlabel('Perturbation Level')
    ax.set_ylabel('Success Rate (%)')
    ax.set_title('AMCL Recovery Degradation Curve — TTC/TTR vs Perturbation Level')
    ax.set_ylim(-5, 105)
    ax.legend()
    ax.grid(True, alpha=0.3)

    out_path = pathlib.Path(output_dir) / 'degradation_curves.png'
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {out_path}")


def plot_recovery_time_boxplots(phase1_data, phase2_data, output_dir):
    """Plot recovery time distributions per level."""
    if not HAS_MATPLOTLIB:
        return

    for mode in MODES:
        fig, ax = plt.subplots(figsize=(10, 6))
        positions = []
        box_data = []
        labels = []
        pos = 1

        for lv in LEVELS:
            times_p1 = phase1_data[mode][lv]['recovery_times']
            if times_p1:
                box_data.append(times_p1)
                labels.append(f'{lv}\nP1')
                positions.append(pos)
                pos += 1

            if phase2_data:
                times_p2 = phase2_data[mode][lv]['recovery_times']
                if times_p2:
                    box_data.append(times_p2)
                    labels.append(f'{lv}\nP2')
                    positions.append(pos)
            pos += 1.5

        if box_data:
            ax.boxplot(box_data, positions=positions[:len(box_data)])
            ax.set_xticklabels(labels[:len(box_data)])
            ax.set_ylabel('Recovery Time (s)')
            ax.set_title(f'{mode.upper()} Recovery Time Distribution')
            ax.grid(True, alpha=0.3, axis='y')

            out_path = pathlib.Path(output_dir) / f'{mode}_recovery_times.png'
            fig.savefig(out_path, dpi=150, bbox_inches='tight')
            print(f"Saved: {out_path}")

        plt.close(fig)


def write_comparison_table(phase1_data, phase2_data, output_dir):
    """Write markdown comparison table."""
    out_path = pathlib.Path(output_dir) / 'comparison_table.md'
    with open(out_path, 'w') as f:
        f.write("# TTC/TTR Before/After Comparison\n\n")
        for mode in MODES:
            f.write(f"## {mode.upper()}\n\n")
            f.write("| Level | Phase 1 Rate | Phase 1 N | ")
            if phase2_data:
                f.write("Phase 2 Rate | Phase 2 N | Delta |\n")
                f.write("|-------|-------------|-----------|")
                f.write("-------------|-----------|-------|\n")
            else:
                f.write("\n|-------|-------------|----------|\n")

            for lv in LEVELS:
                p1 = phase1_data[mode][lv]
                row = f"| {lv} | {p1['success_rate']:.1f}% | {p1['successes']}/{p1['total']} |"
                if phase2_data:
                    p2 = phase2_data[mode][lv]
                    delta = p2['success_rate'] - p1['success_rate']
                    sign = '+' if delta >= 0 else ''
                    row += f" {p2['success_rate']:.1f}% | {p2['successes']}/{p2['total']} | {sign}{delta:.1f}% |"
                f.write(row + "\n")
            f.write("\n")
    print(f"Saved: {out_path}")


def write_tidy_csv(phase1_data, phase2_data, output_dir):
    """Write all data points to a single tidy CSV."""
    out_path = pathlib.Path(output_dir) / 'ttc_ttr_all_data.csv'
    with open(out_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['phase', 'mode', 'level', 'success_rate_pct',
                         'successes', 'total', 'mean_recovery_time_s'])

        for phase_label, data in [('phase1', phase1_data), ('phase2', phase2_data)]:
            if data is None:
                continue
            for mode in MODES:
                for lv in LEVELS:
                    d = data[mode][lv]
                    mean_time = (sum(d['recovery_times']) / len(d['recovery_times'])
                                 if d['recovery_times'] else float('nan'))
                    writer.writerow([phase_label, mode, lv,
                                     f"{d['success_rate']:.1f}",
                                     d['successes'], d['total'],
                                     f"{mean_time:.3f}"])
    print(f"Saved: {out_path}")


def main():
    args = parse_args()
    output_dir = pathlib.Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Phase 1 directory: {args.phase1_dir}")
    phase1_data = aggregate_phase(args.phase1_dir)

    phase2_data = None
    if args.phase2_dir:
        print(f"Phase 2 directory: {args.phase2_dir}")
        phase2_data = aggregate_phase(args.phase2_dir)

    plot_degradation_curves(phase1_data, phase2_data, output_dir)
    plot_recovery_time_boxplots(phase1_data, phase2_data, output_dir)
    write_comparison_table(phase1_data, phase2_data, output_dir)
    write_tidy_csv(phase1_data, phase2_data, output_dir)

    print("\nAnalysis complete.")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Make executable**

Run: `chmod +x src/navlearn_benchmarks/scripts/analyze_ttc_ttr.py`

- [ ] **Step 3: Verify script loads without errors**

Run: `python3 -c "import importlib.util; spec = importlib.util.spec_from_file_location('m', 'src/navlearn_benchmarks/scripts/analyze_ttc_ttr.py'); mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod); print('OK')"`
Expected: `OK`

- [ ] **Step 4: Commit**

```bash
git add src/navlearn_benchmarks/scripts/analyze_ttc_ttr.py
git commit -m "feat: add analyze_ttc_ttr.py for degradation curve analysis

Reads localization CSV files from structured experiment directories and
produces degradation curves, recovery time box plots, comparison tables,
and aggregated tidy CSV for Paper 1."
```

---

### Task 9: Full integration build and colcon test

**Files:** None (verification only)

- [ ] **Step 1: Full workspace build**

Run: `cd /home/mihirmk/robot_ws && colcon build --packages-select navlearn_benchmarks navlearn_localization_eval --symlink-install 2>&1 | tail -15`
Expected: Both packages build successfully.

- [ ] **Step 2: Verify launch file loads with TTC/TTR args**

Run: `ros2 launch navlearn_benchmarks benchmarks.launch.py bad_init_test:=true perturbation_level:=easy goal_min_distance_m:=4.0 --print-description 2>&1 | head -30`
Expected: Shows launch description including localization_metrics and gz_set_pose_server nodes.

- [ ] **Step 3: Commit (if any fixups needed)**

Only commit if Step 1 or Step 2 revealed issues that required fixes.
