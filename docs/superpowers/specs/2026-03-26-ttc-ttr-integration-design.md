# TTC/TTR Integration + AMCL Tuning — Design Spec

**Date:** 2026-03-26
**Status:** Approved
**Profile:** Aggressive only (for all TTC/TTR experiments)

---

## 1. Objective

Integrate TTC (Time to Converge) and TTR (Time to Recover) metrics into the NavLearn benchmark pipeline, tune AMCL for localization recovery, and produce publishable degradation curves showing AMCL recovery limits with 2D LiDAR.

### Dual goals

- **A (Tune):** Make TTC/TTR functional by tuning AMCL and LiDAR simulation parameters so recovery works at moderate perturbation levels.
- **B (Document):** Run structured perturbation experiments at increasing levels to map the degradation curve — where AMCL recovers reliably and where it breaks. This IS the publishable finding.

---

## 2. Goal Generation: Minimum Distance Filter

### Problem

Short goals (< 5s travel time) don't give AMCL enough time to converge after perturbation/kidnap. TTC/TTR measurements get cut short (outcome code 2 = "ended early"), producing ambiguous data.

### Solution

New parameter in `episode_manager`:

```yaml
goal_min_distance_m: 0.0  # default: no filter (backward compatible)
                           # set to 4.0 for TTC/TTR experiments
```

### Implementation

In `EpisodeManager::loadGoals()`, map_random branch (episode_manager.cpp ~line 290):

After existing checks (`inExclusionZone`, `hasClearanceCell`), add:

```cpp
// Minimum inter-goal distance check
if (goal_min_distance_m_ > 0.0) {
  double ref_x, ref_y;
  if (goal_poses_.empty()) {
    // First goal: check distance from robot's actual spawn pose
    // Use spawn_pose_ (captured from TF at map receipt or node init)
    ref_x = spawn_pose_.pose.position.x;
    ref_y = spawn_pose_.pose.position.y;
  } else {
    // Subsequent goals: check distance from previous goal
    ref_x = goal_poses_.back().pose.position.x;
    ref_y = goal_poses_.back().pose.position.y;
  }
  double d = std::hypot(x - ref_x, y - ref_y);
  if (d < goal_min_distance_m_) continue;
}
```

**Note:** `spawn_pose_` is captured from TF (`robot_frame_` in `fixed_frame_`) when the
map is first received, using the existing `sampleStartPoseAt()` method. This avoids
hardcoding `(0, 0)` which may not match the actual spawn position in all worlds.

### Files changed

- `episode_manager.cpp`: Add distance filter in `loadGoals()`
- `episode_manager.hpp`: Add `goal_min_distance_m_` member
- `episode_manager_defaults.yaml`: Add `goal_min_distance_m: 0.0`
- `benchmarks.launch.py`: Add launch arg, auto-set to 4.0 when `bad_init_test` or `kidnap_enabled`

---

## 3. Launch Integration

### Problem

`localization_metrics` and `gz_set_pose_server` are only in `localization_eval.launch.py`, not in `benchmarks.launch.py`. Running benchmarks alone means:
- TTC/TTR metrics are never collected
- Kidnap teleport silently fails (service unavailable)

### Solution

Add both nodes to `benchmarks.launch.py`, gated by `bad_init_test` or `kidnap_enabled`:

```python
# Gate: launch localization eval nodes when TTC or TTR is active
bad_init_test = LaunchConfiguration('bad_init_test')
kidnap_enabled = LaunchConfiguration('kidnap_enabled')

localization_metrics_node = Node(
    condition=IfCondition(
        PythonExpression([
            "'", bad_init_test, "' == 'true' or '",
            kidnap_enabled, "' == 'true'"
        ])
    ),
    package='navlearn_localization_eval',
    executable='localization_metrics',
    parameters=[
        localization_metrics_yaml,
        {'use_sim_time': True},
        {'bad_init_test': bad_init_test},
        {'kidnap_test': kidnap_enabled},
    ],
)

gz_set_pose_server_node = Node(
    condition=IfCondition(
        PythonExpression(["'", kidnap_enabled, "' == 'true'"])
    ),
    package='navlearn_localization_eval',
    executable='gz_set_pose_server',
    parameters=[
        {'use_sim_time': True},
        {'world_name': LaunchConfiguration('world_name')},
    ],
)
```

**Note:** `PythonExpression` in ROS 2 Humble receives string substitutions, not Python
booleans. The explicit `== 'true'` comparison is required — bare `"true or false"` would
always evaluate truthy in Python string context.

### New launch arguments

| Argument | Default | Purpose |
|----------|---------|---------|
| `perturbation_level` | `medium` | Preset: easy/medium/hard/extreme |
| `goal_min_distance_m` | `0.0` | Auto-set to 4.0 when TTC/TTR active |
| `ttc_timeout_sec` | `10.0` | Passed to localization_metrics |
| `ttr_timeout_sec` | `10.0` | Passed to localization_metrics |

### Files changed

- `benchmarks.launch.py`: Add 2 nodes + new launch args + perturbation preset mapping

---

## 4. Recovery-on-Failure in Episode Manager

### Problem

When a goal fails during TTC/TTR experiments, `onResult` just increments `idx_` and moves on. The robot is wherever Nav2 left it (potentially far from the goal, with corrupt localization). The next goal starts from a broken state, causing cascading failures.

### Solution

New recovery state machine stage driven by `timerCallback()`, following the same
async pattern as `KidnapStage` and `PoseSeqStage`. Recovery MUST NOT block `onResult`
because that callback holds the single-threaded executor — blocking it would prevent
TF updates, causing `needCorrection()` to use stale transforms and always time out.

### Recovery state machine

```
enum class RecoveryStage : int {
  IDLE,                    // No recovery in progress
  TELEPORT_PENDING,        // setEntityPose_() called, waiting for service response
  REINIT_PENDING,          // Teleport confirmed, calling AMCL reinit
  WAIT_CONVERGENCE,        // Polling needCorrection() for convergence
  DONE                     // Recovery complete (success or timeout)
};
```

### Recovery sequence (async, driven by timerCallback)

```
1. onResult detects FAILED + (bad_init_test_ || kidnap_enabled_):
   → Set recovery_stage_ = TELEPORT_PENDING
   → Call setEntityPose_(goal_pose)
   → Do NOT increment idx_ yet

2. timerCallback polls recovery_stage_:
   TELEPORT_PENDING:
     → When service future ready: set recovery_stage_ = REINIT_PENDING

   REINIT_PENDING:
     → Call AMCL reinit DIRECTLY (bypass cooldown gate — see note below)
     → Set recovery_stage_ = WAIT_CONVERGENCE
     → Record recovery_start_time_

   WAIT_CONVERGENCE:
     → Poll needCorrection() every timer tick (100ms)
     → If converged: log success, recovery_stage_ = DONE
     → If elapsed > recovery_timeout_sec_ (15s): log warning, recovery_stage_ = DONE

   DONE:
     → Increment idx_, reset recovery state, proceed to next goal
```

### Reinit cooldown bypass

`callReinitGlobalLocalization()` has a 10-second `reinit_cooldown_` that will silently
no-op the recovery reinit in TTR mode (because kidnap already called reinit during the
goal). The recovery must call the AMCL service directly, bypassing the cooldown:

```cpp
void EpisodeManager::forceReinitGlobalLocalization() {
  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  reinit_global_loc_client_->async_send_request(req);
  last_reinit_time_ = this->get_clock()->now();  // update timestamp
  RCLCPP_INFO(get_logger(), "Recovery: forced AMCL global reinit (cooldown bypassed)");
}
```

### Integration point

In `onResult()` (episode_manager.cpp ~line 960), replace unconditional `idx_++`:

```cpp
if (result_code != EpisodeEvent::RESULT_SUCCEEDED &&
    (bad_init_test_ || kidnap_enabled_)) {
  // Don't increment idx_ — recovery will do it when DONE
  recovery_stage_ = RecoveryStage::TELEPORT_PENDING;
  initiateRecoveryTeleport(goal_poses_[idx_]);
} else {
  idx_++;
}
```

### Files changed

- `episode_manager.cpp`: Add `RecoveryStage` state machine in `timerCallback()`,
  `forceReinitGlobalLocalization()`, `initiateRecoveryTeleport()`
- `episode_manager.hpp`: Declare `RecoveryStage` enum, new members
  (`recovery_stage_`, `recovery_start_time_`, `recovery_timeout_sec_`)

---

## 5. AMCL Tuning Parameters

### Phase 1: Current parameters (baseline measurement)

Run experiments with existing params to establish "before" data.

### Phase 2: Tuned parameters

| Parameter | File | Current | Tuned | Rationale |
|-----------|------|---------|-------|-----------|
| LiDAR `samples` | `bumperbot_gazebo.xacro` | 360 | 720 | 2x more scan constraints, closer to real RPLidar A1 |
| AMCL `max_beams` | `amcl.yaml` | 240 | 480 | Use more available rays for matching |
| AMCL `recovery_alpha_slow` | `amcl.yaml` | 0.001 | 0.005 | Detect "lost" state 5x faster |
| AMCL `z_rand` | `amcl.yaml` | 0.05 | 0.10 | Prevent particle depletion during recovery |
| AMCL `max_particles` | `amcl.yaml` | 8000 | 15000 | Better coverage for global localization |
| TTC `ttc_timeout_sec` | `localization_metrics.yaml` | 10.0 | 30.0 | Enough time for convergence from global distribution |
| TTR `ttr_timeout_sec` | `localization_metrics.yaml` | 10.0 | 30.0 | Same |

### Files changed

- `bumperbot_description/urdf/bumperbot_gazebo.xacro`: LiDAR samples 360 → 720
- `bumperbot_localization/config/amcl.yaml`: max_beams, recovery_alpha_slow, z_rand, max_particles
- `navlearn_localization_eval/config/localization_metrics.yaml`: timeout values

### Important note

bumperbot_* packages are normally read-only. Exception granted for this localization research. Changes are limited to parameter values — no architectural changes to bumperbot packages.

---

## 6. Perturbation Level Presets

### TTC presets (false initial pose)

| Level | Position offset | Yaw offset | Launch mapping |
|-------|----------------|------------|----------------|
| easy | 0.25 m | 0.087 rad (5 deg) | `perturbation_level:=easy` |
| medium | 0.50 m | 0.262 rad (15 deg) | `perturbation_level:=medium` |
| hard | 1.0 m | 0.524 rad (30 deg) | `perturbation_level:=hard` |
| extreme | 2.0 m | 0.785 rad (45 deg) | `perturbation_level:=extreme` |

Mapped to episode_manager parameters (existing names — these are **half-ranges** for
`uniform_real_distribution(-val, +val)`):
- `bad_init_lin_range_m`: position perturbation half-range
- `bad_init_yaw_range_rad`: yaw perturbation half-range

### TTR presets (kidnap/teleport)

| Level | kidnap_distance_m | kidnap_max_distance_m |
|-------|-------------------|----------------------|
| easy | 0.3 | 0.5 |
| medium | 0.8 | 1.2 |
| hard | 1.5 | 2.5 |
| extreme | 2.5 | 4.0 |

### Implementation

In `benchmarks.launch.py`, map `perturbation_level` string to parameter values:

```python
perturbation_presets = {
    'easy':    {'bad_init_lin_range_m': 0.25, 'bad_init_yaw_range_rad': 0.087,
                'kidnap_distance_m': 0.3,  'kidnap_max_distance_m': 0.5},
    'medium':  {'bad_init_lin_range_m': 0.50, 'bad_init_yaw_range_rad': 0.262,
                'kidnap_distance_m': 0.8,  'kidnap_max_distance_m': 1.2},
    'hard':    {'bad_init_lin_range_m': 1.00, 'bad_init_yaw_range_rad': 0.524,
                'kidnap_distance_m': 1.5,  'kidnap_max_distance_m': 2.5},
    'extreme': {'bad_init_lin_range_m': 2.00, 'bad_init_yaw_range_rad': 0.785,
                'kidnap_distance_m': 2.5,  'kidnap_max_distance_m': 4.0},
}
```

---

## 7. Experiment Run Matrix

```
Phase 1 (current AMCL params — "before"):
  TTC: 4 levels x 5 episodes x 5 goals = 100 measurements
  TTR: 4 levels x 5 episodes x 5 goals = 100 measurements
  Profile: aggressive
  World: small_house
  Seed: 42

Phase 2 (tuned AMCL params — "after"):
  Same matrix = 200 measurements

Total: 400 TTC/TTR data points
```

### Run commands (Phase 1 example)

```bash
# TTC easy level
ros2 launch bumperbot_bringup simulated_robot.launch.py \
  world_name:=small_house nav2_profile:=aggressive

python3 src/navlearn_benchmarks/scripts/multi_run_harness.py \
  --episodes 5 --goals 5 --seed 42 \
  --output-dir benchmark_reports/runs/ttc_ttr/phase1/ttc_easy \
  --profile aggressive --goal-source map_random \
  --extra-arg bad_init_test:=true \
  --extra-arg perturbation_level:=easy \
  --extra-arg goal_min_distance_m:=4.0
```

Repeat for medium, hard, extreme. Then TTR with `kidnap_enabled:=true` instead of `bad_init_test:=true`.

### multi_run_harness.py change

Add repeatable `--extra-arg` flag (not `--extra-args` as a single string) to pass
additional launch arguments through to `benchmarks.launch.py`:

```python
parser.add_argument('--extra-arg', action='append', default=[],
                    help='Additional launch arg (repeatable), e.g. --extra-arg bad_init_test:=true')
```

Each `--extra-arg` value is appended as a separate element to the `cmd` list passed to
`subprocess.run()`. This avoids whitespace-splitting ambiguity:

```bash
# Usage:
python3 multi_run_harness.py \
  --episodes 5 --goals 5 --seed 42 \
  --output-dir benchmark_reports/runs/ttc_ttr/phase1/ttc_easy \
  --profile aggressive --goal-source map_random \
  --extra-arg bad_init_test:=true \
  --extra-arg perturbation_level:=easy \
  --extra-arg goal_min_distance_m:=4.0
```

---

## 8. Per-Goal Flow (Runtime Behavior)

```
Goal N starts:
├── [TTC mode] Inject false initial pose (half-range by perturbation level)
│   └── Publish to /navlearn/bad_initialpose_event
├── [TTR mode] Kidnap: teleport robot (distance by perturbation level)
│   ├── Call setEntityPose_()
│   └── Call callReinitGlobalLocalization()
├── Send Nav2 goal
├── localization_metrics measures convergence/recovery vs ground truth
├── OUTCOME A — Goal SUCCEEDED:
│   ├── TTC/TTR recorded (success/timeout/ended-early)
│   ├── idx_++ (advance to next goal)
│   └── Proceed to Goal N+1 → perturb/kidnap again
└── OUTCOME B — Goal FAILED/CANCELED:
    ├── TTC/TTR recorded (whatever state it was in)
    ├── ASYNC RECOVERY (state machine in timerCallback, NOT blocking onResult):
    │   ├── TELEPORT_PENDING: setEntityPose_(goal_pose), wait for service response
    │   ├── REINIT_PENDING: forceReinitGlobalLocalization() (bypass cooldown)
    │   ├── WAIT_CONVERGENCE: poll needCorrection() via timer (max 15s)
    │   └── DONE: idx_++, reset recovery state, log outcome
    └── Proceed to Goal N+1 → perturb/kidnap again (clean slate)
```

---

## 9. Analysis Outputs

### Scripts

New analysis script: `src/navlearn_benchmarks/scripts/analyze_ttc_ttr.py`

### Outputs

1. **Degradation curves** (matplotlib): Success rate (%) vs perturbation level
   - 4 curves on one plot: TTC-Phase1, TTC-Phase2, TTR-Phase1, TTR-Phase2
   - X-axis: Easy → Medium → Hard → Extreme
   - Y-axis: 0–100% success rate

2. **Recovery time box plots**: Per level, distribution of successful TTC/TTR times
   - Shows median, quartiles, outliers

3. **Before/after comparison table**: Markdown table with delta columns

4. **CSV aggregation**: All 400 data points in a single tidy CSV for further analysis

### Paper claim

> "NavLearn's TTC/TTR metrics reveal that AMCL with 2D LiDAR (720 samples) recovers reliably from perturbations up to X meters (Y% success rate) but degrades sharply beyond Z meters. Tuning recovery parameters extends reliable recovery from A to B meters, demonstrating the framework's value for systematic localization robustness evaluation."

---

## 10. Files Changed Summary

| Package | File | Change |
|---------|------|--------|
| `navlearn_benchmarks` | `episode_manager.cpp` | Add `goal_min_distance_m` filter, `RecoveryStage` state machine in `timerCallback()`, `forceReinitGlobalLocalization()`, `initiateRecoveryTeleport()`, `spawn_pose_` capture |
| `navlearn_benchmarks` | `episode_manager.hpp` | Declare `RecoveryStage` enum, `recovery_stage_`, `recovery_start_time_`, `recovery_timeout_sec_`, `spawn_pose_`, `goal_min_distance_m_` |
| `navlearn_benchmarks` | `episode_manager_defaults.yaml` | Add `goal_min_distance_m: 0.0` |
| `navlearn_benchmarks` | `benchmarks.launch.py` | Add localization_metrics + gz_set_pose_server nodes, perturbation presets, new launch args |
| `navlearn_benchmarks` | `multi_run_harness.py` | Add `--extra-args` flag |
| `navlearn_benchmarks` | `scripts/analyze_ttc_ttr.py` | New script for degradation curves and analysis |
| `navlearn_localization_eval` | `localization_metrics.yaml` | Make TTC error thresholds configurable (currently hardcoded) |
| `navlearn_localization_eval` | `localization_metrics.cpp` | Read TTC thresholds from params instead of hardcoded values |
| `bumperbot_localization` | `amcl.yaml` | Phase 2 tuning: max_beams, recovery_alpha_slow, z_rand, max_particles |
| `bumperbot_description` | `bumperbot_gazebo.xacro` | Phase 2: LiDAR samples 360 → 720 |

---

## 11. What Is NOT In Scope

- slam_toolbox localization mode comparison (future work, noted for Paper 1 "future directions")
- Baseline profile TTC/TTR (aggressive only to save runtime)
- Multi-world experiments (small_house only for now)
- Custom planner/controller benchmarking
- RL integration
