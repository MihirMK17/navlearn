# NavLearn: A Unified Benchmarking Framework for Mobile Robot Navigation Stacks

*Draft — Paper 1. Target: IEEE RA-L. 6-page limit.*
*Status: Architecture + Related Work sections drafted. Results pending MK2 runs.*

---

## I. Introduction

*(~0.5 page — draft after results are final)*

Mobile robot navigation in real and simulated environments depends on a complex interplay of
localization, planning, and control algorithms. Despite substantial advances in individual
components—AMCL, Nav2 planners, model-predictive controllers—systematic comparison across
algorithm configurations remains ad hoc. Practitioners often report results on in-house
environments under conditions that cannot be reproduced by others.

We present **NavLearn**, an open-source benchmarking framework for ROS 2 / Nav2 that addresses
this gap. NavLearn executes multi-run evaluations across algorithm configurations and measures
navigation efficiency, control quality, localization robustness, and safety in a single
reproducible pipeline. Configuration switching requires only changing a YAML file—no
recompilation, no environment changes.

Our contributions are:
1. A unified metric pipeline integrating localization robustness (TTC, TTR, ATE, RPE) with
   navigation efficiency (path length, nav time, SPL) and safety (collision count, clearance).
2. A config-driven benchmark protocol enabling rigorous Nav2 algorithm profile comparison.
3. Time-to-Converge (TTC) and Time-to-Recover (TTR) as first-class localization metrics
   alongside standard navigation benchmarking.
4. An open-source, seeded, reproducible benchmark against which future Nav2 configurations
   can be compared.

*(Paper organization: Section II reviews related work. Section III describes NavLearn's
architecture. Section IV defines metrics. Section V presents experimental setup. Section VI
reports results. Section VII concludes.)*

---

## II. Related Work

*(~1 page)*

**Navigation benchmarks.** The BARN dataset [Perille et al., ICRA 2022] provides a suite of
obstacle-dense environments with standardized difficulty levels, enabling reproducible
comparison of collision avoidance performance. While BARN excels at stress-testing local
planners, it does not measure localization quality or control efficiency. Arena-Bench [Kastner
et al., RA-L 2022] provides a multi-robot benchmark with dynamic obstacles and social force
scenarios, but targets multi-agent settings and does not quantify localization convergence or
control energy. Neither framework provides config-driven comparison of Nav2 algorithm profiles
without code changes.

**Trajectory evaluation.** EVO [Grupp, 2017] and rpg_trajectory_evaluation [Zhang and Scaramuzza,
2018] provide rigorous ATE and RPE computation for SLAM and VIO trajectories. These tools
operate on recorded trajectory files and produce localization accuracy reports—but they do not
integrate with navigation success metrics, control quality, or safety measurements. NavLearn
incorporates ATE and RPE directly in the benchmarking pipeline alongside per-goal navigation
and control data.

**Nav2 evaluation.** Several works evaluate specific Nav2 planners or controllers in isolation
(e.g., SMAC Planner [Macenski et al., 2021], Regulated Pure Pursuit [Macenski et al., 2023])
but use custom evaluation protocols that are not reproducible without the original author's
environment. No prior work provides a unified, config-driven evaluation harness for the full
Nav2 stack (localization + planning + control) under a single reproducible protocol.

**Convergence metrics.** Localization convergence time has been studied in the context of
particle filter initialization [Fox et al., 1999] and kidnap recovery [Thrun et al., 2001],
but these metrics have not been integrated into standard navigation benchmarks. NavLearn
introduces TTC and TTR as first-class metrics in a navigation evaluation pipeline.

**Summary.** NavLearn fills the gap between existing navigation benchmarks (which lack
localization depth) and SLAM evaluation tools (which lack navigation success metrics) by
unifying both in a single open-source pipeline.

---

## III. System Architecture

*(~1.5 pages — this section is nearly complete)*

### 3.1 Design Philosophy

NavLearn is built around three principles: **config-first**, **reproducibility**, and
**typed data contracts**.

*Config-first:* Algorithm profiles—AMCL parameters, planner settings, controller gains—are
expressed as YAML parameter files. Switching from a "baseline" to an "aggressive" Nav2 profile
requires changing a single launch argument (`nav2_profile:=aggressive`). No recompilation is
required, and all configuration is version-controlled.

*Reproducibility:* Each benchmark run uses a seeded random goal generator. The same seed
produces identical goal sequences across machines and runs, enabling statistical comparison
between runs.

*Typed data contracts:* All inter-node communication uses typed ROS 2 messages defined in the
`navlearn_msgs` package. This decoupling allows metric nodes to be independently upgraded or
replaced without modifying the rest of the pipeline.

### 3.2 Four-Node Metric Pipeline

The benchmarking pipeline consists of four C++ nodes:

**EpisodeManager** manages the benchmarking protocol. It generates seeded random goals from
a 2D occupancy grid, dispatches them to the Nav2 `navigate_to_pose` action server, and
publishes `EpisodeEvent` messages (START and END) that synchronize all metric nodes. It
supports dwell periods between goals, bad-initialization testing, and teleportation-based
kidnap scenarios. At goal dispatch, it computes and attaches the Euclidean start-to-goal
distance (`optimal_path_m`) for SPL computation downstream.

**ControlMetric** subscribes to velocity commands and odometry to compute per-episode control
quality: RMS tracking error in linear and angular velocity, saturation fraction, wheel slip
statistics (mean, std, 95th percentile), and control energy (weighted L2 norm of velocity
commands integrated over the episode).

**TrajectoryMetric** subscribes to odometry to integrate path length and to a ground truth
pose topic to compute localization accuracy. It computes ATE (RMSE of position errors after
nearest-timestamp matching) and RPE (relative pose errors over fixed time intervals). It also
tracks minimum laser scan range across the episode (`min_clearance_m`) as a safety proxy.

**MetricsCompiler** receives END-of-episode events from all three other nodes and—once all
three metrics are available for a given episode—writes a row to a CSV file and updates a
per-run JSON summary. It computes SPL (Success weighted by inverse Path Length) per episode.

### 3.3 Localization Evaluation Module

A separate `navlearn_localization_eval` package provides the `LocalizationMetrics` node for
detailed per-episode localization analysis. It subscribes to AMCL pose, the `/tf` tree, ground
truth, and scan topics to compute:

- **TTC (Time-to-Converge):** seconds from bad-initialization event to AMCL convergence
  (defined as position error < threshold for a hold duration).
- **TTR (Time-to-Recover):** seconds from kidnap teleportation event to localization recovery.
- **Input validity metrics:** message rate, monotonicity violations, and frame mismatches for
  scan, odometry, and TF inputs.

TTC and TTR are implemented as enum-class state machines (`TtcState`, `TtrState`) with logged
transitions, improving debuggability over the scattered boolean approach common in research code.

### 3.4 Benchmarking Harness

The Python `multi_run_harness.py` script automates N sequential runs, each invoked as a
separate `ros2 launch` process. Each run receives a unique seed derived from a base seed plus
the run index. The script fails fast on non-zero exit codes, preventing silent failures from
corrupting a run set.

The `aggregate_runs.py` script reads per-run JSON summaries and computes full statistics
(mean, std, min, max, percentiles p25/p50/p75/p95) for each metric, detects outlier runs
(>2σ from mean), and computes SPL where available.

### 3.5 navlearn_msgs Schema

```
EpisodeEvent: goal_id, state, result, nav_time, start_pose, goal_pose,
              recoveries, collision_count, optimal_path_m
ControlMetric: goal_id, tracking_rms_v, tracking_rms_w, saturation_frac_v/w,
               slip_mean/std/p95, control_energy, samples
TrajectoryMetric: goal_id, path_length_m, duration, samples,
                  ate_rmse_m, rpe_trans_rmse_m, ref_source,
                  min_clearance_m
```

---

## IV. Metrics Definition

*(~0.75 page — draft after confirming all metrics produce non-NaN results)*

### Navigation Efficiency
- **Success rate**: fraction of goals reached within timeout.
- **Nav time** [s]: wall-clock time from goal dispatch to termination.
- **Path length** [m]: odometry-integrated distance traveled.
- **SPL**: Success weighted by inverse Path Length = success × (L* / max(L, L*)), where L* is Euclidean start-to-goal distance and L is actual path length. Penalizes circuitous paths even on successful episodes [Anderson et al., 2018].

### Control Quality
- **Tracking RMS** [m/s, rad/s]: RMS velocity tracking error (commanded vs. measured).
- **Saturation fraction**: fraction of time commands are at ≥98% of v_max or w_max.
- **Control energy**: ∫(λv² + (1-λ)w²)dt, a scalar efficiency proxy.
- **Slip statistics**: wheel slip distribution (mean, std, 95th percentile).

### Localization Accuracy
- **ATE RMSE** [m]: Root mean square of absolute position errors after nearest-timestamp GT matching.
- **RPE RMSE** [m]: Root mean square of relative pose errors over 1-second intervals.
- **TTC** [s]: Time-to-Converge from bad initialization.
- **TTR** [s]: Time-to-Recover from kidnap teleportation.

### Safety
- **Min clearance** [m]: minimum laser scan range observed during episode (proxy for obstacle proximity).
- **Collision count**: number of contact events with obstacles (Gazebo contacts).

---

## V. Experimental Setup

*(~0.5 page — finalize after runs)*

**Robot**: Bumperbot, a differential-drive platform with RPLidar A1 (2D, 360°, 12m range),
MPU-6050 IMU, and wheel encoders. Simulated in Gazebo Fortress with gz_ros2_control.

**World**: `small_house` — a 7×7m indoor environment with walls, furniture, and narrow doorways.

**Localization**: AMCL with EKF odometry fusion (via `robot_localization`).

**Nav2 profiles**:
- *Baseline*: conservative velocity limits (v_max=0.15 m/s), wide inflation radius.
- *Aggressive*: higher velocity limits (v_max=0.25 m/s), tighter inflation, larger look-ahead.

*(Exact parameters in `src/navlearn_benchmarks/config/` in the open-source repository.)*

**Protocol**: 10 independent runs × 5 seeded random goals per profile. All runs use base seed
42; each run uses seed 42+i for i ∈ {0,...,9}. Results reported as mean ± std with p50 and
p95 percentiles.

---

## VI. Results

*(Populate after MK2 final benchmark run — see mk2_preliminary_results.md)*

---

## VII. Conclusion

*(~0.25 page — draft last)*

NavLearn provides the robotics community with a reproducible, config-driven benchmarking
framework for Nav2 navigation stacks. By unifying localization robustness (TTC, TTR, ATE, RPE),
navigation efficiency (path length, nav time, SPL), and safety (collision count, clearance)
in a single open-source pipeline, it enables systematic comparison of algorithm configurations
without bespoke evaluation code.

**Limitations**: Current evaluation is limited to a single 2D LiDAR sensor and a single world.
Dynamic obstacles are not modeled. Computational profiling (CPU usage, latency) is not yet
implemented.

**Future work**: Phase 2 will introduce an RL-trained policy as a third profile for comparison.
Multi-world evaluation across `small_house`, `small_warehouse`, and custom environments is
planned. Real-robot validation on the physical Bumperbot platform is a longer-term goal.

---

*End of current draft. Sections VI and VII to be completed after benchmark runs.*
