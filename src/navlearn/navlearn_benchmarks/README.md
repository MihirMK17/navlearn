# navlearn_benchmarks

The runtime instrumentation of NavLearn: every C++ node that observes, perturbs, or
scores a navigation episode. Launched per-episode by `benchmarks.launch.py`, driven by
the harness in `navlearn_analysis`.

## Nodes

| executable | role | key topics |
|---|---|---|
| `episode_manager_node` | Goal sequencer + perturbation injector: sends `NavigateToPose` goals, samples goals from `/map`, publishes bad-init `initialpose`, teleports the robot mid-goal (kidnap) via the `SetEntityPose` service, clears costmaps between goals | pub `/navlearn/episode_event`, `/navlearn/kidnap_event` |
| `control_metric` | Per-episode `cmd_vel` tracking RMS, saturation fraction, wheel-slip stats, control energy | sub `/bumperbot_controller/cmd_vel` → pub `/navlearn/control_metric` |
| `trajectory_metric` | Path length, min laser clearance, collision rising edges, ATE/RPE against ground truth | sub `/scan`, `/bumperbot/ground_truth_pose` → pub `/navlearn/trajectory_metric` |
| `metrics_compiler` | Joins episode/control/trajectory/kidnap streams by goal UUID into the one-row-per-goal metrics CSV | sub the three above + `/navlearn/kidnap_event` |
| `localization_metrics` | Online TTC/TTR/ATE/RPE state machines against ground truth; writes the long-format localization CSV | sub `/amcl_pose`, `/bumperbot/ground_truth_pose`, `/navlearn/episode_event`, `/navlearn/kidnap_event` |
| `ground_truth_publisher` | Republishes the Ignition ground-truth stream (`/tf_gt`) as `/bumperbot/ground_truth_pose` + the `base_footprint_gt` frame | |
| `gz_set_pose_server` | Bridges the `SetEntityPose` ROS service onto Ignition Transport for the kidnap teleport | |
| `scan_rate_governor` | Decimates `/scan_unfiltered` to a commanded rate for the sensor-starvation leg; forwards scans bit-identically | composed by `bumperbot_bringup` when `scan_rate_hz > 0` |
| `collision_monitor` | Ignition contact stream → collision events | |

Six header-only algorithm units (`bad_init_offset`, `exclusion_zone`, `kidnap_yaw`,
`magnitude_sampler`, `navlearn_seed`, `scan_decimator`) each carry a dedicated gtest.

## The frozen runtime contract

The collected campaign dataset (~2,070 goals of rosbags and CSVs) bakes in these names.
They do not change:

- topics: `/amcl_pose`, `/bumperbot/ground_truth_pose`, `/navlearn/kidnap_event`,
  `/navlearn/episode_event`, `/scan`, `/bumperbot_controller/cmd_vel`
- message types: `navlearn_msgs/*`
- executable names above (campaign teardown kills by process name)

## Testing

`colcon test` runs the 6 gtests plus the three pytest suites that need built nodes
(`test_terminal_pose`, `test_kidnap_reference`, `test_scan_rate_governor`). The
pure-Python suites live in `navlearn_analysis`.

Known gap, on purpose rather than by accident: `localization_metrics` (940 lines,
computes the online TTC/TTR) has no unit tests of its own. Its per-sample rule is
cross-checked offline by `navlearn_analysis`' `recompute_ttr_from_bags`, whose test
suite pins the shared definition (0.20 m / 0.10 rad, 2.0 s hold).
