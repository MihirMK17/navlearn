# navlearn_benchmarks – Performance Plan

## 1. Nodes and Expected Load

### EpisodeManager
- Input:
  - Action server: /navigate_to_pose (Nav2)
- Output:
  - /navlearn/episode_event (START/END per goal)
- Expected rate:
  - Very low (~0.1–0.5 Hz: 1 goal every few seconds).
- Work per event:
  - Build EpisodeEvent messages, timestamping, some TF lookup.
- Potentially heavy:
  - TF lookups on every goal.
  - Logging if we make logs too chatty.

### ControlMetric
- Inputs:
  - /cmd_vel (Twist)
  - /joint_states (sensor_msgs/JointState)
  - /bumperbot_controller/odom (nav_msgs/Odometry)
  - /navlearn/episode_event
- Expected rates:
  - /cmd_vel: ~10–20 Hz during motion.
  - /joint_states: ~50–100 Hz typically.
  - /odom: ~50 Hz.
  - Timer: 100 Hz (your 10ms timer).
- Work per tick:
  - Align time-series buffers, compute tracking errors, slip, control_energy accumulation.
- Potentially heavy:
  - The matching/interpolation in `sample_interp_or_hold`.
  - Doing all this at 100 Hz if CPU gets tight.

### TrajectoryMetric
- Inputs:
  - /bumperbot_controller/odom
  - /navlearn/episode_event
- Expected rates:
  - /odom: ~50 Hz.
- Work:
  - Aggregate path length over the episode.
- Potentially heavy:
  - Per-sample distance computation (but realistically cheap).

### MetricsCompiler
- Inputs:
  - /navlearn/episode_event (END)
  - /navlearn/control_metric
  - /navlearn/trajectory_metric
- Expected rates:
  - One write per episode.
- Work:
  - Aggregate episode fields + write one CSV line + (optionally) JSON summary.
- Potentially heavy:
  - File I/O if we write a lot or flush too often.

## 2. Baseline Scenarios (to fill after runs)

### Scenario A – Small map, few episodes
- Map: small_house
- Episode config: square path, 4 goals, 1 episode
- Observations:
  - Total runtime: 36.04 seconds
  - CPU: cores ~20-25% util, load avg ~2 on 12 cores --> not CPU-bound
  - Memory: ~8 GB / 15.4 GB --> no memory pressure
  - nav_time_mean: 9.01 seconds
  - path_length_mean: 0.155107 meters
  - control_energy_mean: 1678.83
  - Notes (warnings, weird behavior): None

### Scenario B – Stress config
- Map: small_house
- Episode config: 20 episodes, same goals, minimal dwell
- Observations:
  - Total runtime: 435.04 seconds
  - CPU: cores ~25-35% util average with ~40-45% maximum, load avg ~4.5 to 5.5 avg, with 6.3 peak --> not CPU bound
  - Memory: ~ 7.67 GB /15.4 GB --> no memory pressure
  - nav_time_mean: 31.0743 seconds
  - path_length_mean: 2.769 meters
  - control_energy_mean: 5558.43
  - Notes (warnings, weird behavior): 4 goals failed out of 14. This points to the amcl re-tuning requirements and not benchmarking stack failure

