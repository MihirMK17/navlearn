# NavLearn Architecture Diagram

## Full Pipeline (Mermaid)

```mermaid
flowchart TD
    subgraph Harness["Benchmarking Harness (Python)"]
        H["multi_run_harness.py\n--profile baseline|aggressive\n--episodes N --seed S"]
    end

    subgraph Launch["benchmarks.launch.py"]
        L["ROS 2 Launch\nworld_name, nav2_profile\ncsvpath, jsonpath"]
    end

    subgraph Sim["Simulation (Gazebo Fortress)"]
        G["small_house world\nBumperbot robot\nRPLidar A1 /scan"]
        GT["/bumperbot/ground_truth\nGeometry TransformStamped"]
    end

    subgraph Nav["Navigation Stack (Nav2)"]
        AMCL["AMCL\n/amcl_pose"]
        SMAC["SMAC Planner 2D\nglobal planner"]
        RPP["RegulatedPurePursuit\ncontroller"]
    end

    subgraph Pipeline["NavLearn 4-Node Metric Pipeline"]
        EM["episode_manager\n/navlearn/episode_event\nSeeded goals, dwell, kidnap"]
        CM["control_metric\n/navlearn/control_metric\nRMS tracking, energy, slip"]
        TM["trajectory_metric\n/navlearn/trajectory_metric\npath_length, ATE, RPE,\nmin_clearance"]
        MC["metrics_compiler\n→ CSV + JSON output\nSPL, per-episode rows"]
    end

    subgraph LocEval["Localization Eval (Optional)"]
        LM["localization_metrics\n→ localization_eval CSV\nTTC, TTR, AMCL quality"]
    end

    subgraph Output["Output"]
        CSV["navlearn_metrics.csv\nper-goal rows"]
        JSON["navlearn_run_report.json\nrun summary + SPL mean"]
        LCSV["localization_evaluation.csv\nTTC, TTR, ATE, RPE"]
        AGG["aggregate_runs.py\nmean ± std, p50, p95\noutlier detection"]
    end

    H --> Launch
    Launch --> EM
    G -->|/scan| TM
    G -->|/bumperbot_controller/odom| CM
    G -->|/bumperbot_controller/odom| TM
    GT --> TM
    GT --> LM
    AMCL --> LM
    EM -->|EpisodeEvent START/END| CM
    EM -->|EpisodeEvent START/END| TM
    EM -->|EpisodeEvent START/END| MC
    EM -->|EpisodeEvent START/END| LM
    CM -->|ControlMetric| MC
    TM -->|TrajectoryMetric| MC
    MC --> CSV
    MC --> JSON
    LM --> LCSV
    CSV --> AGG
    JSON --> AGG
```

## navlearn_msgs Data Contract

```
EpisodeEvent (per-goal):
  goal_id, state (START|END), result, nav_time
  start_pose, goal_pose
  recoveries, collision_count, optimal_path_m

ControlMetric (per-goal):
  goal_id, tracking_rms_v, tracking_rms_w
  saturation_frac_v, saturation_frac_w
  slip_mean, slip_std, slip_p95
  control_energy, samples

TrajectoryMetric (per-goal):
  goal_id, path_length_m, duration, samples
  ate_rmse_m, ate_count, rpe_trans_rmse_m, rpe_count
  ref_source, ref_frame
  min_clearance_m
```

## Key Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Config switching | YAML profiles, no recompile | Reproducible A/B comparison |
| Goal generation | Seeded random (map_random) | Identical conditions across runs |
| Output format | CSV (per-goal) + JSON (per-run) | CSV for analysis, JSON for CI checks |
| Ground truth | Ignition transport (not ros-gz-bridge) | Lower latency, Gazebo-native |
| ATE/RPE | Nearest-timestamp matching | Handles async GT + odom rates |
| SPL | optimal_path / max(actual, optimal) | Standard embodied AI metric |
