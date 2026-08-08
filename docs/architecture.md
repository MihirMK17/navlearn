# NavLearn Architecture

How a benchmark run actually flows, from the operator's shell to the per-goal CSV row.

## The full pipeline

```mermaid
flowchart TD
    subgraph Operator["Campaign layer (navlearn_analysis)"]
        CR["cell_runner.sh / leg drivers\npreflight → bringup → harness → verified teardown"]
        H["multi_run_harness\nepisodes, seeds, perturbation curves\nspawns probes + rosbag per run"]
        PR["probes: rate_monitor · compute_sampler\ncostmap_corruption_monitor"]
    end

    subgraph Bringup["bumperbot_bringup / simulated_robot.launch.py"]
        L["composition root\nworld, map, controller/planner/localizer,\nheadless, gpu, scan_rate_hz"]
        SPEC["stack_spec →\n~/.navlearn/current_stack_spec.json\n(provenance the harness requires)"]
    end

    subgraph Sim["Simulation (Gazebo Fortress, dGPU via PRIME)"]
        G["world from description or\nnavlearn_assets (GZ_SIM_RESOURCE_PATH)\nBumperBot + RPLidar A1"]
        SC["/scan_unfiltered → [governor]* → sanitizer → /scan"]
        GT["/tf_gt → ground_truth_publisher →\n/bumperbot/ground_truth_pose"]
    end

    subgraph Nav["Nav2 (fragments from bumperbot_navigation)"]
        AMCL["AMCL /amcl_pose"]
        PL["SMAC 2D (vendored, patched)"]
        CT["RPP | DWB | MPPI"]
    end

    subgraph Pipeline["Metric pipeline (navlearn_benchmarks)"]
        EM["episode_manager\nseeded goals · bad-init · kidnap teleport\n/navlearn/episode_event · /navlearn/kidnap_event"]
        CM["control_metric"]
        TM["trajectory_metric"]
        LM["localization_metrics\nonline TTC/TTR"]
        MC["metrics_compiler\njoin by goal UUID"]
        GZS["gz_set_pose_server\nSetEntityPose ⇄ Ignition"]
    end

    subgraph Output["Per-run artifacts"]
        CSV["metrics CSV (row per goal)\n+ localization CSV (long format)"]
        BAG["rosbag (frozen topic set)"]
        RJ["rates/compute/costmap JSONs\n+ stack spec + environment.json"]
    end

    CR --> L
    CR --> H
    H --> PR
    H -->|ros2 launch benchmarks.launch.py| EM
    L --> SPEC --> H
    G --> SC -->|/scan| AMCL
    SC -->|/scan| TM
    GT --> TM
    GT --> LM
    AMCL --> LM
    EM <-->|NavigateToPose| Nav
    EM -->|SetEntityPose| GZS --> G
    EM --> CM & TM & MC & LM
    CM --> MC
    TM --> MC
    MC --> CSV
    LM --> CSV
    H --> BAG
    PR --> RJ
```

\* the scan-rate governor is composed only for sensor-starvation cells.

## The frozen runtime contract

Recorded rosbags and CSVs (~2,070 goals) serialize these names; they are stable across
refactors by policy:

| kind | names |
|---|---|
| topics | `/amcl_pose` · `/bumperbot/ground_truth_pose` · `/navlearn/kidnap_event` · `/navlearn/episode_event` · `/scan` · `/bumperbot_controller/cmd_vel` |
| interfaces | `navlearn_msgs/*` (see that package's README) |
| executables | the pipeline nodes above (teardown kills by process name) |

## Key design decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Config switching | composable YAML fragments, no recompile | parity lives in one `common/` file; arms cannot drift |
| Provenance | stack-spec JSON with live-writer check, git SHA per run | "which configuration produced this number" is always answerable |
| Goal + perturbation draws | splitmix64 from (campaign, run, goal) | unique per goal, identical across controller arms — the precondition for paired statistics |
| Ground truth | Ignition transport (not ros-gz-bridge) | lower latency; also carries the kidnap teleport |
| Perturbation magnitudes | continuous curves, not levels | the sweep is the experiment; no discarded calibration |
| Sensor starvation | decimate the published stream | one variable moves; the simulated sensor stays identical |
| Recovery definition | 0.20 m AND 0.10 rad, 2 s hold | position alone is exactly what the confident-wrong failure fools |
| Analysis | committed, tested package (`navlearn_analysis`) | an analysis that cannot be re-run cannot be defended |
