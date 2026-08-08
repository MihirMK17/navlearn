# navlearn_analysis

NavLearn's Python side as one installed package: the campaign harness, its per-run
probes, the offline claim analyses, and the map/world tooling. Everything is a real
module with real imports; every CLI is a console entry point:

```bash
ros2 run navlearn_analysis <name> [args...]
```

## Module map

| area | modules |
|---|---|
| `harness/` | `multi_run_harness` (the campaign hub: spawns the per-episode launch, supervises it with `run_watchdog`, records forensics + rosbags via `run_forensics`) |
| `probes/` | per-run instrumentation the harness spawns: `rate_monitor` (delivered `/scan` + `cmd_vel` rates, the `scan_rate_met` gate), `compute_sampler`, `costmap_corruption_monitor`, `collision_positive_control` |
| `claims/` | the committed, tested methods behind the paper's numbers: `analyze_curve_campaign`, `recompute_ttr_from_bags`, `run_yaw_claim`, `compare_yaw_slopes`, `run_claim2_models` |
| `stack/` | `validate_nav2_stack` — the nav2 configuration parity gate |
| top level | `nested_models` (hand-rolled logistic fits, closed-form tested), `rate_mechanism`, `mechanism_by_bin`, `sigma_curves`, `world_to_map` (collision-geometry → occupancy map), `map_ambiguity`, `map_spread_gate`, `kidnap_feasibility`, `navlearn_seed` (splitmix64 reference, golden-pinned against the C++) |
| `figures/` | generators for every image in the repo README — each figure names its command |
| `scripts/` | operator shell: `cell_runner.sh` (bringup + harness + verified teardown for one cell), the `leg*_campaign.sh` drivers, `campaign_watchdog.sh` (independent, detached run watchdog) |

## Running a campaign cell

```bash
cd <workspace>
source install/setup.bash
source src/navlearn/navlearn_analysis/scripts/cell_runner.sh
NAVLEARN_WORLD=small_house run_cell results/my_cell \
    --episodes 5 --goals 5 --seed 42 --perturbation ttr \
    -- controller:=rpp planner:=smac2d localizer:=amcl_tuned
```

`run_cell` refuses to launch into a dirty environment, waits for both lifecycle
managers, runs the harness, and tears down by process name with confirmation — the
discipline that keeps unattended overnight campaigns from silently corrupting each
other.

## Testing

`colcon test` discovers `test/` wholesale — 248 cases over the analyses, the watchdog,
forensics, seeds, maps and models. That discovery-by-construction is the point: five of
these test files once sat unregistered in an ament_cmake package and never ran in CI.
A test file here cannot be dark.

Reruns of the claim analyses on the banked campaign CSVs are byte-identical; that
property is checked whenever this package is restructured.
