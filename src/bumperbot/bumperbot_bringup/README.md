# bumperbot_bringup

The composition root. Everything the robot runs — simulated or real — starts from one
of two launch files here; no other package is launched directly.

## Entry points

```bash
# Simulation (the NavLearn campaign path)
ros2 launch bumperbot_bringup simulated_robot.launch.py \
    world_name:=small_house controller:=rpp planner:=smac2d localizer:=amcl_tuned \
    headless:=true use_rviz:=false

# Hardware
ros2 launch bumperbot_bringup real_robot.launch.py
```

## simulated_robot.launch.py arguments

| arg | default | meaning |
|---|---|---|
| `world_name` | `small_house` | world to simulate; resolved by `gazebo.launch.py`'s search (description worlds, then `GZ_SIM_RESOURCE_PATH` packages such as `navlearn_assets`) |
| `map_yaml` | small_house map | what AMCL localizes against — deliberately separate from `world_name`; a mismatch should be explicit, never invented by a naming convention |
| `controller` / `planner` / `localizer` / `ablation` | `rpp` / `smac2d` / `amcl_tuned` / `none` | nav2 stack composition, resolved through `bumperbot_navigation`'s fragment system |
| `headless` | `false` | Gazebo server-only; campaigns use `true` |
| `use_rviz` | `true` | RViz is a second renderer competing with the stack under measurement; campaigns use `false` |
| `gpu` | `true` | PRIME render offload to the discrete GPU; `false` reproduces the Intel-iGPU backend pre-2026-08-06 campaigns used |
| `scan_rate_hz` | `0.0` | sensor-starvation leg: composes the `scan_rate_governor` (from `navlearn_benchmarks`) when above zero |
| `stack_spec_out` | `~/.navlearn/current_stack_spec.json` | provenance record of the composed stack; the benchmark harness refuses to run without a live one |

The stack-spec handoff is the contract between bringup and harness: the navigation
launch writes what was actually composed, the harness records it with every run.
