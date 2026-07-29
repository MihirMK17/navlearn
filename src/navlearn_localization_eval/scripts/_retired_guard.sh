#!/usr/bin/env bash
# Shared retirement guard for the pre-2026-07 campaign orchestrators.
#
# Every script that sources this drives the navigation stack through `nav2_profile:=`,
# an argument removed on 2026-07-28 when config/nav2_stack/ replaced config/nav2_profiles/.
# The profile directories still exist on disk so archived results keep their provenance,
# but the launch files no longer accept a profile name, so these scripts fail at sim launch.
#
# They are also superseded in substance. The profiles they select carry the parameter drift
# the 2026-07-25 audit found — xy_goal_tolerance 0.05/0.08/0.10 across arms,
# movement_time_allowance 6.0 against 60.0, bt_loop_duration 20 against 10, and an MPPI
# ObstaclesCritic at cost_scaling_factor 10.0 against a costmap of 3.0 — and the data they
# produced is retired from Paper 1 per rebuild plan decision D-1.
#
# Kept rather than deleted because they are the record of how the archived data was made.
#
# Usage: source this immediately after the shebang, before any other work.
#     source "$(dirname "${BASH_SOURCE[0]}")/_retired_guard.sh"
#
# Set NAVLEARN_RUN_RETIRED=1 to bypass. The script will still fail at sim launch; the
# escape hatch exists only for reading control flow under `bash -x`.

if [[ "${NAVLEARN_RUN_RETIRED:-0}" != "1" ]]; then
    cat >&2 <<RETIRED
$(basename "${BASH_SOURCE[1]:-this script}") is RETIRED and cannot run.

It selects the Nav2 stack with \`nav2_profile:=\`, which no longer exists — the profile
directories were replaced by composed fragments in config/nav2_stack/, and the profiles it
targets carry the parameter drift documented in docs/o1/experiment_audit.md.

Use instead:
  ros2 launch bumperbot_bringup simulated_robot.launch.py \\
      world_name:=small_house controller:=rpp planner:=smac2d localizer:=amcl_tuned
  python3 src/navlearn_benchmarks/scripts/multi_run_harness.py --output-dir <dir>

The harness now reads the stack from the provenance record the bringup writes, so a run
can no longer be labelled with a configuration it did not use.

See docs/paper1/EXECUTION_SPEC.md for the campaign procedure.
RETIRED
    exit 2
fi
