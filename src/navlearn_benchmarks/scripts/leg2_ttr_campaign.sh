#!/usr/bin/env bash
# Leg 2 of the Paper 1 campaign: time-to-recovery under a continuous kidnap curve.
#
#   3 arms x 24 episodes x 5 goals, seed 20260730, kidnap magnitude log-uniform 0.01-3.0 m
#   drawn per goal from the campaign seed.
#
# This is the re-run. The 2026-08-01 attempt died at run 5 of 24 when planner_server
# segfaulted on an off-map pose estimate and the harness waited 8 h 14 m on a stack that no
# longer existed. Three things changed since:
#
#   1. nav2_smac_planner is vendored and patched (src/third_party) so an off-map pose is
#      refused rather than indexed on. Verified below before a single episode runs.
#   2. multi_run_harness carries a watchdog: no goal progress, or a dead nav2 process, ends
#      the cell with a named non-zero exit instead of waiting.
#   3. Every artefact of a run lands in the run's directory -- node logs, crash reports,
#      stack state at the abort, environment.
#
# Output goes to results/leg2_final_{rpp,dwb,mppi}. The crashed 2026-08-01 attempt stays
# where it is in results/leg2_ttr_rpp: run_cell wipes its output directory, and those four
# salvaged runs are the only record of the failure.
#
# Source ROS BEFORE strict mode: setup.bash references unset variables internally.
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
set -uo pipefail

cd "$HOME/robot_ws"
source src/navlearn_benchmarks/scripts/cell_runner.sh

CAMPAIGN_DIR=results/leg2_final
mkdir -p "$CAMPAIGN_DIR"
CAMPAIGN_LOG="$CAMPAIGN_DIR/campaign.log"

# The whole session's console output, not just the harness's. The 2026-08-01 post-mortem
# had no record of what the driving shell printed.
exec > >(tee -a "$CAMPAIGN_LOG") 2>&1

# Set here as well as in run_cell so the harness and every process it starts inherit it,
# and so the value printed below is the one that will actually apply.
ulimit -c unlimited 2>/dev/null || true

SEED=20260730
EPISODES=24
GOALS=5
CURVE=0.01:3.0

say() { printf '\n===== %s  (%s) =====\n' "$*" "$(date '+%F %T')"; }

say "leg 2 campaign starting"
echo "workspace   : $(git rev-parse --short HEAD)"
echo "seed        : $SEED   episodes: $EPISODES   goals: $GOALS   curve: $CURVE log"
echo "core limit  : $(ulimit -c)"

# --- Gate: the patched planner must actually be the one that loads -----------------
#
# planner_server is the stock binary from /opt/ros; it dlopens whichever
# libnav2_smac_planner.so the ament index resolves first. If the overlay is not sourced,
# or the workspace was rebuilt without --allow-overriding, the campaign would run on the
# unpatched planner and crash exactly as before -- while every record claimed otherwise.
say "verifying the patched planner is the one in front"
SMAC_PREFIX=$(python3 - <<'PY'
import os, pathlib
for prefix in [p for p in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if p]:
    if (pathlib.Path(prefix) / "share/ament_index/resource_index/packages/nav2_smac_planner").exists():
        print(prefix)
        break
PY
)
echo "nav2_smac_planner resolves from: ${SMAC_PREFIX:-<not found>}"
case "$SMAC_PREFIX" in
    "$HOME/robot_ws/install/"*) echo "OK - workspace overlay" ;;
    *) echo "FATAL: the vendored planner is not in front. Rebuild with:"
       echo "  colcon build --packages-select nav2_smac_planner --allow-overriding nav2_smac_planner"
       exit 1 ;;
esac
# The 2D planner is its own library. smac_plugin_2d.xml declares
# <library path="nav2_smac_planner_2d">, so libnav2_smac_planner.so is NOT the object
# pluginlib loads for SmacPlanner2D -- checking that one passes while the planner in use is
# whatever was there before.
SMAC_2D_LIB="$SMAC_PREFIX/lib/libnav2_smac_planner_2d.so"
# grep -c, not grep -q: under `set -o pipefail`, grep -q exits on the first match, strings
# takes SIGPIPE, and the pipeline reports 141 even though the string was found. The gate
# then fails on a correctly patched library.
patch_hits=$(strings -a "$SMAC_2D_LIB" 2>/dev/null | grep -c "is outside the costmap bounds" || true)
if [ "${patch_hits:-0}" -eq 0 ]; then
    echo "FATAL: $SMAC_2D_LIB does not carry the bounds check. Rebuild with:"
    echo "  colcon build --packages-select nav2_smac_planner --allow-overriding nav2_smac_planner"
    exit 1
fi
echo "OK - $SMAC_2D_LIB carries the patch"

# --- The three arms ---------------------------------------------------------------
#
# Same seed for every arm, so all three face identical goals, identical kidnap magnitudes
# and identical delays. That is what makes the paired statistics in the analysis plan valid.
overall=0
for arm in rpp dwb mppi; do
    say "arm: $arm"
    run_cell "$CAMPAIGN_DIR"_"$arm" \
        --episodes "$EPISODES" --goals "$GOALS" --seed "$SEED" \
        --perturbation ttr --magnitude-curve "$CURVE" --magnitude-scale log \
        -- \
        controller:="$arm" planner:=smac2d localizer:=amcl_tuned
    rc=$?
    echo "### arm $arm finished with rc=$rc"
    [ $rc -ne 0 ] && overall=$rc
    # A failed arm does not stop the campaign: the other two are independent cells and
    # their machine time is not recoverable. The non-zero exit at the end is what says a
    # cell needs re-running.
done

say "leg 2 campaign complete"
for arm in rpp dwb mppi; do
    d="$CAMPAIGN_DIR"_"$arm"
    rows=$(cat "$d"/navlearn_metrics_run_*[0-9].csv 2>/dev/null | grep -cv "^Goal_ID" || echo 0)
    crashes=$(ls "$d"/forensics_run_*/*.crash 2>/dev/null | wc -l)
    printf '  %-5s %s goal rows, %s crash report(s)\n' "$arm" "$rows" "$crashes"
done
echo "expected per arm: $((EPISODES * GOALS)) goal rows"
exit $overall
