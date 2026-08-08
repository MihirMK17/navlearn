#!/usr/bin/env bash
# Leg 5: clean baselines — the reference row every perturbed table is read against.
#
#   THIS SCRIPT RUNS THE small_house PART ONLY: 3 controllers × 2 runs × 5 goals = 30.
#
#   The 3-map × RPP part (small_warehouse, bookstore) is DEFERRED, not forgotten: the
#   shipped maps were TurtleBot3-SLAM'd at a different lidar height and the rebuild plan
#   requires a re-SLAM before campaign use, and the bringup does not yet plumb map_name
#   through to the localization launch (global_localization.launch.py resolves maps from
#   bumperbot_mapping only). Running those cells now would collect data on maps already
#   flagged as not campaign-grade — the exact mistake the discarded leg 2 collections
#   taught. Blocked on: re-SLAM of both maps + map_name plumbing (bumperbot_* approval).
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
set -uo pipefail

cd "$HOME/robot_ws"
source src/navlearn/navlearn_analysis/scripts/cell_runner.sh

CAMPAIGN_DIR=results/leg5_baselines
mkdir -p "$CAMPAIGN_DIR"
exec > >(tee -a "$CAMPAIGN_DIR/campaign.log") 2>&1
ulimit -c unlimited 2>/dev/null || true

SEED=20260730
EPISODES=2
GOALS=5

say() { printf '\n===== %s  (%s) =====\n' "$*" "$(date '+%F %T')"; }

say "leg 5 clean baselines (small_house part) starting"
echo "workspace: $(git rev-parse --short HEAD)  seed: $SEED  2 runs x 5 goals per arm"

SMAC_PREFIX=$(python3 - <<'PY'
import os, pathlib
for prefix in [p for p in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if p]:
    if (pathlib.Path(prefix) / "share/ament_index/resource_index/packages/nav2_smac_planner").exists():
        print(prefix)
        break
PY
)
case "$SMAC_PREFIX" in
    "$HOME/robot_ws/install/"*) echo "OK - workspace planner overlay" ;;
    *) echo "FATAL: vendored planner not in front"; exit 1 ;;
esac
patch_hits=$(strings -a "$SMAC_PREFIX/lib/libnav2_smac_planner_2d.so" 2>/dev/null | grep -c "is outside the costmap bounds" || true)
[ "${patch_hits:-0}" -eq 0 ] && { echo "FATAL: planner patch missing"; exit 1; }
EM_LIB="$HOME/robot_ws/install/navlearn_benchmarks/lib/libepisode_manager.so"
corr_hits=$(strings -a "$EM_LIB" 2>/dev/null | grep -c "Goal-start correction applied" || true)
[ "${corr_hits:-0}" -eq 0 ] && { echo "FATAL: goal-start correction missing"; exit 1; }
echo "OK - planner patch + goal-start correction present"

overall=0
for arm in rpp dwb mppi; do
    say "arm: $arm (clean)"
    run_cell "$CAMPAIGN_DIR"_"$arm" \
        --episodes "$EPISODES" --goals "$GOALS" --seed "$SEED" \
        --perturbation clean \
        -- \
        controller:="$arm" planner:=smac2d localizer:=amcl_tuned
    rc=$?
    echo "### arm $arm finished with rc=$rc"
    [ $rc -ne 0 ] && overall=$rc
done

say "leg 5 campaign complete (small_house part)"
for arm in rpp dwb mppi; do
    d="$CAMPAIGN_DIR"_"$arm"
    rows=$(cat "$d"/navlearn_metrics_run_*[0-9].csv 2>/dev/null | grep -cv "^Goal_ID" || echo 0)
    printf '  %-5s %s goal rows\n' "$arm" "$rows"
done
echo "expected per arm: $((EPISODES * GOALS))"
echo "DEFERRED: 3-map x RPP cells — blocked on re-SLAM + map_name plumbing (see header)"
exit $overall
