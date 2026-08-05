#!/usr/bin/env bash
# The yaw-curve leg (PROTOCOL amendment A3, 2026-08-03): recovery under a pure heading
# rotation, displacement pinned to zero.
#
#   3 arms x 24 episodes x 5 goals, seed 20260730, |yaw change| linear-uniform 0-180 deg
#   drawn per goal from KIDNAP_YAW_MAGNITUDE, sign from KIDNAP_YAW_SIGN, teleport in place.
#
# Motivation is the discarded second leg-2 collection: an uncontrolled uniform rotation
# produced 100% recovery at 0-30 deg against 26.8% at 90-180 deg while displacement
# explained nothing. That data was discarded as unattributable; this leg measures the same
# effect with the rotation as the DECLARED variable and nothing else moving.
#
# Same seed as leg 2, so the three arms face identical goals, delays and yaw draws, and
# the yaw curve is paired with the distance curve goal-for-goal across legs.
#
# Source ROS BEFORE strict mode: setup.bash references unset variables internally.
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
set -uo pipefail

cd "$HOME/robot_ws"
source src/navlearn_benchmarks/scripts/cell_runner.sh

CAMPAIGN_DIR=results/leg_yaw
mkdir -p "$CAMPAIGN_DIR"
CAMPAIGN_LOG="$CAMPAIGN_DIR/campaign.log"
exec > >(tee -a "$CAMPAIGN_LOG") 2>&1

ulimit -c unlimited 2>/dev/null || true

SEED=20260730
EPISODES=24
GOALS=5
YAW_CURVE=0:180

say() { printf '\n===== %s  (%s) =====\n' "$*" "$(date '+%F %T')"; }

say "yaw-curve leg starting"
echo "workspace   : $(git rev-parse --short HEAD)"
echo "seed        : $SEED   episodes: $EPISODES   goals: $GOALS   yaw curve: $YAW_CURVE deg linear"
echo "core limit  : $(ulimit -c)"

# --- Gate 1: the patched planner must be the one that loads (same as leg 2) --------
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
SMAC_2D_LIB="$SMAC_PREFIX/lib/libnav2_smac_planner_2d.so"
# grep -c, not grep -q: under pipefail, grep -q exits on first match, strings takes
# SIGPIPE, and the pipeline reports 141 even though the string was found.
patch_hits=$(strings -a "$SMAC_2D_LIB" 2>/dev/null | grep -c "is outside the costmap bounds" || true)
if [ "${patch_hits:-0}" -eq 0 ]; then
    echo "FATAL: $SMAC_2D_LIB does not carry the bounds check. Rebuild it."
    exit 1
fi
echo "OK - $SMAC_2D_LIB carries the patch"

# --- Gate 2: the goal-start correction must be in the binary (same as leg 2) -------
EM_LIB="$HOME/robot_ws/install/navlearn_benchmarks/lib/libepisode_manager.so"
corr_hits=$(strings -a "$EM_LIB" 2>/dev/null | grep -c "Goal-start correction applied" || true)
if [ "${corr_hits:-0}" -eq 0 ]; then
    echo "FATAL: $EM_LIB has no goal-start correction. Rebuild:"
    echo "  colcon build --packages-select navlearn_benchmarks --symlink-install"
    exit 1
fi
echo "OK - episode_manager carries the goal-start correction"

# --- Gate 3: the yaw-curve mode must be in the binary that will run ----------------
#
# The leg's whole design lives behind kidnap_yaw_mode:=curve. If the running binary
# predates the mode, the parameter is silently unknown to the node, every kidnap
# preserves heading, and the campaign collects 360 goals of nothing — invisible in every
# log until the analysis finds |yaw change| identically zero. Same failure shape as the
# goal-start correction; same gate.
yaw_hits=$(strings -a "$EM_LIB" 2>/dev/null | grep -c "Kidnap severity: YAW CURVE" || true)
if [ "${yaw_hits:-0}" -eq 0 ]; then
    echo "FATAL: $EM_LIB has no yaw-curve mode. The leg would run with heading preserved"
    echo "and every goal unperturbed. Rebuild:"
    echo "  colcon build --packages-select navlearn_benchmarks --symlink-install"
    exit 1
fi
echo "OK - episode_manager carries the yaw-curve mode"

# --- Gate 4: the LAUNCH FILE must declare the yaw arguments ------------------------
#
# The binary carrying the mode is necessary but not sufficient: ros2 launch silently
# discards a key:=value whose argument the launch file never declared, the node falls
# back to "preserve", and the campaign collects unrotated goals under a yaw-curve label.
# The 2026-08-04 pilot collected exactly that — displacement 0.8-1.2 m, yaw change
# identically zero — with every prior gate green.
LAUNCH_FILE="$HOME/robot_ws/install/navlearn_benchmarks/share/navlearn_benchmarks/launch/benchmarks.launch.py"
for arg_name in kidnap_yaw_mode kidnap_yaw_min_rad kidnap_yaw_max_rad; do
    decl_hits=$(grep -c "\"$arg_name\"" "$LAUNCH_FILE" 2>/dev/null || true)
    if [ "${decl_hits:-0}" -lt 2 ]; then
        echo "FATAL: $LAUNCH_FILE does not declare AND forward '$arg_name'."
        echo "The yaw parameters would be silently discarded. Rebuild navlearn_benchmarks."
        exit 1
    fi
done
echo "OK - benchmarks.launch.py declares and forwards the yaw arguments"

# --- The three arms ---------------------------------------------------------------
overall=0
for arm in rpp dwb mppi; do
    say "arm: $arm"
    run_cell "$CAMPAIGN_DIR"_"$arm" \
        --episodes "$EPISODES" --goals "$GOALS" --seed "$SEED" \
        --perturbation ttr --yaw-curve "$YAW_CURVE" \
        -- \
        controller:="$arm" planner:=smac2d localizer:=amcl_tuned
    rc=$?
    echo "### arm $arm finished with rc=$rc"
    [ $rc -ne 0 ] && overall=$rc
done

say "yaw-curve leg complete"
for arm in rpp dwb mppi; do
    d="$CAMPAIGN_DIR"_"$arm"
    rows=$(cat "$d"/navlearn_metrics_run_*[0-9].csv 2>/dev/null | grep -cv "^Goal_ID" || echo 0)
    crashes=$(ls "$d"/forensics_run_*/*.crash 2>/dev/null | wc -l)
    printf '  %-5s %s goal rows, %s crash report(s)\n' "$arm" "$rows" "$crashes"
done
echo "expected per arm: $((EPISODES * GOALS)) goal rows"
echo
echo "analysis (after the recompute, which needs the rosbags):"
echo "  python3 src/navlearn_benchmarks/scripts/analyze_curve_campaign.py \\"
echo "    --arm rpp=${CAMPAIGN_DIR}_rpp --arm dwb=${CAMPAIGN_DIR}_dwb --arm mppi=${CAMPAIGN_DIR}_mppi \\"
echo "    --range 0:180 --bins 4 --magnitude-field yaw_change --bin-scale linear \\"
echo "    --expect-goals $((EPISODES * GOALS)) --output-dir ${CAMPAIGN_DIR}_analysis"
exit $overall
