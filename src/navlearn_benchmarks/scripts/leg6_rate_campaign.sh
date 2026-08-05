#!/usr/bin/env bash
# Leg 6: sensor-rate stress (decisions doc §11, "leg 6 is mechanism, not just robustness").
#
#   Mechanism sweep: {10, 5, 2, 1} Hz × {clean, ttc-mid, ttr-mid} × RPP,
#                    5 runs × 5 goals per cell = 12 cells, 300 goals.
#   Subject endpoints: {10, 1} Hz × {clean, ttr-mid} × {DWB, MPPI},
#                    5 runs × 5 goals per cell = 8 cells, 200 goals.
#
#   Seed 20260730 throughout. NOTE on the endpoint count: §11's table says "2 × 2 × 25
#   = 100" for the endpoints while listing three controllers; RPP's endpoint cells are
#   already inside the mechanism sweep, and 25 goals/cell is the unit every other leg
#   uses, so the endpoints here are the two NEW controllers at 25 goals/cell = 200.
#   Recorded as a deviation, not silently.
#
# Falsifiable prediction on record: sensor rate degrades TTR far more steeply than TTC.
# If they degrade in parallel, the §10 basin account is wrong and gets reported as such.
#
# Every cell, including 10 Hz, runs WITH the governor in the scan path (pass-through at
# native rate): a 10 Hz cell without the governor would differ from a starved cell in
# topology as well as rate — two variables.
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
set -uo pipefail

cd "$HOME/robot_ws"
source src/navlearn_benchmarks/scripts/cell_runner.sh

CAMPAIGN_DIR=results/leg6_rate
mkdir -p "$CAMPAIGN_DIR"
exec > >(tee -a "$CAMPAIGN_DIR/campaign.log") 2>&1
ulimit -c unlimited 2>/dev/null || true

SEED=20260730
EPISODES=5
GOALS=5

say() { printf '\n===== %s  (%s) =====\n' "$*" "$(date '+%F %T')"; }

say "leg 6 sensor-rate stress starting"
echo "workspace: $(git rev-parse --short HEAD)  seed: $SEED  5 runs x 5 goals per cell"

# Gates 1+2: patched planner in front, goal-start correction present (same as legs 2/yaw/4).
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
echo "OK - planner carries the patch"
EM_LIB="$HOME/robot_ws/install/navlearn_benchmarks/lib/libepisode_manager.so"
corr_hits=$(strings -a "$EM_LIB" 2>/dev/null | grep -c "Goal-start correction applied" || true)
[ "${corr_hits:-0}" -eq 0 ] && { echo "FATAL: goal-start correction missing"; exit 1; }
echo "OK - goal-start correction present"

# Gate 3: the governor executable must exist in the installed package — a rate cell
# whose governor silently failed to launch runs unstarved under a starved label, which
# is invisible until the rate monitor's post-hoc numbers are read.
GOV_BIN="$HOME/robot_ws/install/navlearn_benchmarks/lib/navlearn_benchmarks/scan_rate_governor"
[ -x "$GOV_BIN" ] || { echo "FATAL: scan_rate_governor not installed/executable"; exit 1; }
echo "OK - scan_rate_governor installed"

# One cell. $1 dir, $2 rate, $3 regime flags..., last arg controller.
cell() {
    local dir="$1" rate="$2" ctrl="$3"; shift 3
    run_cell "$dir" \
        --episodes "$EPISODES" --goals "$GOALS" --seed "$SEED" \
        "$@" \
        -- \
        controller:="$ctrl" planner:=smac2d localizer:=amcl_tuned \
        scan_rate_hz:="$rate"
}

overall=0
declare -a FAILED_CELLS=()

run_and_track() {
    local label="$1"; shift
    say "cell: $label"
    "$@"
    local rc=$?
    echo "### cell $label finished with rc=$rc"
    if [ $rc -ne 0 ]; then overall=$rc; FAILED_CELLS+=("$label"); fi
}

# --- Mechanism sweep: RPP, all rates, all regimes -----------------------------------
for rate in 10 5 2 1; do
    run_and_track "rpp_clean_${rate}hz" \
        cell "$CAMPAIGN_DIR/rpp_clean_${rate}hz" "$rate" rpp --perturbation clean
    run_and_track "rpp_ttcmid_${rate}hz" \
        cell "$CAMPAIGN_DIR/rpp_ttcmid_${rate}hz" "$rate" rpp \
        --perturbation ttc --perturbation-level medium
    run_and_track "rpp_ttrmid_${rate}hz" \
        cell "$CAMPAIGN_DIR/rpp_ttrmid_${rate}hz" "$rate" rpp \
        --perturbation ttr --perturbation-level medium
done

# --- Subject endpoints: DWB and MPPI at the rate extremes ---------------------------
for ctrl in dwb mppi; do
    for rate in 10 1; do
        run_and_track "${ctrl}_clean_${rate}hz" \
            cell "$CAMPAIGN_DIR/${ctrl}_clean_${rate}hz" "$rate" "$ctrl" --perturbation clean
        run_and_track "${ctrl}_ttrmid_${rate}hz" \
            cell "$CAMPAIGN_DIR/${ctrl}_ttrmid_${rate}hz" "$rate" "$ctrl" \
            --perturbation ttr --perturbation-level medium
    done
done

say "leg 6 complete"
total_rows=0
for d in "$CAMPAIGN_DIR"/*/; do
    rows=$(cat "$d"/navlearn_metrics_run_*[0-9].csv 2>/dev/null | grep -cv "^Goal_ID" || echo 0)
    printf '  %-22s %s goal rows\n' "$(basename "$d")" "$rows"
    total_rows=$((total_rows + rows))
done
echo "total goal rows: $total_rows (expected 500)"
[ ${#FAILED_CELLS[@]} -gt 0 ] && echo "FAILED CELLS: ${FAILED_CELLS[*]}"
exit $overall
