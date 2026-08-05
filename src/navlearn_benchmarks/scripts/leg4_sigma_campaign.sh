#!/usr/bin/env bash
# Leg 4: sigma_hit basin ablation (decisions doc §11).
#
#   sigma_hit ∈ {0.05, 0.1, 0.2, 0.4} × RPP × small_house × fixed TTR band (medium
#   preset, 0.8–1.2 m ring) × 5 runs × 5 goals = 100 goals, seed 20260730.
#
# Mechanism leg for §10: sigma_hit sets the width of the likelihood basin AMCL climbs
# during recovery. The perturbation band is FIXED so the localizer parameter is the only
# thing that varies — same one-variable rule as every other leg. The 0.1 level runs the
# campaign localizer fragment content-identically (amcl_sigma_hit_01 is a byte-copy of
# amcl_tuned), so the ablation's control arm IS the campaign configuration.
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
set -uo pipefail

cd "$HOME/robot_ws"
source src/navlearn_benchmarks/scripts/cell_runner.sh

CAMPAIGN_DIR=results/leg4_sigma
mkdir -p "$CAMPAIGN_DIR"
exec > >(tee -a "$CAMPAIGN_DIR/campaign.log") 2>&1
ulimit -c unlimited 2>/dev/null || true

SEED=20260730
EPISODES=5
GOALS=5

say() { printf '\n===== %s  (%s) =====\n' "$*" "$(date '+%F %T')"; }

say "leg 4 sigma_hit ablation starting"
echo "workspace: $(git rev-parse --short HEAD)  seed: $SEED  5 runs x 5 goals per level"

# Gate 1: patched planner in front (same as legs 2/yaw).
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

# Gate 2: goal-start correction in the binary.
EM_LIB="$HOME/robot_ws/install/navlearn_benchmarks/lib/libepisode_manager.so"
corr_hits=$(strings -a "$EM_LIB" 2>/dev/null | grep -c "Goal-start correction applied" || true)
[ "${corr_hits:-0}" -eq 0 ] && { echo "FATAL: goal-start correction missing"; exit 1; }
echo "OK - goal-start correction present"

# Gate 3: every sigma fragment must exist, be installed, and differ from the control
# ONLY in sigma_hit. A fragment that drifted from amcl_tuned in any other key would make
# this a two-variable ablation without anything else noticing.
FRAG_DIR=$(ros2 pkg prefix bumperbot_navigation)/share/bumperbot_navigation/config/nav2_stack/localizers
for name in amcl_sigma_hit_005 amcl_sigma_hit_01 amcl_sigma_hit_02 amcl_sigma_hit_04; do
    [ -f "$FRAG_DIR/$name.yaml" ] || { echo "FATAL: $name.yaml not installed"; exit 1; }
done
for name in amcl_sigma_hit_005 amcl_sigma_hit_02 amcl_sigma_hit_04; do
    if ! diff <(grep -v sigma_hit "$FRAG_DIR/$name.yaml") \
              <(grep -v sigma_hit "$FRAG_DIR/amcl_tuned.yaml") >/dev/null; then
        echo "FATAL: $name.yaml differs from amcl_tuned beyond sigma_hit"; exit 1
    fi
done
diff "$FRAG_DIR/amcl_sigma_hit_01.yaml" "$FRAG_DIR/amcl_tuned.yaml" >/dev/null \
    || { echo "FATAL: amcl_sigma_hit_01 is not a byte-copy of amcl_tuned"; exit 1; }
echo "OK - all four fragments single-variable against amcl_tuned"

overall=0
for sig in 005 01 02 04; do
    say "sigma_hit level: $sig"
    run_cell "$CAMPAIGN_DIR"_"$sig" \
        --episodes "$EPISODES" --goals "$GOALS" --seed "$SEED" \
        --perturbation ttr --perturbation-level medium \
        -- \
        controller:=rpp planner:=smac2d localizer:=amcl_sigma_hit_"$sig"
    rc=$?
    echo "### level $sig finished with rc=$rc"
    [ $rc -ne 0 ] && overall=$rc
done

say "leg 4 complete"
for sig in 005 01 02 04; do
    d="$CAMPAIGN_DIR"_"$sig"
    rows=$(cat "$d"/navlearn_metrics_run_*[0-9].csv 2>/dev/null | grep -cv "^Goal_ID" || echo 0)
    printf '  sigma %-4s %s goal rows\n' "$sig" "$rows"
done
echo "expected per level: $((EPISODES * GOALS))"
exit $overall
