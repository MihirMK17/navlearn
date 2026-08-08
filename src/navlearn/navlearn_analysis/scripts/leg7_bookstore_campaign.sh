#!/usr/bin/env bash
# Leg 7: second-environment replication in the bookstore (PROTOCOL A5, 2026-08-06).
#
#   bookstore, RPP only, seed 20260730. Two cells:
#     TTC distance curve  bad-init 0.01-5.0 m log   24 ep x 5 goals = 120
#     yaw curve           |yaw| 0-180 deg linear    24 ep x 5 goals = 120
#
# The claim being replicated is that the effects exist outside one floor plan, not that
# the between-controller ordering is stable, so one controller rather than three.
#
# A5 pre-registers a prediction that can fail: a bookstore of repeating shelf aisles
# offers less distinguishing structure under rotation than a house of irregular rooms,
# so the yaw cliff should be STEEPER here than in small_house. A flat or shallower
# cliff is a real result and is reported with the same prominence.
#
# MAP: bookstore_geom, generated from the world's collision geometry at the robot's
# LiDAR height (see maps/bookstore_geom/PROVENANCE.md). Gate 4 refuses anything else.
#
# BACKEND: this is the first campaign collected with Ignition rendering on the NVIDIA
# dGPU (PRIME offload, gazebo.launch.py gpu:=true default). Legs 1-6 and the yaw leg
# rendered on the Intel iGPU. Gate 6 refuses to run if the offload wiring is absent,
# and the small_house GPU control (results/backend_control_small_house) is the evidence
# that the backend does not move the banked results.
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
set -uo pipefail

cd "$HOME/robot_ws"
source src/navlearn/navlearn_analysis/scripts/cell_runner.sh

CAMPAIGN_DIR=results/leg7_bookstore
mkdir -p "$CAMPAIGN_DIR"
exec > >(tee -a "$CAMPAIGN_DIR/campaign.log") 2>&1
ulimit -c unlimited 2>/dev/null || true

SEED=20260730
EPISODES=${EPISODES:-24}
GOALS=5
WORLD=bookstore
MAP_YAML="$HOME/robot_ws/install/navlearn_assets/share/navlearn_assets/maps/bookstore_geom/map.yaml"

say() { printf '\n===== %s  (%s) =====\n' "$*" "$(date '+%F %T')"; }

export NAVLEARN_WORLD="$WORLD"

say "leg 7 bookstore replication starting"
echo "workspace: $(git rev-parse --short HEAD)  seed: $SEED  $EPISODES ep x $GOALS goals"
echo "world: $WORLD   map: $MAP_YAML"

# Gates 1+2: patched planner, goal-start correction (as every other leg).
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
yaw_hits=$(strings -a "$EM_LIB" 2>/dev/null | grep -c "Kidnap severity: YAW CURVE" || true)
[ "${yaw_hits:-0}" -eq 0 ] && { echo "FATAL: yaw-curve mode missing from the binary"; exit 1; }
echo "OK - planner patch, goal-start correction, yaw-curve mode all present"

# Gate 3: the launch files must declare AND forward both the yaw arguments and the map
# argument. An undeclared launch argument is discarded in silence, which is how the
# 2026-08-04 yaw pilot collected 5 goals of the wrong experiment with every other gate
# green. A discarded map argument would be worse: the run would localize against
# small_house while driving around a bookstore.
LAUNCH_BENCH="$HOME/robot_ws/install/navlearn_benchmarks/share/navlearn_benchmarks/launch/benchmarks.launch.py"
for arg_name in kidnap_yaw_mode kidnap_yaw_min_rad kidnap_yaw_max_rad; do
    [ "$(grep -c "\"$arg_name\"" "$LAUNCH_BENCH" 2>/dev/null || echo 0)" -lt 2 ] && {
        echo "FATAL: benchmarks.launch.py does not declare and forward '$arg_name'"; exit 1; }
done
LAUNCH_SIM="$HOME/robot_ws/install/bumperbot_bringup/share/bumperbot_bringup/launch/simulated_robot.launch.py"
[ "$(grep -c "map_yaml" "$LAUNCH_SIM" 2>/dev/null || echo 0)" -lt 2 ] && {
    echo "FATAL: simulated_robot.launch.py does not declare and forward 'map_yaml'."
    echo "The campaign would silently localize against the small_house map."; exit 1; }
echo "OK - launch files declare and forward the yaw and map arguments"

# Gate 4: the map must be the geometric one, and must match this world's true extent.
# A wrong or rotated map would measure ground-truth error in a frame the robot is not in.
[ -f "$MAP_YAML" ] || { echo "FATAL: $MAP_YAML missing. Rebuild navlearn_assets."; exit 1; }
case "$MAP_YAML" in
    *bookstore_geom*) ;;
    *) echo "FATAL: refusing to run against a map other than bookstore_geom"; exit 1 ;;
esac
python3 - "$MAP_YAML" <<'PY' || exit 1
import pathlib, sys
meta = {}
for line in pathlib.Path(sys.argv[1]).read_text().splitlines():
    if ":" in line:
        k, _, v = line.partition(":")
        meta[k.strip()] = v.strip()
img = pathlib.Path(sys.argv[1]).parent / meta["image"]
if not img.exists():
    print(f"FATAL: {img} missing"); sys.exit(1)
with open(img, "rb") as f:
    head = f.read(64).split()
w, h = int(head[1]), int(head[2])
res = float(meta["resolution"])
span_x, span_y = w * res, h * res
# The generated bookstore grid measures 17.8 x 16.7 m including the 1 m margin each
# side (356 x 334 cells at 0.05 m). Anything far from that is a map of a different
# world, or a different orientation.
if not (16 < span_x < 20 and 15 < span_y < 19):
    print(f"FATAL: map spans {span_x:.1f} x {span_y:.1f} m, which does not match the "
          f"bookstore geometry (17.8 x 16.7 m including margin). Wrong or rotated map.")
    sys.exit(1)
print(f"OK - map spans {span_x:.1f} x {span_y:.1f} m at {res} m/cell, consistent with the world")
PY

# Gate 5: disk. Every episode records a rosbag, and the bags are not optional -- the
# episode-window recovery outcome is computed from them, so a campaign that runs out of
# space mid-flight loses the outcome, not merely a log. The comparable RPP yaw cell used
# 5 GB for 24 runs; 40 GB leaves room for both cells and a wide margin.
FREE_GB=$(df -BG --output=avail "$HOME/robot_ws" | tail -1 | tr -dc '0-9')
if [ "${FREE_GB:-0}" -lt 40 ]; then
    echo "FATAL: only ${FREE_GB} GB free. Two cells of bags need roughly 15 GB and a"
    echo "campaign that fills the disk loses the outcome it was run to measure."
    exit 1
fi
echo "OK - ${FREE_GB} GB free for bags"

# Gate 6: the NVIDIA render offload must actually be wired, because this campaign's
# scan-rate premise depends on it. Legs 1-6 rendered the GPU LiDAR on the Intel iGPU at
# 3.3-6.2 Hz in this world; if the offload wiring regressed, every cell would fail its
# scan_rate_met gate hours in rather than here. Checks the installed launch file (what
# actually runs), the NVIDIA EGL vendor manifest the launch points at, and the driver.
GAZEBO_LAUNCH="$HOME/robot_ws/install/bumperbot_description/share/bumperbot_description/launch/gazebo.launch.py"
[ "$(grep -c "__NV_PRIME_RENDER_OFFLOAD" "$GAZEBO_LAUNCH" 2>/dev/null || echo 0)" -lt 1 ] && {
    echo "FATAL: gazebo.launch.py does not set the PRIME offload environment."; exit 1; }
[ "$(grep -c "\"gpu\"" "$LAUNCH_SIM" 2>/dev/null || echo 0)" -lt 2 ] && {
    echo "FATAL: simulated_robot.launch.py does not declare and forward 'gpu'."; exit 1; }
[ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ] || {
    echo "FATAL: NVIDIA EGL vendor manifest missing; the headless path would fall back"
    echo "to the Intel iGPU silently."; exit 1; }
DRIVER=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1)
[ -z "$DRIVER" ] && { echo "FATAL: nvidia-smi reports no usable NVIDIA driver."; exit 1; }
echo "OK - NVIDIA offload wired (driver $DRIVER); backend recorded in campaign.log"

overall=0
run_and_track() {
    local label="$1"; shift
    say "cell: $label"
    "$@"
    local rc=$?
    echo "### cell $label finished with rc=$rc"
    [ $rc -ne 0 ] && overall=$rc
    return 0
}

run_and_track "bookstore_ttc_curve" \
    run_cell "$CAMPAIGN_DIR"_ttc \
        --episodes "$EPISODES" --goals "$GOALS" --seed "$SEED" \
        --perturbation ttc --magnitude-curve 0.01:5.0 --magnitude-scale log \
        --extra-arg exclusion_zones:="[]" \
        -- \
        controller:=rpp planner:=smac2d localizer:=amcl_tuned \
        map_yaml:="$MAP_YAML"

run_and_track "bookstore_yaw_curve" \
    run_cell "$CAMPAIGN_DIR"_yaw \
        --episodes "$EPISODES" --goals "$GOALS" --seed "$SEED" \
        --perturbation ttr --yaw-curve 0:180 \
        --extra-arg exclusion_zones:="[]" \
        -- \
        controller:=rpp planner:=smac2d localizer:=amcl_tuned \
        map_yaml:="$MAP_YAML"

say "leg 7 campaign complete"
for cell in ttc yaw; do
    d="$CAMPAIGN_DIR"_"$cell"
    rows=$(cat "$d"/navlearn_metrics_run_*[0-9].csv 2>/dev/null | grep -cv "^Goal_ID" || echo 0)
    crashes=$(ls "$d"/forensics_run_*/*.crash 2>/dev/null | wc -l)
    printf '  %-4s %s goal rows, %s crash report(s)\n' "$cell" "$rows" "$crashes"
done
echo "expected per cell: $((EPISODES * GOALS)) goal rows"
exit $overall
