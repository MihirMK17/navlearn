#!/usr/bin/env bash
# Phase 3 sensor-rate stress test: original 4th test case (no bad-init, no kidnap, LiDAR rate reduced).
# Lever: gpu_lidar <update_rate> in bumperbot_description/urdf/bumperbot_gazebo.xacro (symlink-installed,
# so editing src + restarting sim is sufficient; no colcon rebuild needed).
# Profile: RPP baseline + tuned AMCL (amcl_phase2.yaml) -- SAME controller as Phase 1/2 TTC/TTR localization
#          tests, so sensor-rate effect is not confounded with a controller change.
# Tiers: 10 Hz (control) / 5 / 2 / 1 Hz. Clean navigation under sensor starvation.
# Cells: 4 tiers x 5 episodes x 5 goals x seed 42 = 100 goals.
# Output: results/phase3_sensorrate_baseline/${rate}hz/
# ETA: ~2.5-3 hours background.

export ROS_DOMAIN_ID=1
source "$HOME/robot_ws/install/setup.bash" || true
set -eo pipefail

WS="$HOME/robot_ws"
XACRO="$WS/src/bumperbot_description/urdf/bumperbot_gazebo.xacro"
ORIG_RATE=10                       # restored on exit
STATE_FILE="$WS/results/phase3_sensorrate_state.json"
AMCL_CONFIG="$WS/install/bumperbot_localization/share/bumperbot_localization/config/amcl_phase2.yaml"
PROFILE=baseline
SEED=42
EPISODES=${EPISODES:-5}
GOALS=${GOALS:-5}
SIM_LOG="/tmp/phase3_sensorrate_sim.log"
KILL_GRACE=8
SIM_WARMUP=70          # was 50; gz needs more time to sync /clock under load (per-tier relaunch race)
AMCL_RETRIES=45        # was 30; more patience for amcl lifecycle activation
TOPIC_RETRIES=20
CLEAN_POLL=15          # kill_sim: poll iterations (x2s) to confirm procs actually gone

TIERS=(10 5 2 1)                   # LiDAR update_rate Hz; first = control

log()  { echo "[$(date '+%H:%M:%S')] $*"; }
die()  { log "FATAL: $*"; exit 1; }

# --- on ANY exit (success, die(), Ctrl-C): tear down sim AND restore xacro ---
# Without the kill_sim here, a die() in verify_stack leaks the whole sim process tree.
restore_xacro() {
    set_lidar_rate "$ORIG_RATE" 2>/dev/null || true
    log "xacro restored to ${ORIG_RATE} Hz."
}
cleanup() {
    log "--- CLEANUP (exit) ---"
    kill_sim || true
    restore_xacro
}
trap cleanup EXIT

# Replace <update_rate> ONLY inside the lidar sensor block (avoid IMU's update_rate=100).
set_lidar_rate() {
    local rate="$1"
    sed -i '/<sensor name="lidar"/,/<\/sensor>/ s|<update_rate>[0-9]*</update_rate>|<update_rate>'"$rate"'</update_rate>|' "$XACRO"
    local got
    got=$(sed -n '/<sensor name="lidar"/,/<\/sensor>/ s|.*<update_rate>\([0-9]*\)</update_rate>.*|\1|p' "$XACRO" | head -1)
    [[ "$got" == "$rate" ]] || die "lidar rate set failed (wanted $rate, got '$got')"
}

kill_sim() {
    # Mirrors the battle-tested phase2_orchestrator teardown (relaunches sim 8x reliably).
    # Pattern matches NODE executables only -- NOT navlearn_* package paths, so it can never
    # match this orchestrator's own bash/wrapper even when invoked by full path (the package
    # tokens caused the exit-144 self-kill). No pgid exclusion needed as a result.
    log "--- SIM TEARDOWN ---"
    local procs="simulated_robot.launch.py|benchmarks.launch.py|gz_sim|gzserver|gzclient|ign gazebo|ros_gz_bridge|parameter_bridge|rviz2|amcl|lifecycle_manager|map_server|planner_server|controller_server|bt_navigator|behavior_server|waypoint_follower|smoother_server|velocity_smoother|collision_monitor|episode_manager|metrics_compiler|control_metric|trajectory_metric|localization_metrics|ground_truth_publisher|gz_set_pose|multi_run_harness|joy_node|joy_teleop|joystick_relay|twist_mux|twist_marker|twist_relay|scan_sanitizer|noisy_controller|safety_stop|robot_state_publisher|controller_manager|spawner"
    log "SIGTERM to ROS/Gazebo processes..."
    { ps aux | grep -E "$procs" | grep -v grep | awk '{print $2}' | xargs -r kill -TERM; } 2>/dev/null || true
    sleep "$KILL_GRACE"
    local survivors
    survivors=$(ps aux | grep -E "$procs" | grep -v grep | awk '{print $2}' || true)
    if [[ -n "$survivors" ]]; then
        log "SIGKILL survivors: $survivors"
        echo "$survivors" | xargs -r kill -KILL 2>/dev/null || true
        sleep 4
    fi
    local remaining=0
    remaining=$(ps aux | grep -E "$procs" | grep -v grep | wc -l 2>/dev/null) || remaining=0
    remaining=$(echo "$remaining" | tr -d ' ')
    if [[ "${remaining:-0}" -gt 0 ]]; then
        log "WARNING: $remaining processes still alive after SIGKILL:"
        ps aux | grep -E "$procs" | grep -v grep || true
        die "Sim teardown incomplete -- cannot start next tier with leftover processes."
    fi
    log "Sim teardown CLEAN -- zero processes remaining."
    sleep 2  # let OS release ports/sockets
}

start_sim() {
    # Plain launch, identical to the proven phase2 orchestrator (NO setsid -- setsid detaches the
    # session and breaks the /clock bridge -> frozen sim time -> TF cache stalls -> stack never ready).
    log "--- SIM LAUNCH (nav2_profile=$PROFILE) ---"
    ros2 launch bumperbot_bringup simulated_robot.launch.py \
        world_name:=small_house \
        nav2_profile:="$PROFILE" \
        amcl_config:="$AMCL_CONFIG" \
        > "$SIM_LOG" 2>&1 &
    SIM_PID=$!
    log "Sim PID=$SIM_PID -- warming ${SIM_WARMUP}s..."
    sleep "$SIM_WARMUP"
}

verify_stack() {
    log "--- STACK VERIFICATION (log-based, daemon-independent) ---"
    # Canonical readiness signal (per navlearn-harness-workflow memory + README): BOTH lifecycle
    # managers print "Managed nodes are active" -- localization (map_server+amcl) and navigation
    # (controller+planner+bt_navigator+...). This is the documented-working gate.
    # We do NOT gate on `ros2 param get /amcl sigma_hit`: that query depends on a healthy ros2
    # daemon and silently fails when the daemon is stale, which made the harness never start even
    # though the stack was fully active. Log grep is a file read -- immune to daemon state.
    local need=2 got=0
    for i in $(seq 1 $AMCL_RETRIES); do
        got=$(grep -c "Managed nodes are active" "$SIM_LOG" 2>/dev/null) || got=0
        got=$(echo "$got" | tr -d ' ')
        [[ "${got:-0}" -ge "$need" ]] && { log "Stack ready ($got/$need lifecycle managers active)"; break; }
        sleep 3
        [[ $i -eq $AMCL_RETRIES ]] && { tail -25 "$SIM_LOG"; die "Stack not active ($got/$need managers) after $((AMCL_RETRIES*3))s"; }
    done
    # Soft config confirmation (warn-only). amcl_phase2.yaml is passed explicitly, so sigma_hit=0.1
    # holds by construction; we try to confirm via a FRESH daemon but never FATAL on the query.
    ros2 daemon stop >/dev/null 2>&1 || true
    local val; val=$(timeout 10 ros2 param get /amcl sigma_hit 2>/dev/null | grep -oP '[\d.]+' | head -1 || true)
    if [[ "$val" == "0.1" ]]; then
        log "Confirmed amcl sigma_hit=0.1 (Phase 2 config active)."
    else
        log "NOTE: sigma_hit not confirmable via param query (val='$val'); trusting amcl_config=$(basename "$AMCL_CONFIG")."
    fi
    return 0
}

# Confirm the running sim is actually publishing /scan at the requested rate (sanity gate).
verify_scan_rate() {
    local rate="$1"
    local hz
    hz=$(timeout 12 ros2 topic hz /scan 2>/dev/null | grep -oP 'average rate: \K[0-9.]+' | head -1 || true)
    if [[ -z "$hz" ]]; then
        log "WARN: could not measure /scan rate (continuing; tier=$rate Hz)"
    else
        log "Measured /scan average rate ~= ${hz} Hz (requested ${rate} Hz)"
    fi
}

run_cell() {
    local rate="$1"
    local out_dir="$WS/results/phase3_sensorrate_baseline/${rate}hz"
    mkdir -p "$out_dir"
    log "====== START: sensor-rate ${rate}Hz | $PROFILE clean seed=$SEED -> $out_dir ======"

    python3 "$WS/src/navlearn_benchmarks/scripts/multi_run_harness.py" \
        --episodes $EPISODES --goals $GOALS --seed "$SEED" --profile "$PROFILE" \
        --output-dir "$out_dir" \
        --extra-arg bad_init_test:=false \
        --extra-arg kidnap_enabled:=false \
        --extra-arg perturbation_level:=easy \
        --extra-arg goal_min_distance_m:=4.0 \
        --extra-arg ttc_timeout_sec:=15.0 \
        --extra-arg ttr_timeout_sec:=15.0 \
        2>&1 | tee "/tmp/phase3_sensorrate_${rate}hz.log"

    local n_csv; n_csv=$(find "$out_dir" -name "navlearn_metrics_run_*.csv" 2>/dev/null | wc -l)
    log "Cell done: $n_csv CSVs in $out_dir"
}

mark_complete() {
    python3 - "$STATE_FILE" "$1" <<'PYEOF'
import json, sys
state_file, entry = sys.argv[1], sys.argv[2]
try: s = json.load(open(state_file))
except Exception: s = {"status":"running","completed":[]}
if entry not in s["completed"]: s["completed"].append(entry)
json.dump(s, open(state_file, "w"), indent=2)
PYEOF
}

log "=== Phase 3 Sensor-Rate Stress (4th test case) ==="
log "Profile: $PROFILE | Tiers: ${TIERS[*]} Hz | seed $SEED | ${EPISODES}x${GOALS}=$((EPISODES*GOALS))/tier"
log "Total: ${#TIERS[@]} tiers x $((EPISODES*GOALS)) = $(( ${#TIERS[@]} * EPISODES * GOALS )) goals"
mkdir -p "$(dirname $STATE_FILE)"
echo '{"status":"running","completed":[]}' > "$STATE_FILE"

for rate in "${TIERS[@]}"; do
    log "########## TIER: LiDAR ${rate} Hz ##########"
    kill_sim
    set_lidar_rate "$rate"          # edit symlinked xacro; sim picks it up on next launch
    start_sim
    verify_stack
    verify_scan_rate "$rate"
    run_cell "$rate"
    mark_complete "${rate}hz"
done

kill_sim
python3 - "$STATE_FILE" <<'PYEOF'
import json, sys
s = json.load(open(sys.argv[1])); s["status"] = "done"
json.dump(s, open(sys.argv[1], "w"), indent=2)
PYEOF
log "=== ALL TIERS DONE. Results: $WS/results/phase3_sensorrate_baseline/ ==="
