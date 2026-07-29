#!/usr/bin/env bash

# RETIRED 2026-07-28 — see _retired_guard.sh for why and what replaces it.
source "$(dirname "${BASH_SOURCE[0]}")/_retired_guard.sh"
# Phase 3 Follow-up: fixed_bt orchestrator
# Runs ONLY mppi_baseline_fixed_bt (TTC × 4 levels) to isolate BT-timeout effect
# from velocity tuning. Same structure as phase3_orchestrator.sh.

export ROS_DOMAIN_ID=1
source "$HOME/robot_ws/install/setup.bash" || true
set -eo pipefail

WS="$HOME/robot_ws"
STATE_FILE="$WS/results/phase3_fixed_bt_state.json"
AMCL_CONFIG="$WS/install/bumperbot_localization/share/bumperbot_localization/config/amcl_phase2.yaml"
EPISODES=${EPISODES:-5}
GOALS=${GOALS:-5}
SEED=${SEED:-42}
SIM_LOG="/tmp/phase3_fixed_bt_sim.log"
KILL_GRACE=8
SIM_WARMUP=50
AMCL_RETRIES=30
TOPIC_RETRIES=20

NAV2_PROFILE="mppi_baseline_fixed_bt"
ALL_LEVELS=(easy medium hard extreme)

log()  { echo "[$(date '+%H:%M:%S')] $*"; }
die()  { log "FATAL: $*"; exit 1; }

# ── Sim teardown ──
kill_sim() {
    log "--- SIM TEARDOWN ---"
    local procs="simulated_robot.launch.py|benchmarks.launch.py|gz_sim|gzserver|gzclient|ign gazebo|ros_gz_bridge|parameter_bridge|rviz2|nav2|amcl|lifecycle_manager|map_server|planner_server|controller_server|bt_navigator|behavior_server|waypoint_follower|smoother_server|velocity_smoother|collision_monitor|episode_manager|metrics_compiler|control_metric|trajectory_metric|localization_metrics|ground_truth_publisher|gz_set_pose|navlearn_benchmarks|navlearn_localization_eval|joy_node|joy_teleop|joystick_relay|twist_mux|twist_marker|twist_relay|scan_sanitizer|noisy_controller|safety_stop|robot_state_publisher|controller_manager|spawner"
    local self_pgid
    self_pgid=$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ') || self_pgid=$$
    _safe_pids() {
        ps aux | grep -E "$procs" | grep -v grep | awk '{print $2}' \
            | while read -r pid; do
                local pg; pg=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ') || continue
                [[ "$pg" != "$self_pgid" ]] && echo "$pid"
              done
    }
    log "SIGTERM ROS/Gazebo (pgid ${self_pgid} excluded)..."
    { _safe_pids | xargs -r kill -TERM; } 2>/dev/null || true
    sleep "$KILL_GRACE"
    local survivors; survivors=$(_safe_pids || true)
    if [[ -n "$survivors" ]]; then
        log "SIGKILL survivors: $survivors"
        echo "$survivors" | xargs -r kill -KILL 2>/dev/null || true
        sleep 4
    fi
    local remaining=$(_safe_pids | wc -l); remaining=${remaining// /}
    [[ "${remaining:-0}" -gt 0 ]] && die "Teardown incomplete: $remaining"
    log "Teardown CLEAN."
    sleep 2
}

start_sim() {
    log "--- SIM LAUNCH (nav2_profile=$NAV2_PROFILE) ---"
    ros2 launch bumperbot_bringup simulated_robot.launch.py \
        world_name:=small_house \
        nav2_profile:="$NAV2_PROFILE" \
        amcl_config:="$AMCL_CONFIG" \
        > "$SIM_LOG" 2>&1 &
    SIM_PID=$!
    log "Sim PID=$SIM_PID — warming up ${SIM_WARMUP}s..."
    sleep "$SIM_WARMUP"
}

verify_stack() {
    log "--- STACK VERIFICATION ---"
    for i in $(seq 1 $AMCL_RETRIES); do
        val=$(ros2 param get /amcl sigma_hit 2>/dev/null | grep -oP '[\d.]+' | head -1 || true)
        [[ "$val" == "0.1" ]] && { log "  AMCL: sigma_hit=0.1 ✓"; break; }
        sleep 3; [[ $i -eq $AMCL_RETRIES ]] && die "AMCL sigma_hit never reached 0.1"
    done
    for i in $(seq 1 $TOPIC_RETRIES); do
        ros2 node list 2>/dev/null | grep -q "/map_server" && { log "  map_server: ✓"; break; }
        sleep 3; [[ $i -eq $TOPIC_RETRIES ]] && die "map_server never appeared"
    done
    for i in $(seq 1 $TOPIC_RETRIES); do
        ros2 topic list 2>/dev/null | grep -q "^/amcl_pose$" && { log "  /amcl_pose: ✓"; break; }
        sleep 3; [[ $i -eq $TOPIC_RETRIES ]] && die "/amcl_pose never appeared"
    done
    for i in $(seq 1 $TOPIC_RETRIES); do
        ros2 node list 2>/dev/null | grep -q "bt_navigator" && { log "  bt_navigator: ✓"; break; }
        sleep 3; [[ $i -eq $TOPIC_RETRIES ]] && die "bt_navigator never appeared"
    done
    log "Stack verification PASSED."
}

run_level() {
    local level="$1"
    local out_dir="$WS/results/phase3_${NAV2_PROFILE}_ttc/${level}"
    mkdir -p "$out_dir"
    log "Running $NAV2_PROFILE ttc $level → $out_dir"

    python3 "$WS/src/navlearn_benchmarks/scripts/multi_run_harness.py" \
        --episodes $EPISODES --goals $GOALS --seed $SEED --profile "$NAV2_PROFILE" \
        --output-dir "$out_dir" \
        --extra-arg bad_init_test:=true \
        --extra-arg kidnap_enabled:=false \
        --extra-arg perturbation_level:=$level \
        --extra-arg goal_min_distance_m:=4.0 \
        --extra-arg ttc_timeout_sec:=15.0 \
        --extra-arg ttr_timeout_sec:=15.0 \
        2>&1 | tee "/tmp/phase3_${NAV2_PROFILE}_ttc_${level}.log"

    local n_csv=$(find "$out_dir" -name "navlearn_metrics_run_*.csv" 2>/dev/null | wc -l)
    log "Cell complete: $n_csv CSVs in $out_dir"
}

write_state() {
    python3 - "$STATE_FILE" "$1" <<'PYEOF'
import json, sys
state_file, status = sys.argv[1], sys.argv[2]
try: s = json.load(open(state_file))
except: s = {"status":"running","completed":[],"current":None}
s["status"] = status
json.dump(s, open(state_file, "w"), indent=2)
PYEOF
}

mark_complete() {
    python3 - "$STATE_FILE" "$1" <<'PYEOF'
import json, sys
state_file, entry = sys.argv[1], sys.argv[2]
try: s = json.load(open(state_file))
except: s = {"status":"running","completed":[],"current":None}
if entry not in s["completed"]: s["completed"].append(entry)
s["current"] = None
json.dump(s, open(state_file, "w"), indent=2)
PYEOF
}

# ── Main ──
log "=== Phase 3 Fixed-BT Orchestrator ==="
log "Profile: $NAV2_PROFILE | Episodes: $EPISODES | Goals: $GOALS | Seed: $SEED"
write_state "running"

# Single sim for all 4 levels (same profile)
kill_sim
start_sim
verify_stack

for level in "${ALL_LEVELS[@]}"; do
    log "====== START: ${NAV2_PROFILE}_ttc_${level} ======"
    run_level "$level"
    mark_complete "${NAV2_PROFILE}_ttc_${level}"
done

kill_sim
write_state "complete"
log "=== Phase 3 Fixed-BT COMPLETE ==="
