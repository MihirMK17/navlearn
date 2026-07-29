#!/usr/bin/env bash

# RETIRED 2026-07-28 — see _retired_guard.sh for why and what replaces it.
source "$(dirname "${BASH_SOURCE[0]}")/_retired_guard.sh"
# Phase 3 follow-up: seed replication on extreme cells
# Runs mppi_baseline + mppi_aggressive at extreme TTC + extreme TTR
# with seeds 43 and 44 (in addition to seed=42 already in Phase 3).
# Validates Wilson CIs across seed selection — addresses single-seed critique.

export ROS_DOMAIN_ID=1
source "$HOME/robot_ws/install/setup.bash" || true
set -eo pipefail

WS="$HOME/robot_ws"
STATE_FILE="$WS/results/phase3_seed_repl_state.json"
AMCL_CONFIG="$WS/install/bumperbot_localization/share/bumperbot_localization/config/amcl_phase2.yaml"
EPISODES=${EPISODES:-5}
GOALS=${GOALS:-5}
SIM_LOG="/tmp/phase3_seed_repl_sim.log"
KILL_GRACE=8
SIM_WARMUP=50
AMCL_RETRIES=30
TOPIC_RETRIES=20

# 2 profiles × 2 modes × 2 seeds = 8 cells (extreme level only)
PROFILES=(mppi_baseline mppi_aggressive)
SEEDS=(43 44)
MODES=(ttc ttr)

log()  { echo "[$(date '+%H:%M:%S')] $*"; }
die()  { log "FATAL: $*"; exit 1; }

kill_sim() {
    log "--- SIM TEARDOWN ---"
    local procs="simulated_robot.launch.py|benchmarks.launch.py|gz_sim|gzserver|gzclient|ign gazebo|ros_gz_bridge|parameter_bridge|rviz2|nav2|amcl|lifecycle_manager|map_server|planner_server|controller_server|bt_navigator|behavior_server|waypoint_follower|smoother_server|velocity_smoother|collision_monitor|episode_manager|metrics_compiler|control_metric|trajectory_metric|localization_metrics|ground_truth_publisher|gz_set_pose|navlearn_benchmarks|navlearn_localization_eval|joy_node|joy_teleop|joystick_relay|twist_mux|twist_marker|twist_relay|scan_sanitizer|noisy_controller|safety_stop|robot_state_publisher|controller_manager|spawner"
    local self_pgid; self_pgid=$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ') || self_pgid=$$
    _safe_pids() {
        ps aux | grep -E "$procs" | grep -v grep | awk '{print $2}' | while read -r pid; do
            local pg; pg=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ') || continue
            [[ "$pg" != "$self_pgid" ]] && echo "$pid"
        done
    }
    { _safe_pids | xargs -r kill -TERM; } 2>/dev/null || true
    sleep "$KILL_GRACE"
    local survivors; survivors=$(_safe_pids || true)
    [[ -n "$survivors" ]] && echo "$survivors" | xargs -r kill -KILL 2>/dev/null || true
    sleep 4
    log "Teardown done."
}

start_sim() {
    local profile="$1"
    log "--- SIM LAUNCH (nav2_profile=$profile) ---"
    ros2 launch bumperbot_bringup simulated_robot.launch.py \
        world_name:=small_house \
        nav2_profile:="$profile" \
        amcl_config:="$AMCL_CONFIG" \
        > "$SIM_LOG" 2>&1 &
    SIM_PID=$!
    log "Sim PID=$SIM_PID — warming ${SIM_WARMUP}s..."
    sleep "$SIM_WARMUP"
}

verify_stack() {
    for i in $(seq 1 $AMCL_RETRIES); do
        val=$(ros2 param get /amcl sigma_hit 2>/dev/null | grep -oP '[\d.]+' | head -1 || true)
        [[ "$val" == "0.1" ]] && break
        sleep 3; [[ $i -eq $AMCL_RETRIES ]] && die "AMCL not ready"
    done
    for i in $(seq 1 $TOPIC_RETRIES); do
        ros2 node list 2>/dev/null | grep -q "bt_navigator" && { log "Stack ready"; return 0; }
        sleep 3
    done
    die "bt_navigator never appeared"
}

run_cell() {
    local profile="$1" mode="$2" seed="$3"
    local out_dir="$WS/results/phase3_seed_repl/${profile}_${mode}_extreme_seed${seed}"
    local bad_init kidnap
    [[ "$mode" == "ttc" ]] && { bad_init="true"; kidnap="false"; } || { bad_init="false"; kidnap="true"; }
    mkdir -p "$out_dir"
    log "====== START: $profile $mode extreme seed=$seed → $out_dir ======"

    python3 "$WS/src/navlearn_benchmarks/scripts/multi_run_harness.py" \
        --episodes $EPISODES --goals $GOALS --seed "$seed" --profile "$profile" \
        --output-dir "$out_dir" \
        --extra-arg bad_init_test:=$bad_init \
        --extra-arg kidnap_enabled:=$kidnap \
        --extra-arg perturbation_level:=extreme \
        --extra-arg goal_min_distance_m:=4.0 \
        --extra-arg ttc_timeout_sec:=15.0 \
        --extra-arg ttr_timeout_sec:=15.0 \
        2>&1 | tee "/tmp/phase3_seed_repl_${profile}_${mode}_seed${seed}.log"

    local n_csv=$(find "$out_dir" -name "navlearn_metrics_run_*.csv" 2>/dev/null | wc -l)
    log "Cell done: $n_csv CSVs"
}

mark_complete() {
    python3 - "$STATE_FILE" "$1" <<'PYEOF'
import json, sys
state_file, entry = sys.argv[1], sys.argv[2]
try: s = json.load(open(state_file))
except: s = {"status":"running","completed":[]}
if entry not in s["completed"]: s["completed"].append(entry)
json.dump(s, open(state_file, "w"), indent=2)
PYEOF
}

log "=== Phase 3 Seed Replication ==="
log "Profiles: ${PROFILES[*]} | Modes: ${MODES[*]} | Seeds: ${SEEDS[*]} | Extreme only"
mkdir -p "$(dirname $STATE_FILE)"
echo '{"status":"running","completed":[]}' > "$STATE_FILE"

for profile in "${PROFILES[@]}"; do
    kill_sim
    start_sim "$profile"
    verify_stack
    for mode in "${MODES[@]}"; do
        for seed in "${SEEDS[@]}"; do
            run_cell "$profile" "$mode" "$seed"
            mark_complete "${profile}_${mode}_extreme_seed${seed}"
        done
    done
done

kill_sim
python3 -c "
import json
s = json.load(open('$STATE_FILE'))
s['status'] = 'complete'
json.dump(s, open('$STATE_FILE','w'), indent=2)
"
log "=== Phase 3 Seed Replication COMPLETE ==="
