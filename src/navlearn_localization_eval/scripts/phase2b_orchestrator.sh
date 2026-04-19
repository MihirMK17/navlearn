#!/usr/bin/env bash
# Phase 2b NavLearn Orchestrator — TTR-only ablation
# Isolates H1: revert recovery_alpha_fast 0.05 → 0.1 while keeping other Phase 2 changes.
# Same graceful teardown + CSV-outcome validation as Phase 2 orchestrator v3.

export ROS_DOMAIN_ID=1
source "$HOME/robot_ws/install/setup.bash" || true

set -eo pipefail

WS="$HOME/robot_ws"
STATE_FILE="$WS/results/phase2b_state.json"
AMCL_CONFIG="$WS/install/navlearn_localization_eval/share/navlearn_localization_eval/config/amcl_phase2b.yaml"
EPISODES=5
GOALS=5
SEED=42
PROFILE="aggressive"
SIM_LOG="/tmp/phase2b_sim.log"
KILL_GRACE=8
SIM_WARMUP=50
AMCL_RETRIES=30
TOPIC_RETRIES=20

log() { echo "[$(date '+%H:%M:%S')] $*"; }
die() { log "FATAL: $*"; exit 1; }

init_state() {
    mkdir -p "$WS/results"
    [[ -f "$STATE_FILE" ]] || python3 -c "
import json, datetime
s = {'status':'running','completed':[],'pending':['ttr_easy','ttr_medium','ttr_hard','ttr_extreme'],'current':None,'started_at':None,'last_updated':datetime.datetime.now().isoformat()}
open('$STATE_FILE','w').write(json.dumps(s,indent=2))
print('Phase 2b state initialized.')
"
}

update_state() {
    local level="$1" action="$2"
    python3 - "$STATE_FILE" "$level" "$action" <<'PYEOF'
import json, sys, datetime
path, level, action = sys.argv[1:]
with open(path) as f: s = json.load(f)
now = datetime.datetime.now().isoformat()
if action == "start":
    s["current"] = level; s["started_at"] = now
elif action == "complete":
    if level not in s["completed"]: s["completed"].append(level)
    if level in s["pending"]: s["pending"].remove(level)
    s["current"] = None
elif action == "fail":
    s["status"] = "failed"; s["current"] = level
s["last_updated"] = now
with open(path, "w") as f: json.dump(s, f, indent=2)
PYEOF
}

mark_done() {
    python3 - "$STATE_FILE" <<'PYEOF'
import json, sys, datetime
with open(sys.argv[1]) as f: s = json.load(f)
s["status"] = "complete"; s["current"] = None
s["last_updated"] = datetime.datetime.now().isoformat()
with open(sys.argv[1], "w") as f: json.dump(s, f, indent=2)
PYEOF
}

pending_levels() {
    python3 -c "import json; s=json.load(open('$STATE_FILE')); print(' '.join(s.get('pending',[])))"
}

kill_sim() {
    log "--- SIM TEARDOWN ---"
    local procs="simulated_robot.launch.py|benchmarks.launch.py|gz_sim|gzserver|gzclient|ign gazebo|ros_gz_bridge|parameter_bridge|rviz2|nav2|amcl|lifecycle_manager|map_server|planner_server|controller_server|bt_navigator|behavior_server|waypoint_follower|smoother_server|velocity_smoother|collision_monitor|episode_manager|metrics_compiler|control_metric|trajectory_metric|localization_metrics|ground_truth_publisher|gz_set_pose|navlearn_benchmarks|navlearn_localization_eval|joy_node|joy_teleop|joystick_relay|twist_mux|twist_marker|twist_relay|scan_sanitizer|noisy_controller|safety_stop|robot_state_publisher|controller_manager|spawner"
    log "SIGTERM..."
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
        ps aux | grep -E "$procs" | grep -v grep
        die "Sim teardown incomplete."
    fi
    log "Sim teardown CLEAN."
    sleep 10  # 10s cooldown: let OS release ports/sockets before relaunch
}

start_sim() {
    log "--- SIM LAUNCH (Phase 2b config) ---"
    ros2 launch bumperbot_bringup simulated_robot.launch.py \
        world_name:=small_house \
        nav2_profile:=$PROFILE \
        amcl_config:="$AMCL_CONFIG" \
        > "$SIM_LOG" 2>&1 &
    SIM_PID=$!
    log "Sim PID=$SIM_PID — warming up ${SIM_WARMUP}s..."
    sleep "$SIM_WARMUP"
}

verify_stack() {
    log "--- STACK VERIFICATION ---"

    # Phase 2b verification: sigma_hit=0.1 AND recovery_alpha_fast=0.1
    log "Checking AMCL sigma_hit..."
    local val
    for i in $(seq 1 $AMCL_RETRIES); do
        val=$(ros2 param get /amcl sigma_hit 2>/dev/null | grep -oP '[\d.]+' | head -1 || true)
        [[ "$val" == "0.1" ]] && { log "  AMCL: sigma_hit=0.1 ✓"; break; }
        log "  Waiting for AMCL ($i/$AMCL_RETRIES, got='$val')..."
        sleep 3
        [[ $i -eq $AMCL_RETRIES ]] && die "AMCL sigma_hit never reached 0.1"
    done

    log "Checking AMCL recovery_alpha_fast..."
    val=$(ros2 param get /amcl recovery_alpha_fast 2>/dev/null | grep -oP '[\d.]+' | head -1 || true)
    [[ "$val" == "0.1" ]] && log "  AMCL: recovery_alpha_fast=0.1 ✓" || die "Phase 2b expects recovery_alpha_fast=0.1, got '$val'"

    for node in map_server bt_navigator; do
        log "Checking $node node..."
        for i in $(seq 1 $TOPIC_RETRIES); do
            ros2 node list 2>/dev/null | grep -q "$node" && { log "  $node: active ✓"; break; }
            sleep 3
            [[ $i -eq $TOPIC_RETRIES ]] && die "$node never appeared"
        done
    done

    for topic in /scan /amcl_pose; do
        log "Checking $topic..."
        for i in $(seq 1 $TOPIC_RETRIES); do
            ros2 topic list 2>/dev/null | grep -q "^${topic}$" && { log "  $topic: present ✓"; break; }
            sleep 3
            [[ $i -eq $TOPIC_RETRIES ]] && die "$topic never appeared"
        done
    done

    log "Stack verification PASSED."
}

run_level() {
    local level="$1"
    local out_dir="$WS/results/phase2b_ttr/${level}"
    mkdir -p "$out_dir"

    log "Running TTR $level → $out_dir (SYNCHRONOUS)"
    python3 "$WS/src/navlearn_benchmarks/scripts/multi_run_harness.py" \
        --episodes $EPISODES --goals $GOALS --seed $SEED --profile $PROFILE \
        --output-dir "$out_dir" \
        --extra-arg bad_init_test:=false \
        --extra-arg kidnap_enabled:=true \
        --extra-arg perturbation_level:=$level \
        --extra-arg goal_min_distance_m:=4.0 \
        --extra-arg ttc_timeout_sec:=15.0 \
        --extra-arg ttr_timeout_sec:=15.0 \
        2>&1 | tee "/tmp/phase2b_ttr_${level}.log"

    log "Harness exited. Validating CSVs..."
    validate_csvs "$level" "$out_dir"
}

validate_csvs() {
    local level="$1" out_dir="$2"
    local bad=0
    for csv_file in "$out_dir"/*.csv; do
        [[ -f "$csv_file" ]] || continue
        local n_goals
        n_goals=$(python3 -c "
import csv
rows = list(csv.DictReader(open('$csv_file')))
print(sum(1 for r in rows if r.get('Metric Name','') == 'TTR Outcome'))
" 2>/dev/null || echo 0)
        if [[ "$n_goals" -lt "$GOALS" ]]; then
            log "  PARTIAL: $(basename $csv_file) has $n_goals/$GOALS — deleting."
            rm -f "$csv_file"
            bad=$((bad+1))
        else
            log "  OK: $(basename $csv_file) — $n_goals/$GOALS ✓"
        fi
    done

    if [[ "$bad" -gt 0 ]]; then
        log "Filling $bad missing run(s) with seed +100..."
        python3 "$WS/src/navlearn_benchmarks/scripts/multi_run_harness.py" \
            --episodes "$bad" --goals $GOALS --seed $((SEED+100)) --profile $PROFILE \
            --output-dir "$out_dir" \
            --extra-arg bad_init_test:=false \
            --extra-arg kidnap_enabled:=true \
            --extra-arg perturbation_level:=$level \
            --extra-arg goal_min_distance_m:=4.0 \
            --extra-arg ttc_timeout_sec:=15.0 \
            --extra-arg ttr_timeout_sec:=15.0 \
            2>&1 | tee -a "/tmp/phase2b_ttr_${level}.log"
        validate_csvs "$level" "$out_dir"
        return
    fi

    local final; final=$(ls "$out_dir"/*.csv 2>/dev/null | wc -l)
    log "TTR $level VALIDATED: $final complete runs."
    echo "  [$(date '+%Y-%m-%d %H:%M')] phase2b ttr $level: $final runs → $out_dir" >> "$WS/SESSION_LOG.md"
}

main() {
    log "=== Phase 2b Orchestrator (TTR-only ablation) ==="
    log "Config: $AMCL_CONFIG"
    init_state

    local levels; levels=$(pending_levels)
    [[ -z "$levels" ]] && { log "All TTR levels done."; mark_done; exit 0; }
    log "Pending: $levels"

    for entry in $levels; do
        local level="${entry#*_}"
        log "====== START: $entry ======"
        update_state "$entry" start

        kill_sim
        start_sim
        verify_stack

        run_level "$level"

        update_state "$entry" complete
        log "====== DONE: $entry ======"
    done

    kill_sim
    mark_done

    log "=== Phase 2b complete (4 TTR levels). ==="
    log "Next: python3 src/navlearn_benchmarks/scripts/analyze_ttc_ttr.py \\"
    log "  --phase1-dir results/phase1 --phase2-dir results/phase2b \\"
    log "  --output-dir results/phase2b_analysis"
}

main "$@"
