#!/usr/bin/env bash
# Phase 2 NavLearn Orchestrator — v3
# Graceful sim teardown + full stack verification before every harness.
# Harness runs SYNCHRONOUSLY — sim is never killed mid-run.
# CSV contents validated (goal counts) before marking any level done.

# Source ROS setup ONCE before strict mode — setup.bash has non-zero exits internally
export ROS_DOMAIN_ID=1
source "$HOME/robot_ws/install/setup.bash" || true

set -eo pipefail

WS="$HOME/robot_ws"
STATE_FILE="$WS/results/phase2_state.json"
AMCL_CONFIG="$WS/install/bumperbot_localization/share/bumperbot_localization/config/amcl_phase2.yaml"
EPISODES=5
GOALS=5
SEED=42
PROFILE="aggressive"
SIM_LOG="/tmp/phase2_sim.log"
KILL_GRACE=8    # seconds between SIGTERM and SIGKILL
SIM_WARMUP=50   # seconds after launch before polling nodes
AMCL_RETRIES=30 # × 3s = 90s max wait for sigma_hit
TOPIC_RETRIES=20 # × 3s = 60s max wait per topic

log() { echo "[$(date '+%H:%M:%S')] $*"; }
die() { log "FATAL: $*"; exit 1; }

# ── State ─────────────────────────────────────────────────────────────────────

init_state() {
    mkdir -p "$WS/results"
    [[ -f "$STATE_FILE" ]] || python3 -c "
import json, datetime
s = {'status':'running','completed':[],'pending':['ttc_easy','ttc_medium','ttc_hard','ttc_extreme','ttr_easy','ttr_medium','ttr_hard','ttr_extreme'],'current':None,'started_at':None,'last_updated':datetime.datetime.now().isoformat()}
open('$STATE_FILE','w').write(json.dumps(s,indent=2))
print('State initialized.')
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

# ── Graceful sim teardown ──────────────────────────────────────────────────────

kill_sim() {
    log "--- SIM TEARDOWN ---"

    # 1. SIGTERM to named processes — covers full bringup stack so no zombies stack up
    local procs="simulated_robot.launch.py|benchmarks.launch.py|gz_sim|gzserver|gzclient|ign gazebo|ros_gz_bridge|parameter_bridge|rviz2|nav2|amcl|lifecycle_manager|map_server|planner_server|controller_server|bt_navigator|behavior_server|waypoint_follower|smoother_server|velocity_smoother|collision_monitor|episode_manager|metrics_compiler|control_metric|trajectory_metric|localization_metrics|ground_truth_publisher|gz_set_pose|navlearn_benchmarks|navlearn_localization_eval|joy_node|joy_teleop|joystick_relay|twist_mux|twist_marker|twist_relay|scan_sanitizer|noisy_controller|safety_stop|robot_state_publisher|controller_manager|spawner"
    log "SIGTERM to ROS/Gazebo processes..."
    { ps aux | grep -E "$procs" | grep -v grep | awk '{print $2}' | xargs -r kill -TERM; } 2>/dev/null || true
    sleep "$KILL_GRACE"

    # 2. SIGKILL survivors
    local survivors
    survivors=$(ps aux | grep -E "$procs" | grep -v grep | awk '{print $2}' || true)
    if [[ -n "$survivors" ]]; then
        log "SIGKILL survivors: $survivors"
        echo "$survivors" | xargs -r kill -KILL 2>/dev/null || true
        sleep 4
    fi

    # 3. Verify clean
    local remaining=0
    remaining=$(ps aux | grep -E "$procs" | grep -v grep | wc -l 2>/dev/null) || remaining=0
    remaining=$(echo "$remaining" | tr -d ' ')
    if [[ "${remaining:-0}" -gt 0 ]]; then
        log "WARNING: $remaining processes still alive after SIGKILL:"
        ps aux | grep -E "$procs" | grep -v grep
        die "Sim teardown incomplete — cannot start next run with leftover processes."
    fi

    log "Sim teardown CLEAN — zero processes remaining."
    sleep 2  # let OS release ports/sockets
}

# ── Sim launch + full stack verification ──────────────────────────────────────

start_sim() {
    log "--- SIM LAUNCH ---"
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

    # 1. AMCL sigma_hit = 0.1 (Phase 2 config active)
    log "Checking AMCL sigma_hit..."
    local val
    for i in $(seq 1 $AMCL_RETRIES); do
        val=$(ros2 param get /amcl sigma_hit 2>/dev/null | grep -oP '[\d.]+' | head -1 || true)
        [[ "$val" == "0.1" ]] && { log "  AMCL: sigma_hit=0.1 ✓"; break; }
        log "  Waiting for AMCL ($i/$AMCL_RETRIES, got='$val')..."
        sleep 3
        [[ $i -eq $AMCL_RETRIES ]] && die "AMCL sigma_hit never reached 0.1"
    done

    # 2. map_server node alive
    log "Checking map_server node..."
    for i in $(seq 1 $TOPIC_RETRIES); do
        ros2 node list 2>/dev/null | grep -q "/map_server" && { log "  map_server: active ✓"; break; }
        log "  Waiting for map_server ($i/$TOPIC_RETRIES)..."
        sleep 3
        [[ $i -eq $TOPIC_RETRIES ]] && die "map_server never appeared"
    done

    # 3. /scan publishing (lidar) — check topic list not topic info
    log "Checking /scan topic (lidar)..."
    for i in $(seq 1 $TOPIC_RETRIES); do
        ros2 topic list 2>/dev/null | grep -q "^/scan$" && { log "  /scan: present ✓"; break; }
        log "  Waiting for /scan ($i/$TOPIC_RETRIES)..."
        sleep 3
        [[ $i -eq $TOPIC_RETRIES ]] && die "/scan never appeared"
    done

    # 4. /amcl_pose present
    log "Checking /amcl_pose..."
    for i in $(seq 1 $TOPIC_RETRIES); do
        ros2 topic list 2>/dev/null | grep -q "^/amcl_pose$" && { log "  /amcl_pose: present ✓"; break; }
        log "  Waiting for /amcl_pose ($i/$TOPIC_RETRIES)..."
        sleep 3
        [[ $i -eq $TOPIC_RETRIES ]] && die "/amcl_pose never appeared"
    done

    # 5. bt_navigator active
    log "Checking bt_navigator node..."
    for i in $(seq 1 $TOPIC_RETRIES); do
        ros2 node list 2>/dev/null | grep -q "bt_navigator" && { log "  bt_navigator: active ✓"; break; }
        log "  Waiting for bt_navigator ($i/$TOPIC_RETRIES)..."
        sleep 3
        [[ $i -eq $TOPIC_RETRIES ]] && die "bt_navigator never appeared"
    done

    log "Stack verification PASSED — map_server, lidar, amcl_pose, bt_navigator, AMCL sigma_hit all confirmed."
}

# ── Harness (synchronous) + CSV validation ────────────────────────────────────

run_level() {
    local mode="$1" level="$2"
    local out_dir="$WS/results/phase2_${mode}/${level}"
    mkdir -p "$out_dir"

    local bad_init kidnap
    [[ "$mode" == "ttc" ]] && { bad_init="true"; kidnap="false"; } || { bad_init="false"; kidnap="true"; }

    log "Running $mode $level → $out_dir (SYNCHRONOUS — sim stays alive until harness exits)"

    # Harness runs foreground — this function BLOCKS until all episodes complete
    python3 "$WS/src/navlearn_benchmarks/scripts/multi_run_harness.py" \
        --episodes $EPISODES --goals $GOALS --seed $SEED --profile $PROFILE \
        --output-dir "$out_dir" \
        --extra-arg bad_init_test:=$bad_init \
        --extra-arg kidnap_enabled:=$kidnap \
        --extra-arg perturbation_level:=$level \
        --extra-arg goal_min_distance_m:=4.0 \
        --extra-arg ttc_timeout_sec:=15.0 \
        --extra-arg ttr_timeout_sec:=15.0 \
        2>&1 | tee "/tmp/phase2_${mode}_${level}.log"

    log "Harness process exited. Validating CSV contents..."
    validate_csvs "$mode" "$level" "$out_dir" "$bad_init" "$kidnap"
}

validate_csvs() {
    local mode="$1" level="$2" out_dir="$3" bad_init="$4" kidnap="$5"
    local outcome_key; [[ "$mode" == "ttc" ]] && outcome_key="TTC Outcome" || outcome_key="TTR Outcome"
    local bad=0

    for csv_file in "$out_dir"/*.csv; do
        [[ -f "$csv_file" ]] || continue
        local n_goals
        n_goals=$(python3 -c "
import csv
rows = list(csv.DictReader(open('$csv_file')))
print(sum(1 for r in rows if r.get('Metric Name','') == '$outcome_key'))
" 2>/dev/null || echo 0)
        if [[ "$n_goals" -lt "$GOALS" ]]; then
            log "  PARTIAL: $(basename $csv_file) has $n_goals/$GOALS goals — deleting."
            rm -f "$csv_file"
            bad=$((bad+1))
        else
            log "  OK: $(basename $csv_file) — $n_goals/$GOALS goals ✓"
        fi
    done

    if [[ "$bad" -gt 0 ]]; then
        log "Filling $bad missing run(s) with seed offset +100..."
        python3 "$WS/src/navlearn_benchmarks/scripts/multi_run_harness.py" \
            --episodes "$bad" --goals $GOALS --seed $((SEED+100)) --profile $PROFILE \
            --output-dir "$out_dir" \
            --extra-arg bad_init_test:=$bad_init \
            --extra-arg kidnap_enabled:=$kidnap \
            --extra-arg perturbation_level:=$level \
            --extra-arg goal_min_distance_m:=4.0 \
            --extra-arg ttc_timeout_sec:=15.0 \
            --extra-arg ttr_timeout_sec:=15.0 \
            2>&1 | tee -a "/tmp/phase2_${mode}_${level}.log"
        # Re-validate after fill
        validate_csvs "$mode" "$level" "$out_dir" "$bad_init" "$kidnap"
        return
    fi

    local final; final=$(ls "$out_dir"/*.csv 2>/dev/null | wc -l)
    log "$mode $level VALIDATED: $final complete runs."
    echo "  [$(date '+%Y-%m-%d %H:%M')] phase2 $mode $level: $final runs → $out_dir" >> "$WS/SESSION_LOG.md"
}

# ── Main ──────────────────────────────────────────────────────────────────────

main() {
    log "=== Phase 2 Orchestrator v3 ==="
    init_state

    local levels; levels=$(pending_levels)
    [[ -z "$levels" ]] && { log "All levels done."; mark_done; exit 0; }
    log "Pending: $levels"

    for entry in $levels; do
        local mode="${entry%%_*}" level="${entry#*_}"
        log "====== START: $entry ======"
        update_state "$entry" start

        kill_sim       # graceful teardown + verify clean
        start_sim      # launch + warmup
        verify_stack   # full stack check before harness

        run_level "$mode" "$level"   # synchronous — sim lives until harness exits

        update_state "$entry" complete
        log "====== DONE: $entry ======"
    done

    kill_sim   # final teardown after last level
    mark_done

    log "=== All 8 levels complete. ==="
    log "Next: python3 src/navlearn_benchmarks/scripts/analyze_ttc_ttr.py \\"
    log "  --phase1-dir results/phase1 --phase2-dir results/phase2 \\"
    log "  --output-dir results/phase2_analysis"

    cat > "$WS/.claude/phase2_resume_prompt.md" <<RESUMEOF
Phase 2 data collection COMPLETE. All 8 levels done.
State: $STATE_FILE

Next (run in ~/robot_ws):
1. python3 src/navlearn_benchmarks/scripts/analyze_ttc_ttr.py \\
     --phase1-dir results/phase1 --phase2-dir results/phase2 \\
     --output-dir results/phase2_analysis
2. Commit results + analysis
3. PR feat/phase2-experiments → main
RESUMEOF
    log "Resume prompt → $WS/.claude/phase2_resume_prompt.md"
}

main "$@"
