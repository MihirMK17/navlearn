#!/usr/bin/env bash
# Phase 3 NavLearn Orchestrator — MPPI Controller Benchmarks
# Runs 2 MPPI profiles × 4 perturbation levels × TTC + TTR = 16 levels.
# Mirrors phase2_orchestrator.sh: single bringup + synchronous harness + graceful teardown.
# Usage: bash phase3_orchestrator.sh [--dry-run] [--profile mppi_baseline|mppi_aggressive|all]

export ROS_DOMAIN_ID=1
source "$HOME/robot_ws/install/setup.bash" || true

set -eo pipefail

WS="$HOME/robot_ws"
STATE_FILE="$WS/results/phase3_state.json"
AMCL_CONFIG="$WS/install/bumperbot_localization/share/bumperbot_localization/config/amcl_phase2.yaml"
EPISODES=${EPISODES:-5}
GOALS=${GOALS:-5}
SEED=${SEED:-42}
SIM_LOG="/tmp/phase3_sim.log"
KILL_GRACE=8
SIM_WARMUP=50
AMCL_RETRIES=30   # ×3s = 90s max
TOPIC_RETRIES=20  # ×3s = 60s max

DRY_RUN=false
PROFILE_FILTER="all"  # all | mppi_baseline | mppi_aggressive

# Parse args
while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run) DRY_RUN=true; shift ;;
        --profile) PROFILE_FILTER="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

# All 16 levels: {nav2_profile}_{mode}_{level}
ALL_LEVELS=(
    mppi_baseline_ttc_easy   mppi_baseline_ttc_medium   mppi_baseline_ttc_hard   mppi_baseline_ttc_extreme
    mppi_baseline_ttr_easy   mppi_baseline_ttr_medium   mppi_baseline_ttr_hard   mppi_baseline_ttr_extreme
    mppi_aggressive_ttc_easy mppi_aggressive_ttc_medium mppi_aggressive_ttc_hard mppi_aggressive_ttc_extreme
    mppi_aggressive_ttr_easy mppi_aggressive_ttr_medium mppi_aggressive_ttr_hard mppi_aggressive_ttr_extreme
)

log()  { echo "[$(date '+%H:%M:%S')] $*"; }
die()  { log "FATAL: $*"; exit 1; }

# ── Dry-run ────────────────────────────────────────────────────────────────────

dry_run_summary() {
    echo "=== Phase 3 MPPI Dry-Run — Run Matrix ==="
    echo "AMCL config : $AMCL_CONFIG"
    echo "Episodes    : $EPISODES × $GOALS goals, seed=$SEED"
    echo "Profile     : $PROFILE_FILTER"
    echo ""
    local count=0
    for entry in "${ALL_LEVELS[@]}"; do
        [[ "$PROFILE_FILTER" != "all" && "$entry" != ${PROFILE_FILTER}_* ]] && continue
        local nav2_profile mode level
        nav2_profile="${entry%%_ttc_*}"; nav2_profile="${nav2_profile%%_ttr_*}"
        mode="${entry#${nav2_profile}_}"; mode="${mode%%_*}"
        level="${entry##*_${mode}_}"
        echo "  [$((++count))] nav2_profile=$nav2_profile  mode=$mode  level=$level"
        echo "       → results/phase3_${nav2_profile}_${mode}/${level}/"
    done
    echo ""
    echo "Total: $count levels × $EPISODES episodes × $GOALS goals = $((count * EPISODES * GOALS)) goal runs"
    echo "(Dry-run complete — no sim launched.)"
}

[[ "$DRY_RUN" == "true" ]] && { dry_run_summary; exit 0; }

# ── State ──────────────────────────────────────────────────────────────────────

init_state() {
    mkdir -p "$WS/results"
    if [[ ! -f "$STATE_FILE" ]]; then
        local pending_json="["
        local first=true
        for entry in "${ALL_LEVELS[@]}"; do
            [[ "$PROFILE_FILTER" != "all" && "$entry" != ${PROFILE_FILTER}_* ]] && continue
            $first || pending_json+=","
            pending_json+="\"$entry\""
            first=false
        done
        pending_json+="]"
        python3 -c "
import json, datetime
s = {
    'status': 'running',
    'profile_filter': '$PROFILE_FILTER',
    'completed': [],
    'pending': $pending_json,
    'current': None,
    'started_at': datetime.datetime.now().isoformat(),
    'last_updated': datetime.datetime.now().isoformat()
}
open('$STATE_FILE','w').write(json.dumps(s, indent=2))
print('Phase 3 state initialized.')
"
    fi
}

update_state() {
    python3 - "$STATE_FILE" "$1" "$2" <<'PYEOF'
import json, sys, datetime
path, entry, action = sys.argv[1:]
with open(path) as f: s = json.load(f)
now = datetime.datetime.now().isoformat()
if action == "start":
    s["current"] = entry; s["started_at"] = now
elif action == "complete":
    if entry not in s["completed"]: s["completed"].append(entry)
    if entry in s["pending"]: s["pending"].remove(entry)
    s["current"] = None
elif action == "fail":
    s["status"] = "failed"; s["current"] = entry
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
    python3 -c "
import json
s = json.load(open('$STATE_FILE'))
pending = s.get('pending', [])
if '$PROFILE_FILTER' != 'all':
    pending = [e for e in pending if e.startswith('${PROFILE_FILTER}_')]
print(' '.join(pending))
"
}

# ── Sim teardown (identical to phase2_orchestrator.sh) ────────────────────────

kill_sim() {
    log "--- SIM TEARDOWN ---"
    local procs="simulated_robot.launch.py|benchmarks.launch.py|gz_sim|gzserver|gzclient|ign gazebo|ros_gz_bridge|parameter_bridge|rviz2|nav2|amcl|lifecycle_manager|map_server|planner_server|controller_server|bt_navigator|behavior_server|waypoint_follower|smoother_server|velocity_smoother|collision_monitor|episode_manager|metrics_compiler|control_metric|trajectory_metric|localization_metrics|ground_truth_publisher|gz_set_pose|navlearn_benchmarks|navlearn_localization_eval|joy_node|joy_teleop|joystick_relay|twist_mux|twist_marker|twist_relay|scan_sanitizer|noisy_controller|safety_stop|robot_state_publisher|controller_manager|spawner"
    # Collect PIDs of this orchestrator and its entire process group to avoid self-kill
    local self_pgid
    self_pgid=$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ') || self_pgid=$$
    _safe_pids() {
        ps aux | grep -E "$procs" | grep -v grep | awk '{print $2}' \
            | while read -r pid; do
                local pg; pg=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ') || continue
                [[ "$pg" != "$self_pgid" ]] && echo "$pid"
              done
    }
    log "SIGTERM to ROS/Gazebo processes (pgid ${self_pgid} excluded)..."
    { _safe_pids | xargs -r kill -TERM; } 2>/dev/null || true
    sleep "$KILL_GRACE"
    local survivors
    survivors=$(_safe_pids || true)
    if [[ -n "$survivors" ]]; then
        log "SIGKILL survivors: $survivors"
        echo "$survivors" | xargs -r kill -KILL 2>/dev/null || true
        sleep 4
    fi
    local remaining=0
    remaining=$(_safe_pids | wc -l 2>/dev/null) || remaining=0
    remaining=$(echo "$remaining" | tr -d ' ')
    if [[ "${remaining:-0}" -gt 0 ]]; then
        log "WARNING: $remaining processes still alive after SIGKILL:"
        _safe_pids
        die "Sim teardown incomplete — cannot start next run."
    fi
    log "Sim teardown CLEAN."
    sleep 2
}

# ── Sim launch + stack verification ───────────────────────────────────────────

start_sim() {
    local nav2_profile="$1"
    log "--- SIM LAUNCH (nav2_profile=$nav2_profile) ---"
    ros2 launch bumperbot_bringup simulated_robot.launch.py \
        world_name:=small_house \
        nav2_profile:="$nav2_profile" \
        amcl_config:="$AMCL_CONFIG" \
        > "$SIM_LOG" 2>&1 &
    SIM_PID=$!
    log "Sim PID=$SIM_PID — warming up ${SIM_WARMUP}s..."
    sleep "$SIM_WARMUP"
}

verify_stack() {
    log "--- STACK VERIFICATION ---"
    local val
    for i in $(seq 1 $AMCL_RETRIES); do
        val=$(ros2 param get /amcl sigma_hit 2>/dev/null | grep -oP '[\d.]+' | head -1 || true)
        [[ "$val" == "0.1" ]] && { log "  AMCL: sigma_hit=0.1 ✓"; break; }
        log "  Waiting for AMCL ($i/$AMCL_RETRIES, got='$val')..."
        sleep 3
        [[ $i -eq $AMCL_RETRIES ]] && die "AMCL sigma_hit never reached 0.1"
    done
    for i in $(seq 1 $TOPIC_RETRIES); do
        ros2 node list 2>/dev/null | grep -q "/map_server" && { log "  map_server: active ✓"; break; }
        log "  Waiting for map_server ($i/$TOPIC_RETRIES)..."
        sleep 3; [[ $i -eq $TOPIC_RETRIES ]] && die "map_server never appeared"
    done
    for i in $(seq 1 $TOPIC_RETRIES); do
        ros2 topic list 2>/dev/null | grep -q "^/scan$" && { log "  /scan: present ✓"; break; }
        log "  Waiting for /scan ($i/$TOPIC_RETRIES)..."
        sleep 3; [[ $i -eq $TOPIC_RETRIES ]] && die "/scan never appeared"
    done
    for i in $(seq 1 $TOPIC_RETRIES); do
        ros2 topic list 2>/dev/null | grep -q "^/amcl_pose$" && { log "  /amcl_pose: present ✓"; break; }
        log "  Waiting for /amcl_pose ($i/$TOPIC_RETRIES)..."
        sleep 3; [[ $i -eq $TOPIC_RETRIES ]] && die "/amcl_pose never appeared"
    done
    for i in $(seq 1 $TOPIC_RETRIES); do
        ros2 node list 2>/dev/null | grep -q "bt_navigator" && { log "  bt_navigator: active ✓"; break; }
        log "  Waiting for bt_navigator ($i/$TOPIC_RETRIES)..."
        sleep 3; [[ $i -eq $TOPIC_RETRIES ]] && die "bt_navigator never appeared"
    done
    log "Stack verification PASSED."
}

# ── Harness + CSV validation ───────────────────────────────────────────────────

run_level() {
    local nav2_profile="$1" mode="$2" level="$3"
    local out_dir="$WS/results/phase3_${nav2_profile}_${mode}/${level}"
    local bad_init kidnap
    [[ "$mode" == "ttc" ]] && { bad_init="true"; kidnap="false"; } || { bad_init="false"; kidnap="true"; }

    mkdir -p "$out_dir"
    log "Running $nav2_profile $mode $level → $out_dir"

    python3 "$WS/src/navlearn_benchmarks/scripts/multi_run_harness.py" \
        --episodes $EPISODES --goals $GOALS --seed $SEED --profile "$nav2_profile" \
        --output-dir "$out_dir" \
        --extra-arg bad_init_test:=$bad_init \
        --extra-arg kidnap_enabled:=$kidnap \
        --extra-arg perturbation_level:=$level \
        --extra-arg goal_min_distance_m:=4.0 \
        --extra-arg ttc_timeout_sec:=15.0 \
        --extra-arg ttr_timeout_sec:=15.0 \
        2>&1 | tee "/tmp/phase3_${nav2_profile}_${mode}_${level}.log"

    log "Harness exited. Validating CSVs..."
    validate_csvs "$nav2_profile" "$mode" "$level" "$out_dir" "$bad_init" "$kidnap"
}

validate_csvs() {
    local nav2_profile="$1" mode="$2" level="$3" out_dir="$4" bad_init="$5" kidnap="$6"
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
            log "  PARTIAL: $(basename "$csv_file") has $n_goals/$GOALS goals — deleting."
            rm -f "$csv_file"; bad=$((bad+1))
        else
            log "  OK: $(basename "$csv_file") — $n_goals/$GOALS goals ✓"
        fi
    done
    if [[ "$bad" -gt 0 ]]; then
        log "Filling $bad missing run(s) with seed offset +100..."
        python3 "$WS/src/navlearn_benchmarks/scripts/multi_run_harness.py" \
            --episodes "$bad" --goals $GOALS --seed $((SEED+100)) --profile "$nav2_profile" \
            --output-dir "$out_dir" \
            --extra-arg bad_init_test:=$bad_init \
            --extra-arg kidnap_enabled:=$kidnap \
            --extra-arg perturbation_level:=$level \
            --extra-arg goal_min_distance_m:=4.0 \
            --extra-arg ttc_timeout_sec:=15.0 \
            --extra-arg ttr_timeout_sec:=15.0 \
            2>&1 | tee -a "/tmp/phase3_${nav2_profile}_${mode}_${level}.log"
        validate_csvs "$nav2_profile" "$mode" "$level" "$out_dir" "$bad_init" "$kidnap"
        return
    fi
    local final; final=$(ls "$out_dir"/*.csv 2>/dev/null | wc -l)
    log "$nav2_profile $mode $level VALIDATED: $final complete runs."
    echo "  [$(date '+%Y-%m-%d %H:%M')] phase3 $nav2_profile $mode $level: $final runs → $out_dir" >> "$WS/SESSION_LOG.md" 2>/dev/null || true
}

# ── Main ───────────────────────────────────────────────────────────────────────

main() {
    log "=== Phase 3 MPPI Orchestrator ==="
    log "Profile filter: $PROFILE_FILTER | Episodes: $EPISODES | Goals: $GOALS | Seed: $SEED"
    init_state

    local levels; levels=$(pending_levels)
    [[ -z "$levels" ]] && { log "All levels done."; mark_done; exit 0; }
    log "Pending: $levels"

    local prev_nav2_profile=""
    for entry in $levels; do
        # Parse: mppi_baseline_ttc_easy → nav2_profile=mppi_baseline, mode=ttc, level=easy
        local nav2_profile mode level
        if [[ "$entry" == mppi_aggressive_* ]]; then
            nav2_profile="mppi_aggressive"
        else
            nav2_profile="mppi_baseline"
        fi
        local rest="${entry#${nav2_profile}_}"
        mode="${rest%%_*}"
        level="${rest#${mode}_}"

        log "====== START: $entry ($nav2_profile / $mode / $level) ======"
        update_state "$entry" start

        kill_sim

        # Only relaunch sim if nav2_profile changed (avoids re-warmup for same profile)
        if [[ "$nav2_profile" != "$prev_nav2_profile" ]]; then
            start_sim "$nav2_profile"
            verify_stack
            prev_nav2_profile="$nav2_profile"
        else
            log "Same nav2_profile ($nav2_profile) — skipping re-launch, re-verifying stack..."
            verify_stack
        fi

        run_level "$nav2_profile" "$mode" "$level"
        update_state "$entry" complete
        log "====== DONE: $entry ======"
    done

    kill_sim
    mark_done

    log "=== All Phase 3 levels complete. Running analysis... ==="
    python3 "$WS/src/navlearn_benchmarks/scripts/analyze_ttc_ttr.py" \
        --phase1-dir "$WS/results/phase1" \
        --phase2-dir "$WS/results/phase2" \
        --phase3-dir "$WS/results/phase3_ttc_mppi_aggressive" \
        --output-dir "$WS/results/phase3_analysis" \
        2>&1 | tee "/tmp/phase3_analysis.log" || log "WARNING: analyze_ttc_ttr.py failed (may need --phase3-dir support)"

    log "=== Phase 3 complete. Results in $WS/results/phase3_*. ==="
}

main "$@"
