#!/usr/bin/env bash
# Phase 3 follow-up: compute profile script
# Runs RPP baseline, MPPI baseline, MPPI aggressive ONCE EACH (1 episode × 5 goals, no perturbation)
# Captures controller_server CPU% + RSS via pidstat throughout the run.
# Outputs: results/phase3_compute_profile/<profile>/pidstat.log + harness CSV.

export ROS_DOMAIN_ID=1
source "$HOME/robot_ws/install/setup.bash" || true
set -eo pipefail

WS="$HOME/robot_ws"
AMCL_CONFIG="$WS/install/bumperbot_localization/share/bumperbot_localization/config/amcl_phase2.yaml"
OUT_ROOT="$WS/results/phase3_compute_profile"
SIM_LOG="/tmp/phase3_compute_sim.log"
KILL_GRACE=8
SIM_WARMUP=50

PROFILES=(baseline mppi_baseline mppi_aggressive)

log()  { echo "[$(date '+%H:%M:%S')] $*"; }
die()  { log "FATAL: $*"; exit 1; }

kill_sim() {
    log "--- SIM TEARDOWN ---"
    local procs="simulated_robot.launch.py|benchmarks.launch.py|gz_sim|gzserver|gzclient|ign gazebo|ros_gz_bridge|parameter_bridge|rviz2|nav2|amcl|lifecycle_manager|map_server|planner_server|controller_server|bt_navigator|behavior_server|waypoint_follower|smoother_server|velocity_smoother|collision_monitor|episode_manager|metrics_compiler|control_metric|trajectory_metric|localization_metrics|ground_truth_publisher|gz_set_pose|navlearn_benchmarks|navlearn_localization_eval|joy_node|joy_teleop|joystick_relay|twist_mux|twist_marker|twist_relay|scan_sanitizer|noisy_controller|safety_stop|robot_state_publisher|controller_manager|spawner|pidstat"
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

run_profile() {
    local profile="$1"
    local out_dir="$OUT_ROOT/$profile"
    mkdir -p "$out_dir"
    log "====== START: $profile ======"

    kill_sim

    # Launch sim
    ros2 launch bumperbot_bringup simulated_robot.launch.py \
        world_name:=small_house \
        nav2_profile:="$profile" \
        amcl_config:="$AMCL_CONFIG" \
        > "$SIM_LOG" 2>&1 &
    SIM_PID=$!
    log "Sim PID=$SIM_PID — warming ${SIM_WARMUP}s..."
    sleep "$SIM_WARMUP"

    # Verify Nav2 up
    for i in $(seq 1 30); do
        ros2 node list 2>/dev/null | grep -q "bt_navigator" && break
        sleep 3
        [[ $i -eq 30 ]] && die "bt_navigator never appeared for $profile"
    done
    log "Nav2 ready."

    # Poll controller_server CPU%/MEM via ps in a background loop (1s interval)
    # Header: timestamp pid %cpu %mem rss(KB) vsz(KB)
    (
        echo "timestamp,pid,cpu_pct,mem_pct,rss_kb,vsz_kb"
        while true; do
            ts=$(date +%s.%N)
            ps -C controller_server -o pid=,%cpu=,%mem=,rss=,vsz= --no-headers 2>/dev/null | while read line; do
                echo "$ts,$line" | tr -s ' ' ',' | sed 's/,,/,/g; s/^,//'
            done
            sleep 1
        done
    ) > "$out_dir/pidstat.log" 2>&1 &
    PIDSTAT_PID=$!
    log "ps-poller PID=$PIDSTAT_PID started"

    # Run harness: 1 episode × 5 goals, no perturbation (TTC mode but easy level + bad_init=false)
    python3 "$WS/src/navlearn_benchmarks/scripts/multi_run_harness.py" \
        --episodes 1 --goals 5 --seed 42 --profile "$profile" \
        --output-dir "$out_dir" \
        --extra-arg bad_init_test:=false \
        --extra-arg kidnap_enabled:=false \
        --extra-arg perturbation_level:=easy \
        --extra-arg goal_min_distance_m:=4.0 \
        --extra-arg ttc_timeout_sec:=15.0 \
        --extra-arg ttr_timeout_sec:=15.0 \
        2>&1 | tee "$out_dir/harness.log"

    # Stop pidstat
    kill -TERM $PIDSTAT_PID 2>/dev/null || true
    sleep 2
    log "Profile $profile run complete."

    kill_sim
}

log "=== Phase 3 Compute Profile ==="
mkdir -p "$OUT_ROOT"

for profile in "${PROFILES[@]}"; do
    run_profile "$profile"
done

log "=== Phase 3 Compute Profile COMPLETE ==="
log "Results: $OUT_ROOT"
log "Summary:"
for profile in "${PROFILES[@]}"; do
    log_file="$OUT_ROOT/$profile/pidstat.log"
    if [[ -f "$log_file" ]]; then
        n=$(grep -c "controller_server" "$log_file" 2>/dev/null || echo 0)
        log "  $profile: $n pidstat samples"
    fi
done
