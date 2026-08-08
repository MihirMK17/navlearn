#!/usr/bin/env bash
# Shared cell runner for campaign scripts. Source this, then call run_cell.
#
# Exists because of a twice-observed failure mode (2026-07-30, 2026-07-31): launching a
# bringup while the previous one is still dying makes the NEW one fail ~0.4 s in, across
# unrelated nodes (map_server, controller_server, both spawners), in a way that looks
# like whatever changed most recently. A fixed sleep is a guess about shutdown time;
# this waits for the actual condition — zero stale processes — with a deadline, and
# refuses to launch into a dirty environment rather than producing a run that fails
# mysteriously.
#
# Source ROS BEFORE strict mode: setup.bash references unset variables internally.

NAVLEARN_NODES="ruby gzserver planner_server controller_server amcl bt_navigator \
map_server behavior_server smoother_server trajectory_metric control_metric \
metrics_compiler lifecycle_manager robot_state_publisher parameter_bridge \
collision_monitor episode_manager_node rviz2 spawner scan_rate_governor scan_sanitizer \
twist_mux joy_teleop controller_manager velocity_smoother waypoint_follower \
ground_truth_publisher localization_metrics gz_set_pose_server ekf_node \
imu_republisher safety_stop"
# The last six were absent until 2026-08-07: the three localization-eval nodes are
# composed by benchmarks.launch.py yet were never killed by teardown nor seen by
# preflight — a surviving gz_set_pose_server holds the Ignition service the next
# cell's kidnaps depend on. ekf/imu/safety_stop come up with the bringup itself.

# The kernel keeps 15 characters of a process name in /proc/PID/comm, so pgrep -x and
# pkill -x match nothing at all for the 11 names above that are longer -- controller_server,
# robot_state_publisher, lifecycle_manager and the rest. Teardown was therefore never
# killing them by name and preflight was never seeing them: the environment could be
# reported clean with the previous cell's controller_server still holding its DDS
# endpoints. Truncating the pattern is what makes both checks match what the kernel stores.
_comm() { printf '%.15s' "$1"; }

# Say it once, keep it twice. The '###' lines are the only record of preflight verdicts,
# teardown warnings and per-cell timing, and until now they existed solely on whatever
# terminal happened to be open -- which is exactly the information missing from the
# 2026-08-01 post-mortem. NAVLEARN_CELL_LOG is set per cell by run_cell.
_say() {
    printf '%s\n' "$*"
    if [ -n "${NAVLEARN_CELL_LOG:-}" ]; then
        printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" >> "$NAVLEARN_CELL_LOG"
    fi
}

# Kill everything a bringup starts, then WAIT until it is actually gone.
navlearn_teardown() {
    local pid n deadline stale
    pid=$(pgrep -f "ros2 launch bumperbot_bringu[p]" | head -1) || true
    if [ -n "$pid" ]; then kill -INT "$pid" 2>/dev/null || true; fi
    sleep 8
    for n in $NAVLEARN_NODES; do pkill -TERM -x "$(_comm "$n")" 2>/dev/null || true; done

    # The rosbag recorder is not in NAVLEARN_NODES and cannot be: its process name is
    # "ros2", shared with every other ros2 CLI invocation, so -x would be both too broad
    # and useless. It also runs in its own session (start_new_session, so the harness can
    # signal it without signalling itself), which means killing the harness or the campaign
    # script from outside leaves it running -- subscribed to every topic, competing for DDS
    # traffic. Observed 2026-08-02: an orphan from a stopped campaign survived teardown and
    # made unrelated node tests flaky until it was found by hand. SIGINT first so a
    # recorder caught mid-run still finalises its metadata.
    pkill -INT -f "bag recor[d]" 2>/dev/null || true
    sleep 5
    for n in $NAVLEARN_NODES; do pkill -KILL -x "$(_comm "$n")" 2>/dev/null || true; done
    pkill -KILL -f "bag recor[d]" 2>/dev/null || true

    # The part a fixed sleep cannot give: confirmation. Poll until no tracked process
    # remains, up to 60 s. gzserver/ruby holding the Ignition transport port is the
    # specific stale state that kills the next bringup's lifecycle transitions.
    deadline=$((SECONDS+60))
    while [ $SECONDS -lt $deadline ]; do
        stale=0
        for n in $NAVLEARN_NODES; do
            if pgrep -x "$(_comm "$n")" >/dev/null 2>&1; then stale=1; break; fi
        done
        if pgrep -f "ros2 launch bumperbot_bringu[p]" >/dev/null 2>&1; then stale=1; fi
        if [ "$stale" -eq 0 ]; then break; fi
        sleep 2
    done
    if [ "$stale" -ne 0 ]; then
        _say "navlearn_teardown: WARNING - stale processes survived KILL:"
        for n in $NAVLEARN_NODES; do pgrep -x "$(_comm "$n")" >/dev/null 2>&1 && _say "    $n"; done
    fi
    # Settle for DDS discovery and the Ignition transport socket to actually close.
    sleep 10
    return 0
}

# Refuse to launch into a dirty environment. Cheap, and turns the mystery failure into a
# named one.
navlearn_preflight() {
    local n dirty=0
    # A surviving recorder subscribes to every topic and competes for DDS traffic; it made
    # unrelated node tests flaky on 2026-08-02 before it was found by hand.
    if pgrep -f "bag recor[d]" >/dev/null 2>&1; then
        _say "preflight: stale rosbag recorder still running"
        dirty=1
    fi
    for n in $NAVLEARN_NODES; do
        if pgrep -x "$(_comm "$n")" >/dev/null 2>&1; then
            _say "preflight: stale process '$n' still running"
            dirty=1
        fi
    done
    return $dirty
}

# run_cell <output_dir> <harness args...> -- <launch args...>
# Brings up the stack, waits for BOTH lifecycle managers, runs the harness, tears down.
# Returns 0 on success; a failed bringup is reported and skipped, never silently degraded.
run_cell() {
    local dir="$1"; shift
    local -a harness_args=() launch_args=()
    local seen_sep=0 arg rc=0
    for arg in "$@"; do
        if [ "$arg" = "--" ]; then seen_sep=1; continue; fi
        if [ $seen_sep -eq 0 ]; then harness_args+=("$arg"); else launch_args+=("$arg"); fi
    done

    rm -rf "$dir"; mkdir -p "$dir"
    NAVLEARN_CELL_LOG="$dir/cell_runner.log"

    if ! navlearn_preflight; then
        _say "### $dir: PREFLIGHT DIRTY - forcing teardown first"
        navlearn_teardown
        if ! navlearn_preflight; then
            _say "### $dir: environment still dirty after teardown; cell skipped"
            return 1
        fi
    fi

    # A segfaulting node leaves a core only if the limit allows it. The 2026-08-01
    # planner_server crash was diagnosable purely because one was written; the root cause
    # was in the core and in nothing else that was kept.
    ulimit -c unlimited 2>/dev/null || _say "### $dir: WARNING - cannot raise core limit"

    _say "### $dir: launching bringup  ($(date +%H:%M:%S))"
    # The bringup's own node logs land inside the cell rather than in the shared
    # ~/.ros/log, so they are attributable to this cell by location.
    mkdir -p "$dir/ros_logs/bringup"
    ROS_LOG_DIR="$dir/ros_logs/bringup" \
    # The world comes from NAVLEARN_WORLD rather than being fixed here. It used to be
    # hardcoded to small_house, so a cell in another world would have had to pass
    # world_name a second time and depend on ros2 launch's duplicate-argument
    # precedence to win -- an unstated assumption that, if it were ever wrong, would
    # simulate one building while localizing in another and still complete every run.
    ros2 launch bumperbot_bringup simulated_robot.launch.py \
        world_name:="${NAVLEARN_WORLD:-small_house}" headless:=true use_rviz:=false \
        "${launch_args[@]}" > "$dir/bringup.log" 2>&1 &

    local deadline=$((SECONDS+300)) n=0
    while [ $SECONDS -lt $deadline ]; do
        n=$(grep -c "Managed nodes are active" "$dir/bringup.log" 2>/dev/null) || n=0
        if [ "${n:-0}" -ge 2 ]; then break; fi
        # Fail fast on the race signature instead of waiting out the full deadline.
        if grep -q "Failed to bring up all requested nodes" "$dir/bringup.log" 2>/dev/null; then
            _say "### $dir: BRINGUP ABORTED (lifecycle failure - the startup race signature)"
            navlearn_teardown
            return 1
        fi
        sleep 4
    done
    if [ "${n:-0}" -lt 2 ]; then
        _say "### $dir: BRINGUP TIMED OUT - cell skipped, not silently degraded"
        navlearn_teardown
        return 1
    fi

    # Overrides a plugin never declares are silently ignored by ROS 2; a probe would then
    # read as "no effect" when it was never applied.
    _say "### $dir: stack active; undeclared-param warnings: $(grep -c 'not declared' "$dir/bringup.log" || true)"

    PYTHONUNBUFFERED=1 ros2 run navlearn_analysis multi_run_harness \
        "${harness_args[@]}" --output-dir "$dir" > "$dir/harness.log" 2>&1
    rc=$?

    # The harness's exit code is the campaign's only machine-readable statement about
    # whether this cell is analysable. It used to be echoed and thrown away, so a cell that
    # died halfway looked from here exactly like one that finished.
    case $rc in
        0) _say "### $dir: harness OK  ($(date +%H:%M:%S))" ;;
        2) _say "### $dir: INCOMPLETE - no metrics CSV (rc=2)" ;;
        3) _say "### $dir: INCOMPLETE - short CSV, episode did not finish (rc=3)" ;;
        4) _say "### $dir: ABORTED - watchdog: run stalled, no goal progress (rc=4)" ;;
        5) _say "### $dir: ABORTED - watchdog: navigation stack died (rc=5)" ;;
        *) _say "### $dir: INCOMPLETE - harness exit=$rc" ;;
    esac
    if [ $rc -ne 0 ]; then
        _say "### $dir: reason in $dir/harness.log; cell must be re-run"
    fi

    navlearn_teardown
    return $rc
}
