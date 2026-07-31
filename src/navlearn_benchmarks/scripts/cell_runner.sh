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
twist_mux joy_teleop controller_manager velocity_smoother waypoint_follower"

# Kill everything a bringup starts, then WAIT until it is actually gone.
navlearn_teardown() {
    local pid n deadline stale
    pid=$(pgrep -f "ros2 launch bumperbot_bringu[p]" | head -1) || true
    if [ -n "$pid" ]; then kill -INT "$pid" 2>/dev/null || true; fi
    sleep 8
    for n in $NAVLEARN_NODES; do pkill -TERM -x "$n" 2>/dev/null || true; done
    sleep 5
    for n in $NAVLEARN_NODES; do pkill -KILL -x "$n" 2>/dev/null || true; done

    # The part a fixed sleep cannot give: confirmation. Poll until no tracked process
    # remains, up to 60 s. gzserver/ruby holding the Ignition transport port is the
    # specific stale state that kills the next bringup's lifecycle transitions.
    deadline=$((SECONDS+60))
    while [ $SECONDS -lt $deadline ]; do
        stale=0
        for n in $NAVLEARN_NODES; do
            if pgrep -x "$n" >/dev/null 2>&1; then stale=1; break; fi
        done
        if pgrep -f "ros2 launch bumperbot_bringu[p]" >/dev/null 2>&1; then stale=1; fi
        if [ "$stale" -eq 0 ]; then break; fi
        sleep 2
    done
    if [ "$stale" -ne 0 ]; then
        echo "navlearn_teardown: WARNING - stale processes survived KILL:" >&2
        for n in $NAVLEARN_NODES; do pgrep -x "$n" >/dev/null 2>&1 && echo "    $n" >&2; done
    fi
    # Settle for DDS discovery and the Ignition transport socket to actually close.
    sleep 10
    return 0
}

# Refuse to launch into a dirty environment. Cheap, and turns the mystery failure into a
# named one.
navlearn_preflight() {
    local n dirty=0
    for n in $NAVLEARN_NODES; do
        if pgrep -x "$n" >/dev/null 2>&1; then
            echo "preflight: stale process '$n' still running" >&2
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
    local seen_sep=0 arg
    for arg in "$@"; do
        if [ "$arg" = "--" ]; then seen_sep=1; continue; fi
        if [ $seen_sep -eq 0 ]; then harness_args+=("$arg"); else launch_args+=("$arg"); fi
    done

    rm -rf "$dir"; mkdir -p "$dir"

    if ! navlearn_preflight; then
        echo "### $dir: PREFLIGHT DIRTY - forcing teardown first"
        navlearn_teardown
        if ! navlearn_preflight; then
            echo "### $dir: environment still dirty after teardown; cell skipped"
            return 1
        fi
    fi

    echo "### $dir: launching bringup  ($(date +%H:%M:%S))"
    ros2 launch bumperbot_bringup simulated_robot.launch.py \
        world_name:=small_house headless:=true use_rviz:=false \
        "${launch_args[@]}" > "$dir/bringup.log" 2>&1 &

    local deadline=$((SECONDS+300)) n=0
    while [ $SECONDS -lt $deadline ]; do
        n=$(grep -c "Managed nodes are active" "$dir/bringup.log" 2>/dev/null) || n=0
        if [ "${n:-0}" -ge 2 ]; then break; fi
        # Fail fast on the race signature instead of waiting out the full deadline.
        if grep -q "Failed to bring up all requested nodes" "$dir/bringup.log" 2>/dev/null; then
            echo "### $dir: BRINGUP ABORTED (lifecycle failure - the startup race signature)"
            navlearn_teardown
            return 1
        fi
        sleep 4
    done
    if [ "${n:-0}" -lt 2 ]; then
        echo "### $dir: BRINGUP TIMED OUT - cell skipped, not silently degraded"
        navlearn_teardown
        return 1
    fi

    # Overrides a plugin never declares are silently ignored by ROS 2; a probe would then
    # read as "no effect" when it was never applied.
    echo "### $dir: stack active; undeclared-param warnings: $(grep -c 'not declared' "$dir/bringup.log" || true)"

    python3 -u src/navlearn_benchmarks/scripts/multi_run_harness.py \
        "${harness_args[@]}" --output-dir "$dir" > "$dir/harness.log" 2>&1
    echo "### $dir: harness exit=$?  ($(date +%H:%M:%S))"
    navlearn_teardown
    return 0
}
