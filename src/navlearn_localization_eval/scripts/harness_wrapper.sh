#!/usr/bin/env bash
# harness_wrapper.sh — Resilient wrapper for multi_run_harness.py
# Adds: known-failure detection, auto-retry, failure log capture, 30% hard-stop threshold.
# Usage: harness_wrapper.sh <run_id> <harness_args...>
# Example: harness_wrapper.sh run_001 --episodes 5 --goals 5 --seed 42 ...

set -eo pipefail

WS="${WS:-$HOME/robot_ws}"
HARNESS="$WS/src/navlearn_benchmarks/scripts/multi_run_harness.py"
FAILURE_DIR="$WS/results/failures"
INGEST_SCRIPT="$HOME/.claude/brain/scripts/ingest_harness_failure.py"
FAILURE_THRESHOLD=30   # % of total runs; hard-stop if exceeded
MAX_RETRIES=1          # retry count per run on known-failure match

RUN_ID="${1:?Usage: harness_wrapper.sh <run_id> <harness_args...>}"
shift
HARNESS_ARGS=("$@")

mkdir -p "$FAILURE_DIR"

LOG_FILE="$FAILURE_DIR/${RUN_ID}.log"
STDERR_FILE="$FAILURE_DIR/${RUN_ID}.stderr"
FAILURE_SUMMARY="$FAILURE_DIR/summary.jsonl"

log() { echo "[$(date '+%H:%M:%S')] [wrapper:$RUN_ID] $*"; }

# Known failure patterns → mitigation commands
declare -A FAILURE_MITIGATIONS
FAILURE_MITIGATIONS=(
    ["AMCL_SIGMA_TIMEOUT"]="pkill -9 -f amcl 2>/dev/null; sleep 5"
    ["SCAN_MISSING"]="pkill -9 -f gz_sim 2>/dev/null; sleep 8"
    ["CSV_MISMATCH"]="sleep 3"
    ["ZOMBIE_PROCESS"]="pkill -9 -f gz_sim 2>/dev/null; pkill -9 -f ros2 2>/dev/null; sleep 8"
    ["TOPIC_LATENCY"]="sleep 10"
)

classify_failure() {
    local stderr_tail="$1"
    if echo "$stderr_tail" | grep -q "sigma_hit never reached\|AMCL.*timeout\|sigma_hit.*retries"; then
        echo "AMCL_SIGMA_TIMEOUT"
    elif echo "$stderr_tail" | grep -q "/scan.*never appeared\|scan.*not.*published\|lidar.*missing"; then
        echo "SCAN_MISSING"
    elif echo "$stderr_tail" | grep -q "PARTIAL.*goals\|goal.*mismatch\|fewer goals"; then
        echo "CSV_MISMATCH"
    elif echo "$stderr_tail" | grep -q "still alive after SIGKILL\|zombie\|process.*remaining"; then
        echo "ZOMBIE_PROCESS"
    elif echo "$stderr_tail" | grep -q "latency\|timeout.*topic\|waiting.*topic"; then
        echo "TOPIC_LATENCY"
    else
        echo "UNKNOWN"
    fi
}

run_harness() {
    log "Running harness (attempt $1/$((MAX_RETRIES+1))): python3 $HARNESS ${HARNESS_ARGS[*]}"
    python3 "$HARNESS" "${HARNESS_ARGS[@]}" \
        > >(tee -a "$LOG_FILE") \
        2> >(tee -a "$STDERR_FILE" >&2)
}

# Track cumulative failure rate across wrapper invocations via summary file
check_failure_threshold() {
    local total failures rate
    total=$(grep -c '"status"' "$FAILURE_SUMMARY" 2>/dev/null || echo 0)
    failures=$(grep -c '"status": "failed"' "$FAILURE_SUMMARY" 2>/dev/null || echo 0)
    [[ "$total" -eq 0 ]] && return 0
    rate=$(( (failures * 100) / total ))
    if [[ "$rate" -ge "$FAILURE_THRESHOLD" ]]; then
        log "HARD STOP: failure rate $rate% >= threshold $FAILURE_THRESHOLD% ($failures/$total runs failed)"
        exit 2
    fi
}

record_outcome() {
    local status="$1" failure_class="$2"
    python3 -c "
import json, datetime
entry = {
    'run_id': '$RUN_ID',
    'status': '$status',
    'failure_class': '$failure_class',
    'timestamp': datetime.datetime.now().isoformat(),
    'log': '$LOG_FILE'
}
with open('$FAILURE_SUMMARY', 'a') as f:
    f.write(json.dumps(entry) + '\n')
" 2>/dev/null || true
}

main() {
    check_failure_threshold

    # First attempt
    if run_harness 1; then
        log "SUCCESS on attempt 1"
        record_outcome "success" "none"
        return 0
    fi

    EXIT_CODE=$?
    log "Harness exited with code $EXIT_CODE — analyzing failure..."

    # Classify from last 200 lines of stderr
    local stderr_tail failure_class
    stderr_tail=$(tail -200 "$STDERR_FILE" 2>/dev/null || echo "")
    failure_class=$(classify_failure "$stderr_tail")
    log "Failure class: $failure_class"

    # Ingest to errors_seen.md
    if [[ -f "$INGEST_SCRIPT" ]]; then
        python3 "$INGEST_SCRIPT" \
            --run-id "$RUN_ID" \
            --failure-class "$failure_class" \
            --log-file "$STDERR_FILE" \
            --harness-args "${HARNESS_ARGS[*]}" \
            2>/dev/null || true
    fi

    if [[ "$failure_class" == "UNKNOWN" ]]; then
        log "Unknown failure — skipping retry, recording as failed."
        record_outcome "failed" "$failure_class"
        return "$EXIT_CODE"
    fi

    # Known failure: run mitigation then retry once
    local mitigation="${FAILURE_MITIGATIONS[$failure_class]}"
    log "Running mitigation for $failure_class: $mitigation"
    eval "$mitigation" 2>/dev/null || true

    if run_harness 2; then
        log "SUCCESS on retry after $failure_class mitigation"
        record_outcome "success_after_retry" "$failure_class"
        return 0
    fi

    log "FAILED on retry too — recording as failed"
    record_outcome "failed" "$failure_class"
    check_failure_threshold
    return "$EXIT_CODE"
}

main
