#!/usr/bin/env bash
# Independent watchdog for an unattended campaign.
#
# Usage: nohup bash campaign_watchdog.sh <campaign_dir> <process_pattern> &
#   e.g. nohup bash campaign_watchdog.sh results/leg2_final leg2_ttr_campaign &
#
# Why it exists, and why it is a separate detached process
#   On 2026-08-01 a campaign hung and nobody noticed for 8 h 14 m, because the only thing
#   watching was a completion notification -- and a hang produces no completion. The harness
#   watchdog added since then bounds a hung *episode*, but it cannot report on the campaign
#   dying, being killed, or the machine going quiet, because it would die with it.
#
#   Anything that watches from inside the same process tree as the observer has the same
#   problem one level up. This runs detached, survives the terminal and the agent session
#   that started it, writes to a file, and raises a desktop notification -- so the alert
#   arrives even if every tool that launched the run has been closed.
#
#   Coverage rule: it must emit on EVERY terminal state, not just the good one. Silence
#   from a watchdog is indistinguishable from silence from a hang.
set -uo pipefail

CAMPAIGN_DIR="${1:?usage: campaign_watchdog.sh <campaign_dir> <process_pattern>}"
PATTERN="${2:?usage: campaign_watchdog.sh <campaign_dir> <process_pattern>}"

# The pattern appears verbatim in this script's own command line, so a plain `pgrep -f`
# matches the watchdog itself and the campaign reads as running forever -- observed on
# 2026-08-02, when the leg 2 watchdog outlived the campaign it was watching and never said
# FINISHED. Bracketing the last character makes the regex match the campaign's command
# line but not the literal pattern carried in our own.
PATTERN="${PATTERN%?}[${PATTERN: -1}]"
STALL_S="${STALL_S:-900}"      # node output silent this long = something is wrong
POLL_S="${POLL_S:-120}"

ALERTS="$CAMPAIGN_DIR/watchdog.log"
mkdir -p "$CAMPAIGN_DIR"

alert() {
    printf '[%s] %s\n' "$(date '+%F %T')" "$*" >> "$ALERTS"
    # Best-effort desktop notification. A watchdog that dies because the session has no
    # display would defeat its own purpose, so every failure here is ignored.
    DISPLAY="${DISPLAY:-:0}" notify-send -u critical "NavLearn campaign" "$*" 2>/dev/null || true
}

alert "watchdog armed on $CAMPAIGN_DIR (pattern '$PATTERN', stall ${STALL_S}s, pid $$)"

last_crash_count=0
while true; do
    sleep "$POLL_S"

    if ! pgrep -f "$PATTERN" >/dev/null 2>&1; then
        if grep -q "campaign complete" "$CAMPAIGN_DIR/campaign.log" 2>/dev/null; then
            alert "FINISHED: campaign completed and exited cleanly"
        else
            alert "DIED: campaign process is gone and never printed 'campaign complete'"
        fi
        break
    fi

    # Node stdout is the one signal that is continuous while anything is really running.
    # The campaign log only moves at arm boundaries, which is hours apart.
    newest=$(ls -t "$CAMPAIGN_DIR"_*/navlearn_nodes_run_*.log 2>/dev/null | head -1)
    if [ -n "$newest" ]; then
        age=$(( $(date +%s) - $(stat -c %Y "$newest") ))
        if [ "$age" -gt "$STALL_S" ]; then
            alert "STALLED: no node output for ${age}s (newest: $(basename "$newest"))"
        fi
    fi

    crashes=$(ls "$CAMPAIGN_DIR"_*/forensics_run_*/*.crash 2>/dev/null | wc -l)
    if [ "$crashes" -gt "$last_crash_count" ]; then
        alert "CRASH: $crashes crash report(s) now present -- a node died during a run"
        last_crash_count=$crashes
    fi

    aborts=$(grep -c "ABORTED BY WATCHDOG" "$CAMPAIGN_DIR"_*/harness.log 2>/dev/null \
             | awk -F: '{s+=$NF} END {print s+0}')
    if [ "${aborts:-0}" -gt 0 ]; then
        alert "ABORTS: $aborts episode(s) killed by the harness watchdog so far"
    fi
done

alert "watchdog exiting"
