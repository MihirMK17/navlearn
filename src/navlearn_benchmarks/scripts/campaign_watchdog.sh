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

# The pattern is passed as an argument, so it sits verbatim in this script's own argv and
# `pgrep -f` matches the watchdog itself -- the campaign then reads as running forever.
#
# Bracketing the pattern does NOT fix this, which is the trap: the bracketed regex
# "leg2_ttr_campaig[n]" still matches the raw "leg2_ttr_campaign" carried in our own
# command line. Observed 2026-08-02: the leg 2 watchdog outlived a campaign that had
# finished 15 minutes earlier and reported STALLED instead of FINISHED.
#
# The only reliable fix is identity, not pattern shape: ignore our own PID and our own
# children (the transient subshells pgrep itself spawns).
#
# Ancestors are excluded for the same reason as self: the shell that launched this
# watchdog necessarily typed the pattern on its own command line, so it matches too.
# Descendants are excluded because the command substitution below forks a copy of this
# shell, inheriting our argv.
_ancestors() {
    local p=$$ ppid
    while [ -n "$p" ] && [ "$p" != "1" ] && [ "$p" != "0" ]; do
        printf '%s ' "$p"
        ppid=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
        [ "$ppid" = "$p" ] && break
        p=$ppid
    done
}

# Skip ourselves and our ancestors BY PID, and skip our own direct forks (the command
# substitution below clones this shell, argv included).
#
# Only ppid == $$ is skipped, never "ppid is an ancestor": the campaign is normally a
# SIBLING of this watchdog, launched by the same shell, so excluding everything parented
# by an ancestor would skip the one process we exist to watch. That mistake made the
# watchdog report a running campaign as gone within four seconds of arming.
_campaign_alive() {
    local p ppid skip cmd
    skip=" $(_ancestors) "
    for p in $(pgrep -f "$PATTERN" 2>/dev/null); do
        # Ourselves and the shell that launched us: both necessarily carry the pattern.
        case "$skip" in *" $p "*) continue ;; esac

        # Re-read the candidate from /proc and require it to still be a live process whose
        # command line still contains the pattern.
        #
        # pgrep forks to run, and that fork inherits our argv; by the time the loop body
        # executes it has already exited, leaving a PID whose ppid reads empty. Comparing
        # that empty value against $$ never matched, so the watchdog saw a phantom campaign
        # and reported a finished run as still running -- indefinitely. Validating against
        # /proc closes the race and also rejects a PID recycled onto something unrelated.
        cmd=$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null) || continue
        [ -z "$cmd" ] && continue
        case "$cmd" in *"$PATTERN"*) ;; *) continue ;; esac

        # Our own direct forks, for the same argv-inheritance reason.
        ppid=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
        [ "$ppid" = "$$" ] && continue

        return 0
    done
    return 1
}
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

    if ! _campaign_alive; then
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
