#!/usr/bin/env python3
# Copyright 2026 Mihir Kulkarni
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Liveness and progress supervision for an unattended benchmark run.

Why it exists
    ``multi_run_harness`` used to call ``subprocess.run`` on the per-episode launch and
    wait for it to return. That is correct for every way a run can end except the one that
    actually happened: on 2026-08-01 ``planner_server`` segfaulted, the lifecycle manager
    reset the managed nodes, and the launch stayed up forever with nothing left to drive.
    The harness waited 8 h 14 m on a stack that no longer existed. A hang emits no event,
    so no completion notification can fire on it -- the only thing that catches it is
    something that periodically asks whether the run is still making progress.

What it watches
    Two detectors, deliberately independent, because they see different failures:

    liveness   A required stack process is gone. Decisive and fast: the segfault above was
               visible ~2 s after it happened, three minutes before the next episode
               launched into the wreckage. Aborts immediately.

    progress   The metrics CSV has not gained a goal row within the per-goal budget. This
               is the only detector that catches a stack which is alive but stuck -- a
               controller grinding against an unreachable plan, a goal whose action never
               terminates.

    The stall deadline restarts at every completed goal rather than running from the start
    of the episode, and the window before the first goal gets its own larger budget. Both
    exist to protect the slow-but-healthy case: the longest goals in the campaign are the
    hardest ones, and a watchdog that culls them biases every number downstream.
"""

import dataclasses
import logging
import os
import pathlib
import signal
import subprocess
import time

#: Exit codes. 2 and 3 already mean "no CSV" and "short CSV" in multi_run_harness, so a
#: cell runner can distinguish all four failures from the status alone.
EXIT_STALLED = 4
EXIT_STACK_DOWN = 5

#: Processes whose absence means the navigation stack is no longer able to run a goal.
#: Deliberately short: these are the nodes the harness cannot proceed without, not every
#: node a bringup starts. Adding a node that is legitimately absent in some cell -- the
#: pose server outside TTR, say -- would abort healthy cells.
REQUIRED_STACK_PROCESSES = (
    "planner_server",
    "controller_server",
    "bt_navigator",
    "amcl",
    "map_server",
)


@dataclasses.dataclass(frozen=True)
class Abort:
    """A decision to stop the cell, with the reason that will be logged and exited on."""

    code: int
    reason: str


def completed_goals(csv_path) -> int:
    """Return the number of goal rows the metrics compiler has written so far.

    A row is written for every terminated goal regardless of outcome, so this counts
    progress rather than success -- a cell full of failures is still a cell that is
    running. A missing file means the compiler has not written its header yet, which is
    startup, not a fault; the startup budget covers that window.
    """
    path = pathlib.Path(csv_path)
    try:
        text = path.read_text()
    except (OSError, UnicodeDecodeError):
        return 0
    rows = [line for line in text.splitlines() if line.strip()]
    return max(len(rows) - 1, 0)  # minus header


#: The kernel stores at most TASK_COMM_LEN-1 = 15 characters of a process name in
#: /proc/PID/comm. ``controller_server`` is 17, so it is stored as ``controller_serv`` and
#: an exact-match probe on the full name finds nothing however healthy the stack is. A
#: liveness check with that hole aborts every cell on its first poll, which is worse than
#: having no watchdog at all: it destroys campaigns rather than merely failing to save one.
COMM_MAX = 15


def _running_comms() -> set:
    """Return the set of process names currently visible in /proc.

    Read directly rather than shelled out to pgrep: pgrep silently returns no matches for
    any pattern longer than COMM_MAX, and the looser ``pgrep -f`` form matches the
    watchdog's own command line and would report every process as alive.
    """
    comms = set()
    try:
        entries = os.listdir("/proc")
    except OSError:
        return comms
    for entry in entries:
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/comm") as handle:
                comms.add(handle.read().strip())
        except OSError:
            continue  # exited between listdir and open
    return comms


def dead_processes(names, launch_pid=None) -> list:
    """Return the names of required processes that are not running."""
    missing = []

    if launch_pid is not None:
        try:
            os.kill(int(launch_pid), 0)
        except (OSError, ValueError):
            missing.append(f"bringup(pid {launch_pid})")

    if not names:
        return missing

    comms = _running_comms()
    if not comms:
        # An unusable probe must not be read as a dead stack: that would abort a healthy
        # campaign because /proc was momentarily unreadable.
        return missing

    for name in names:
        if name[:COMM_MAX] not in comms:
            missing.append(name)

    return missing


class RunWatchdog:
    """Decides, on each poll, whether the current episode is still worth waiting for."""

    def __init__(self, *, progress, liveness, goal_timeout_s, startup_timeout_s,
                 clock=time.monotonic, goals_total=None):
        """
        Args:
            progress: callable returning the number of goals completed so far.
            liveness: callable returning the names of required processes now missing.
            goal_timeout_s: budget for one goal, measured from the previous completion.
            startup_timeout_s: budget for bringup plus the first goal.
            clock: monotonic time source; injected so tests need not sleep.
            goals_total: goals expected this episode, for the abort message only.
        """
        self._progress = progress
        self._liveness = liveness
        self._goal_timeout_s = float(goal_timeout_s)
        self._startup_timeout_s = float(startup_timeout_s)
        self._clock = clock
        self._goals_total = goals_total

        self._completed = 0
        self._last_progress_t = clock()

    @property
    def completed(self) -> int:
        """Goals observed complete at the last poll."""
        return self._completed

    def poll(self):
        """Return an :class:`Abort` if the episode should be stopped, else ``None``."""
        missing = self._liveness()
        if missing:
            # Reported before the stall check so the log names the cause rather than the
            # symptom it produces four minutes later.
            return Abort(
                EXIT_STACK_DOWN,
                "navigation stack is down: {} no longer running. The launch cannot "
                "complete and every remaining goal in this cell would be rejected."
                .format(", ".join(missing)),
            )

        now = self._clock()
        completed = self._progress()
        if completed > self._completed:
            self._completed = completed
            self._last_progress_t = now
            return None

        stalled_for = now - self._last_progress_t

        if self._completed == 0:
            if stalled_for > self._startup_timeout_s:
                return Abort(
                    EXIT_STALLED,
                    "the first goal did not complete within the {:.1f} s startup budget. "
                    "Bringup, map load and the first plan all fit inside it, so this is a "
                    "stack that came up and never navigated."
                    .format(self._startup_timeout_s),
                )
            return None

        if stalled_for > self._goal_timeout_s:
            of_total = f" of {self._goals_total}" if self._goals_total else ""
            return Abort(
                EXIT_STALLED,
                "no goal completed in {:.1f} s ({} goal{} done{}). The stack is alive but "
                "is not making progress."
                .format(self._goal_timeout_s, self._completed,
                        "" if self._completed == 1 else "s", of_total),
            )

        return None


def supervise(process, watchdog, poll_interval_s=2.0, grace_s=20.0, sleep=time.sleep):
    """Wait for ``process``, aborting it if ``watchdog`` says the run is finished.

    Returns ``None`` when the run exits on its own -- the healthy path, and the one that
    must stay indistinguishable from the old ``subprocess.run`` behaviour. Returns the
    :class:`Abort` when the watchdog fires, having first killed the launch.

    ``process`` must have been started with ``start_new_session=True``. The kill is sent to
    the process group, because ``ros2 launch`` is a supervisor: signalling only its PID
    leaves the nodes running, and the next episode then launches into a dirty environment
    and fails for reasons that look nothing like the original fault.
    """
    while True:
        if process.poll() is not None:
            return None

        verdict = watchdog.poll()
        if verdict is not None:
            _kill_group(process, grace_s)
            return verdict

        sleep(poll_interval_s)


def _kill_group(process, grace_s):
    """SIGINT the launch's process group, then SIGKILL whatever is left.

    SIGINT first because that is the signal ``ros2 launch`` shuts its nodes down on; going
    straight to SIGKILL would orphan them.
    """
    try:
        pgid = os.getpgid(process.pid)
    except OSError:
        return  # already gone

    for sig, wait_s in ((signal.SIGINT, grace_s), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(pgid, sig)
        except OSError:
            return
        try:
            process.wait(timeout=wait_s)
            return
        except subprocess.TimeoutExpired:
            logging.warning(
                "Launch pid %d ignored %s after %.0f s; escalating.",
                process.pid, sig.name, wait_s,
            )
