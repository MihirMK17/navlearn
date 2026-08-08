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

"""Tests for the unattended-run watchdog.

Why it exists
    On 2026-08-01 planner_server segfaulted 12 minutes into a 24-episode cell. The
    lifecycle manager tore the nav stack down, the launch the harness was waiting on never
    exited, and the harness blocked on it for 8 h 14 m. Five of seventy-two runs completed.
    The harness had no way to notice: it called subprocess.run and waited, and a hang
    produces no event for a completion notification to fire on.

    The watchdog turns that silence into a named, non-zero exit. Two independent
    detectors, because they fail at different times: a stack process dying is visible
    within seconds and is unambiguous, while a stack that is alive but no longer making
    progress -- a controller spinning against an unreachable plan, a goal that never
    terminates -- is only visible as the absence of a completed goal.

What it must get right
    Both false verdicts are expensive and neither is symmetric with the other. Aborting a
    healthy cell throws away machine time and, worse, biases the campaign toward whichever
    conditions happen to run fast: the slowest goals are exactly the hardest ones. Missing
    a hang costs a night. So the stall deadline is measured from the last observed
    progress rather than from the start of the run, the pre-first-goal window gets its own
    larger budget because bringup and first plan are not a goal, and a dead process aborts
    immediately rather than waiting out a timeout it would satisfy anyway.
"""

import importlib.util
import os
import pathlib
import signal
import subprocess
import sys
import time

import pytest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(TEST_DIR)
WATCHDOG = os.path.join(PACKAGE_DIR, "scripts", "run_watchdog.py")
HARNESS = os.path.join(PACKAGE_DIR, "scripts", "multi_run_harness.py")

#: Longest goal observed in the leg 1 campaign (results/leg1_final_*, n=360): 205.8 s.
#: A per-goal budget at or below that would abort healthy cells, and the slowest goals are
#: the hardest ones -- culling them would bias every rate in the paper.
LEG1_SLOWEST_GOAL_S = 205.8


def _load(name, path):
    """Import a module from an explicit path, as the launch files themselves do."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def wd():
    return _load("run_watchdog", WATCHDOG)


@pytest.fixture(scope="module")
def harness():
    return _load("multi_run_harness", HARNESS)


class FakeClock:
    """A monotonic clock the test advances explicitly.

    Real sleeps would make this suite take longer than the timeouts it is pinning, and a
    suite slow enough to skip is a suite that stops guarding anything.
    """

    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, seconds):
        self.t += seconds


def _make(wd, clock, progress, liveness=lambda: [], goal_timeout_s=120.0,
          startup_timeout_s=300.0):
    return wd.RunWatchdog(
        progress=progress,
        liveness=liveness,
        goal_timeout_s=goal_timeout_s,
        startup_timeout_s=startup_timeout_s,
        clock=clock,
    )


# --- progress detector -------------------------------------------------------------


def test_advancing_progress_never_aborts(wd):
    """A cell that keeps completing goals runs as long as it needs to."""
    clock = FakeClock()
    completed = [0]
    watchdog = _make(wd, clock, progress=lambda: completed[0], goal_timeout_s=120.0)

    for _ in range(10):
        completed[0] += 1
        clock.advance(119.0)
        assert watchdog.poll() is None


def test_stalled_goal_aborts_as_stalled(wd):
    """No goal completed within the per-goal budget is a hang, not a slow goal."""
    clock = FakeClock()
    watchdog = _make(wd, clock, progress=lambda: 1, goal_timeout_s=120.0)

    watchdog.poll()  # observe the first completed goal
    clock.advance(121.0)
    verdict = watchdog.poll()

    assert verdict is not None
    assert verdict.code == wd.EXIT_STALLED
    assert "120" in verdict.reason
    assert "1" in verdict.reason  # goals completed before the stall


def test_stall_deadline_is_measured_from_the_last_completed_goal(wd):
    """A goal finishing resets the budget; the clock does not run from the cell start."""
    clock = FakeClock()
    completed = [1]
    watchdog = _make(wd, clock, progress=lambda: completed[0], goal_timeout_s=120.0)

    watchdog.poll()
    clock.advance(119.0)
    completed[0] = 2
    assert watchdog.poll() is None

    clock.advance(119.0)
    assert watchdog.poll() is None, "deadline must restart at the second goal"

    clock.advance(2.0)
    assert watchdog.poll() is not None


def test_first_goal_gets_the_startup_budget_not_the_goal_budget(wd):
    """Bringup, map load and the first plan are not a goal and must not be timed as one."""
    clock = FakeClock()
    watchdog = _make(wd, clock, progress=lambda: 0, goal_timeout_s=120.0,
                     startup_timeout_s=300.0)

    clock.advance(200.0)
    assert watchdog.poll() is None, "still inside the startup budget"

    clock.advance(101.0)
    verdict = watchdog.poll()
    assert verdict is not None
    assert verdict.code == wd.EXIT_STALLED
    assert "first goal" in verdict.reason


# --- liveness detector -------------------------------------------------------------


def test_a_dead_stack_process_aborts_immediately(wd):
    """planner_server dying is decisive; nothing is gained by waiting out a timeout."""
    clock = FakeClock()
    watchdog = _make(wd, clock, progress=lambda: 1,
                     liveness=lambda: ["planner_server"])

    verdict = watchdog.poll()

    assert verdict is not None
    assert verdict.code == wd.EXIT_STACK_DOWN
    assert "planner_server" in verdict.reason


def test_a_dead_process_outranks_a_stall(wd):
    """When both fire, report the cause rather than the symptom."""
    clock = FakeClock()
    watchdog = _make(wd, clock, progress=lambda: 1,
                     liveness=lambda: ["planner_server"], goal_timeout_s=120.0)

    clock.advance(600.0)
    verdict = watchdog.poll()

    assert verdict.code == wd.EXIT_STACK_DOWN


def test_abort_codes_are_distinct_and_non_zero(wd):
    """A cell runner must be able to tell the two failures apart from the exit status."""
    assert wd.EXIT_STALLED != wd.EXIT_STACK_DOWN
    assert wd.EXIT_STALLED != 0
    assert wd.EXIT_STACK_DOWN != 0
    # 2 and 3 already mean "no CSV" and "short CSV" in multi_run_harness.
    assert wd.EXIT_STALLED not in (2, 3)
    assert wd.EXIT_STACK_DOWN not in (2, 3)


# --- real probes -------------------------------------------------------------------


def test_completed_goals_counts_data_rows_only(wd, tmp_path):
    """Progress is read from the metrics CSV, whose header is not a goal."""
    csv = tmp_path / "metrics.csv"
    csv.write_text("goal_index,result\n0,SUCCESS\n1,FAILED\n")

    assert wd.completed_goals(csv) == 2


def test_completed_goals_is_zero_before_the_run_writes_anything(wd, tmp_path):
    """The CSV does not exist until the compiler starts; that is startup, not an error."""
    assert wd.completed_goals(tmp_path / "not_written_yet.csv") == 0


def _stand_in(tmp_path, name):
    """A real executable with a chosen process name.

    Named uniquely rather than reusing ``sleep``: any other process on the machine called
    ``sleep`` -- a background wait, another campaign -- would satisfy the probe and the test
    would pass or fail on what else happens to be running.
    """
    binary = tmp_path / name
    binary.write_bytes(pathlib.Path("/bin/sleep").read_bytes())
    binary.chmod(0o755)
    return binary


def test_dead_processes_is_empty_while_the_process_runs(wd, tmp_path):
    """The liveness probe must not cry wolf on a healthy stack."""
    binary = _stand_in(tmp_path, "navlearn_alive")
    child = subprocess.Popen([str(binary), "30"])
    try:
        time.sleep(0.5)
        assert wd.dead_processes(["navlearn_alive"], launch_pid=os.getpid()) == []
    finally:
        child.kill()
        child.wait()


def test_dead_processes_names_a_process_that_is_gone(wd, tmp_path):
    """The name is the whole point: 'the run hung' is not a diagnosis."""
    binary = _stand_in(tmp_path, "navlearn_doomed")
    child = subprocess.Popen([str(binary), "30"])
    child.kill()
    child.wait()
    time.sleep(0.5)

    dead = wd.dead_processes(["navlearn_doomed"], launch_pid=os.getpid())

    assert dead == ["navlearn_doomed"]


def test_dead_processes_matches_a_name_longer_than_the_kernel_comm_limit(wd, tmp_path):
    """``controller_server`` is 17 characters; the kernel stores 15.

    /proc/PID/comm is truncated to TASK_COMM_LEN-1, so ``pgrep -x controller_server``
    matches nothing however healthy the stack is. A liveness probe with that hole reports a
    running stack as dead and aborts every cell on its first poll -- strictly worse than no
    watchdog, because it destroys campaigns instead of merely failing to save them.
    """
    binary = tmp_path / "controller_server"
    binary.write_bytes(pathlib.Path("/bin/sleep").read_bytes())
    binary.chmod(0o755)
    child = subprocess.Popen([str(binary), "30"])
    try:
        time.sleep(0.5)
        assert wd.dead_processes(["controller_server"]) == []
    finally:
        child.kill()
        child.wait()


def test_dead_processes_reports_a_launch_that_has_exited(wd):
    """A torn-down bringup is the failure the 8-hour hang was made of."""
    child = subprocess.Popen(["true"])
    child.wait()

    dead = wd.dead_processes([], launch_pid=child.pid)

    assert any("bringup" in name for name in dead)


# --- supervision -------------------------------------------------------------------


def test_supervise_returns_none_when_the_run_exits_on_its_own(wd):
    """A healthy run must be unaffected by the watchdog."""
    process = subprocess.Popen(["true"], start_new_session=True)
    never = _make(wd, FakeClock(), progress=lambda: 1)

    assert wd.supervise(process, never, poll_interval_s=0.05) is None
    assert process.returncode == 0


def test_supervise_kills_a_hung_run_and_reports_why(wd):
    """The defect this whole file exists for: the launch must not outlive the abort."""
    process = subprocess.Popen(["sleep", "300"], start_new_session=True)

    class AlwaysStalled:
        def poll(self):
            return wd.Abort(wd.EXIT_STALLED, "no goal completed in 120 s")

    verdict = wd.supervise(process, AlwaysStalled(), poll_interval_s=0.05,
                           grace_s=2.0)

    assert verdict is not None
    assert verdict.code == wd.EXIT_STALLED
    assert process.poll() is not None, "the hung launch was left running"


def test_supervise_kills_the_whole_process_group(wd):
    """ros2 launch spawns the nodes; killing only the launch leaves the stack behind."""
    process = subprocess.Popen(
        [sys.executable, "-c",
         "import subprocess, sys; "
         "c = subprocess.Popen(['sleep', '300']); "
         "print(c.pid, flush=True); "
         "c.wait()"],
        stdout=subprocess.PIPE, text=True, start_new_session=True,
    )
    grandchild = int(process.stdout.readline().strip())

    class AlwaysDown:
        def poll(self):
            return wd.Abort(wd.EXIT_STACK_DOWN, "planner_server is gone")

    wd.supervise(process, AlwaysDown(), poll_interval_s=0.05, grace_s=2.0)

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            os.kill(grandchild, 0)
        except OSError:
            break
        time.sleep(0.1)
    else:
        os.kill(grandchild, signal.SIGKILL)
        pytest.fail("grandchild survived the abort; only the launch process was killed")


def test_supervise_captures_state_before_it_kills_the_launch(wd):
    """Teardown is what makes a hang unreproducible.

    Once the process group is signalled, the state that would have explained the failure is
    gone. A forensic hook that runs after the kill records the aftermath, not the fault.
    """
    process = subprocess.Popen(["sleep", "300"], start_new_session=True)
    observed = {}

    class Stalled:
        def poll(self):
            return wd.Abort(wd.EXIT_STALLED, "stalled")

    def on_abort(verdict):
        observed["alive_when_called"] = process.poll() is None
        observed["reason"] = verdict.reason

    wd.supervise(process, Stalled(), poll_interval_s=0.05, grace_s=2.0, on_abort=on_abort)

    assert observed["alive_when_called"] is True, "state was captured after the kill"
    assert observed["reason"] == "stalled"


def test_supervise_kills_the_launch_even_if_the_forensic_hook_raises(wd):
    """A diagnostic that can fail must not become a second outage."""
    process = subprocess.Popen(["sleep", "300"], start_new_session=True)

    class Stalled:
        def poll(self):
            return wd.Abort(wd.EXIT_STALLED, "stalled")

    def explode(_verdict):
        raise RuntimeError("forensics blew up")

    verdict = wd.supervise(process, Stalled(), poll_interval_s=0.05, grace_s=2.0,
                           on_abort=explode)

    assert verdict.code == wd.EXIT_STALLED
    assert process.poll() is not None, "the hung launch survived a failing hook"


# --- harness wiring ----------------------------------------------------------------


def _args(harness, monkeypatch, *extra):
    monkeypatch.setattr(
        sys, "argv",
        ["multi_run_harness.py", "--perturbation", "clean",
         "--output-dir", "/tmp/unused", *extra],
    )
    return harness.parse_args()


def test_harness_exposes_both_watchdog_budgets(harness, monkeypatch):
    """The budgets are cell properties: a sensor-starved leg is slower than a clean one."""
    args = _args(harness, monkeypatch, "--goal-timeout", "42", "--startup-timeout", "99")

    assert args.goal_timeout == 42.0
    assert args.startup_timeout == 99.0


def test_harness_watchdog_defaults_clear_the_slowest_observed_goal(harness, monkeypatch):
    """A default that aborts a legitimately slow goal is worse than no watchdog."""
    args = _args(harness, monkeypatch)

    assert args.goal_timeout > LEG1_SLOWEST_GOAL_S
    assert args.startup_timeout > LEG1_SLOWEST_GOAL_S


def test_harness_watchdog_cannot_be_silently_disabled(harness, monkeypatch, capsys):
    """Zero or negative would read as 'off' and reproduce the 8-hour hang."""
    monkeypatch.setattr(
        sys, "argv",
        ["multi_run_harness.py", "--perturbation", "clean", "--output-dir", "/tmp/unused",
         "--goal-timeout", "0"],
    )
    with pytest.raises(SystemExit):
        harness.parse_args()

    assert "positive" in capsys.readouterr().err


def test_launch_supervised_reports_the_exit_code_of_a_healthy_run(harness, wd, tmp_path):
    """The watchdog must be invisible on the path that already worked."""
    class Never:
        def poll(self):
            return None

    log = tmp_path / "nodes.log"
    with open(log, "w") as handle:
        code, abort = harness.launch_supervised(["true"], handle, Never(),
                                                poll_interval_s=0.05)

    assert code == 0
    assert abort is None


def test_launch_supervised_kills_a_hung_launch_and_spares_the_harness(
        harness, wd, tmp_path):
    """The group kill must reach the launch's children and stop at the harness itself.

    If the launch were not started in its own session, killpg would target the harness's
    own group -- this test process -- and take the campaign down with the cell.
    """
    class Stalled:
        def poll(self):
            return wd.Abort(wd.EXIT_STALLED, "no goal completed in 600.0 s")

    log = tmp_path / "nodes.log"
    with open(log, "w") as handle:
        code, abort = harness.launch_supervised(["sleep", "300"], handle, Stalled(),
                                                poll_interval_s=0.05)

    assert abort is not None
    assert abort.code == wd.EXIT_STALLED
    assert code != 0, "a killed launch must not report success"
    os.kill(os.getpid(), 0)  # the harness process survived the group kill
