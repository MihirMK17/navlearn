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

"""Tests for the curve-campaign analysis.

Why it exists
    The leg 1 tables were produced ad hoc and the method left the machine with the session
    that ran it. These are paper numbers: an analysis that cannot be re-run is one step
    from an analysis that cannot be defended. This script IS the method, and every
    definition in it is pinned here against data whose answer is known by construction.

What it must get right
    The definitions come from docs/paper1/PROTOCOL.md (frozen 2026-07-30) and from what
    the nodes actually wrote, verified in source:

      true success   ground truth within the stack's xy_goal_tolerance at episode end
                     ("True Distance To Goal (m)" <= tolerance, tolerance read from the
                     run's own stack spec -- never hardcoded)
      false success  Nav2 reported SUCCEEDED while ground truth was outside tolerance
      recovered      TTR Outcome == 1 (in threshold at goal end, sustained; the node
                     regresses RECOVERED -> RECOVERING on a threshold break)
      TTR            seconds to first sustained convergence, only meaningful when
                     recovered; sentinel -1 rows must never enter a median
      magnitude      realised "Kidnap Displacement (m)" for TTR legs -- the commanded
                     band is a bin label, and regressing on it handicaps distance
                     (see test_kidnap_reference) -- and realised bad-init displacement
                     for TTC legs

    Goals whose kidnap was not applied are attrition, reported and excluded; pooling them
    with applied kidnaps would dilute every rate with unperturbed goals.
"""

import importlib.util
import json
import os

import pytest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(TEST_DIR)
SCRIPT = os.path.join(PACKAGE_DIR, "scripts", "analyze_curve_campaign.py")


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def mod():
    return _load("analyze_curve_campaign", SCRIPT)


WIDE_HEADER = (
    "Goal_ID,Goal Result,True Distance To Goal (m),GT Available,"
    "Kidnap Attempted,Kidnap Applied,Kidnap Displacement (m),"
    "Kidnap Commanded Magnitude (m),Nav Time (sec)"
)


def _arm(tmp_path, name, wide_rows, loc_rows, tolerance=0.05):
    """Materialise one arm directory in the harness's real layout."""
    arm = tmp_path / f"leg_final_{name}"
    arm.mkdir()
    (arm / "navlearn_metrics_run_1_20260802_000000.csv").write_text(
        WIDE_HEADER + "\n" + "\n".join(wide_rows) + "\n")
    (arm / "navlearn_metrics_run_1_20260802_000000_localization.csv").write_text(
        "GoalID,Metric Class,Metric Source,Metric Name,Metric Value,Timestamp\n"
        + "\n".join(loc_rows) + "\n")
    (arm / "navlearn_stack_spec_run_1_20260802_000000.json").write_text(json.dumps({
        "identity": {"xy_goal_tolerance": tolerance},
        "selection": {"controller": name},
    }))
    return arm


def _ttr_rows(goal, outcome, ttr):
    return [
        f"{goal},Localization Recovery,TTR,TTR Outcome,{outcome},44.0",
        f"{goal},Localization Recovery,TTR,TTR,{ttr},44.0",
    ]


# --- definitions -------------------------------------------------------------------


def test_the_four_outcome_classes_are_classified_exactly(mod, tmp_path):
    """One goal per class, each answer known by construction."""
    arm = _arm(
        tmp_path, "rpp",
        [
            # true success: SUCCEEDED and truly at the goal
            "g1,SUCCEEDED,0.03,1,1,1,0.50,0.50,30.0",
            # false success: SUCCEEDED, 1.2 m away
            "g2,SUCCEEDED,1.20,1,1,1,0.60,0.60,30.0",
            # honest failure: FAILED, far away
            "g3,FAILED,4.00,1,1,1,2.00,2.00,90.0",
            # boundary: exactly at tolerance counts as true (<=, matching the protocol's
            # "within")
            "g4,SUCCEEDED,0.05,1,1,1,0.70,0.70,30.0",
            # the MPPI signature: truly at the goal but never declared. True success is
            # about where the robot IS, not what Nav2 said -- leg 1's true 73.3% against
            # reported 38.3% is only possible under this reading.
            "g5,FAILED,0.03,1,1,1,0.40,0.40,120.0",
        ],
        _ttr_rows("g1", 1, 2.0) + _ttr_rows("g2", 0, -1.0)
        + _ttr_rows("g3", 2, -1.0) + _ttr_rows("g4", 1, 3.0) + _ttr_rows("g5", 1, 2.5),
    )

    goals = mod.load_arm(arm)

    verdicts = {g.goal_id: (g.true_success, g.false_success) for g in goals}
    assert verdicts == {
        "g1": (True, False),
        "g2": (False, True),
        "g3": (False, False),
        "g4": (True, False),
        "g5": (True, False),
    }


def test_tolerance_comes_from_the_stack_spec_not_a_constant(mod, tmp_path):
    """A stack probed at a different tolerance must be judged at that tolerance."""
    arm = _arm(
        tmp_path, "rpp",
        ["g1,SUCCEEDED,0.20,1,1,1,0.5,0.5,30.0"],
        _ttr_rows("g1", 1, 2.0),
        tolerance=0.25,
    )

    goals = mod.load_arm(arm)

    assert goals[0].true_success, "0.20 m is within a 0.25 m tolerance"


def test_recovery_comes_from_ttr_outcome_and_ttr_median_ignores_sentinels(mod, tmp_path):
    """Outcome 1 is the sustained-recovery flag; -1 is 'no time', not a fast recovery."""
    arm = _arm(
        tmp_path, "rpp",
        [
            "g1,SUCCEEDED,0.03,1,1,1,0.5,0.5,30.0",
            "g2,FAILED,3.00,1,1,1,2.5,2.5,90.0",
            "g3,SUCCEEDED,0.03,1,1,1,0.5,0.5,30.0",
        ],
        _ttr_rows("g1", 1, 4.0) + _ttr_rows("g2", 0, -1.0) + _ttr_rows("g3", 1, 8.0),
    )

    goals = mod.load_arm(arm)
    summary = mod.summarise(goals)

    assert summary["recovered_pct"] == pytest.approx(100 * 2 / 3)
    assert summary["ttr_median_s"] == pytest.approx(6.0), \
        "the -1 sentinel entered the median"


def test_unapplied_kidnaps_are_attrition_not_data(mod, tmp_path):
    """A goal whose kidnap failed ran unperturbed; pooling it dilutes every rate."""
    arm = _arm(
        tmp_path, "rpp",
        [
            "g1,SUCCEEDED,0.03,1,1,1,0.5,0.5,30.0",
            "g2,SUCCEEDED,0.03,1,1,0,-1,1.5,30.0",   # attempted, never applied
        ],
        _ttr_rows("g1", 1, 2.0),
    )

    goals = mod.load_arm(arm)
    summary = mod.summarise(goals)

    assert summary["n"] == 1
    assert summary["attrition"] == 1


def test_magnitude_is_the_realised_displacement(mod, tmp_path):
    """The commanded band is a label; the robot moved by the realised distance."""
    arm = _arm(
        tmp_path, "rpp",
        ["g1,SUCCEEDED,0.03,1,1,1,0.83,0.80,30.0"],
        _ttr_rows("g1", 1, 2.0),
    )

    goals = mod.load_arm(arm)

    assert goals[0].magnitude == pytest.approx(0.83)


def test_missing_ground_truth_is_excluded_and_counted(mod, tmp_path):
    """Without ground truth, true/false success is unknowable, not zero."""
    arm = _arm(
        tmp_path, "rpp",
        [
            "g1,SUCCEEDED,0.03,1,1,1,0.5,0.5,30.0",
            "g2,SUCCEEDED,0.00,0,1,1,0.5,0.5,30.0",   # GT Available = 0
        ],
        _ttr_rows("g1", 1, 2.0) + _ttr_rows("g2", 1, 2.0),
    )

    summary = mod.summarise(mod.load_arm(arm))

    assert summary["n"] == 1
    assert summary["no_gt"] == 1


def test_each_arm_keeps_its_own_attrition_counts(mod, tmp_path):
    """Loading arm B must not overwrite arm A's attrition.

    The report loads every arm first and summarises later; counts smuggled through
    module or function state would make all three arms report whichever loaded last.
    """
    arm_a = _arm(
        tmp_path, "rpp",
        [
            "g1,SUCCEEDED,0.03,1,1,1,0.5,0.5,30.0",
            "g2,SUCCEEDED,0.03,1,1,0,-1,1.5,30.0",   # attrition in A
        ],
        _ttr_rows("g1", 1, 2.0),
    )
    arm_b = _arm(
        tmp_path, "dwb",
        ["g1,SUCCEEDED,0.03,1,1,1,0.5,0.5,30.0"],    # none in B
        _ttr_rows("g1", 1, 2.0),
    )

    goals_a = mod.load_arm(arm_a)
    goals_b = mod.load_arm(arm_b)

    assert mod.summarise(goals_a)["attrition"] == 1
    assert mod.summarise(goals_b)["attrition"] == 0


# --- binning -----------------------------------------------------------------------


def test_bins_are_log_quartiles_of_the_commanded_range(mod):
    """Same construction as the leg 1 tables: log-quartiles of the sweep range."""
    edges = mod.log_bin_edges(0.01, 3.0, 4)

    assert edges[0] == pytest.approx(0.01)
    assert edges[-1] == pytest.approx(3.0)
    ratios = [edges[i + 1] / edges[i] for i in range(4)]
    assert all(r == pytest.approx(ratios[0], rel=1e-9) for r in ratios), \
        "bins must be uniform in log space"


def test_binned_rates_place_each_goal_in_exactly_one_bin(mod, tmp_path):
    """A goal on a bin edge must not be double-counted or dropped."""
    arm = _arm(
        tmp_path, "rpp",
        [
            "g1,SUCCEEDED,0.03,1,1,1,0.02,0.02,30.0",
            "g2,SUCCEEDED,1.00,1,1,1,2.90,2.90,30.0",
            f"g3,FAILED,3.00,1,1,1,{0.01 * (300 ** 0.25):.6f},0.04,90.0",  # on edge 1
        ],
        _ttr_rows("g1", 1, 2.0) + _ttr_rows("g2", 0, -1.0) + _ttr_rows("g3", 0, -1.0),
    )

    goals = mod.load_arm(arm)
    bins = mod.binned(goals, mod.log_bin_edges(0.01, 3.0, 4))

    assert sum(b["n"] for b in bins) == 3


# --- end to end --------------------------------------------------------------------


def test_report_covers_every_arm_and_states_the_definitions(mod, tmp_path):
    """The output must be self-describing: numbers without definitions rot."""
    for name in ("rpp", "dwb"):
        _arm(tmp_path, name,
             ["g1,SUCCEEDED,0.03,1,1,1,0.5,0.5,30.0"], _ttr_rows("g1", 1, 2.0))

    report = mod.build_report(
        {"rpp": mod.load_arm(tmp_path / "leg_final_rpp"),
         "dwb": mod.load_arm(tmp_path / "leg_final_dwb")},
        lo=0.01, hi=3.0,
    )

    for needle in ("rpp", "dwb", "xy_goal_tolerance", "TTR Outcome", "PROTOCOL.md",
                   "realised"):
        assert needle in report, f"report does not mention {needle}"


def test_arm_with_wrong_row_count_is_refused(mod, tmp_path):
    """Analysing a partial arm silently is how a crashed campaign becomes a result."""
    arm = _arm(
        tmp_path, "rpp",
        ["g1,SUCCEEDED,0.03,1,1,1,0.5,0.5,30.0"], _ttr_rows("g1", 1, 2.0))

    with pytest.raises(SystemExit):
        mod.load_arm(arm, expect_goals=120)
