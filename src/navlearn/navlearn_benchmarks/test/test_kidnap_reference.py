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

"""Integration tests for the kidnap reference pose reaching the metrics CSV.

What they protect
    The secondary Paper 1 claim compares two predictors of recovery: displacement
    distance and destination ambiguity. Ambiguity is computable offline from the map at
    the destination pose. Realised displacement is not computable from anything unless
    the robot's pose immediately before the teleport is recorded — the alternative is to
    regress distance on its nominal commanded band, which handicaps the very predictor
    under test and makes the comparison unfair in the direction of the paper's claim.

    Two failure modes would be silent. A missing reference pose defaults to (0, 0),
    which is a legitimate map coordinate rather than an obvious sentinel, so the
    regression would run on fabricated displacements. And destination yaw was never
    written at all: ambiguity is an entropy over (x, y, theta), so a destination without
    theta cannot be scored.

Why these run without a simulator
    MetricsCompiler emits a row from an END event plus a ControlMetric and a
    TrajectoryMetric sharing a goal id, with any KidnapEvent for that goal folded in.
    None of that needs Gazebo, so the contract-to-column path is exercised directly.
"""

import math
import os

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 Python runtime not available")

from navlearn_msgs.msg import EpisodeEvent, KidnapEvent  # noqa: E402

# Shared with the terminal-pose suite: same compiler binary, same three-message trigger.
# Duplicating the runner would let the two drift, and a drifted runner is how a column
# mismatch survives a green test suite.
from test_terminal_pose import _episode_end, _run_compiler, _uuid  # noqa: E402

KIDNAP_COLUMNS = [
    "Kidnap Attempted",
    "Kidnap Applied",
    "Kidnap Target_X (m)",
    "Kidnap Target_Y (m)",
    "Kidnap Target_Yaw (deg)",
    "Kidnap Reference Available",
    "Kidnap Reference_X (m)",
    "Kidnap Reference_Y (m)",
    "Kidnap Reference_Yaw (deg)",
    "Kidnap Displacement (m)",
    "Kidnap Yaw Change (deg)",
    "Kidnap Commanded Magnitude (m)",
]


@pytest.fixture(scope="module")
def sourced():
    """Skip the module unless the built workspace is on the environment."""
    if "navlearn_benchmarks" not in os.environ.get("AMENT_PREFIX_PATH", ""):
        pytest.skip("workspace not sourced; run after 'source install/setup.bash'")


def _yaw_quat(yaw):
    """Build a quaternion message body for a planar yaw."""
    return {"z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}


def _kidnap(
    goal_id,
    *,
    target_xy,
    target_yaw=0.0,
    reference_xy=None,
    reference_yaw=0.0,
    success=True,
    commanded_magnitude=-1.0,
):
    """Build a KidnapEvent; reference_xy=None means the reference was never captured."""
    ev = KidnapEvent()
    ev.goal_id = goal_id
    ev.attempt_id = goal_id
    ev.success = success
    ev.commanded_magnitude_m = commanded_magnitude

    ev.kidnap_pose.position.x, ev.kidnap_pose.position.y = target_xy
    q = _yaw_quat(target_yaw)
    ev.kidnap_pose.orientation.z, ev.kidnap_pose.orientation.w = q["z"], q["w"]

    if reference_xy is None:
        ev.reference_available = False
        ev.reference_pose.orientation.w = 1.0
    else:
        ev.reference_available = True
        ev.reference_pose.position.x, ev.reference_pose.position.y = reference_xy
        q = _yaw_quat(reference_yaw)
        ev.reference_pose.orientation.z, ev.reference_pose.orientation.w = (
            q["z"],
            q["w"],
        )
    return ev


def _run(tmp_path, episode, kidnap):
    """Run the compiler for one episode with an accompanying kidnap event."""
    return _run_compiler(tmp_path, [episode], kidnaps=[kidnap] if kidnap else [])


def _plain_episode(seed):
    """A succeeded episode with a populated terminal report."""
    return _episode_end(
        _uuid(seed),
        result=EpisodeEvent.RESULT_SUCCEEDED,
        true_xy=(0.03, 0.0),
        est_xy=(0.02, 0.0),
        goal_xy=(0.0, 0.0),
    )


def test_kidnap_columns_present(sourced, tmp_path):  # noqa: F811
    """Every kidnap field reaches the CSV under its documented name."""
    ep = _plain_episode(11)
    rows, _ = _run(
        tmp_path, ep, _kidnap(ep.goal_id, target_xy=(1.0, 1.0), reference_xy=(0.0, 0.0))
    )
    for column in KIDNAP_COLUMNS:
        assert column in rows[0], f"missing column: {column}"


def test_header_and_row_column_counts_match(sourced, tmp_path):  # noqa: F811
    """A header/row mismatch shifts every column and is invisible in a spreadsheet."""
    ep = _plain_episode(12)
    _, csv_path = _run(
        tmp_path, ep, _kidnap(ep.goal_id, target_xy=(2.0, 3.0), reference_xy=(1.0, 1.0))
    )
    with open(csv_path) as handle:
        lines = [line for line in handle.read().splitlines() if line.strip()]
    header_cols = lines[0].count(",") + 1
    for line in lines[1:]:
        assert (
            line.count(",") + 1 == header_cols
        ), f"row has {line.count(',') + 1} columns, header has {header_cols}"


def test_realised_displacement_is_measured_not_nominal(sourced, tmp_path):  # noqa: F811
    """Displacement is the reference-to-target distance, not the commanded band.

    3-4-5 triangle so an off-by-one in the column order cannot coincidentally pass.
    """
    ep = _plain_episode(13)
    rows, _ = _run(
        tmp_path, ep, _kidnap(ep.goal_id, target_xy=(4.0, 5.0), reference_xy=(1.0, 1.0))
    )
    row = rows[0]
    assert row["Kidnap Reference Available"] == "1"
    assert float(row["Kidnap Reference_X (m)"]) == pytest.approx(1.0, abs=1e-6)
    assert float(row["Kidnap Reference_Y (m)"]) == pytest.approx(1.0, abs=1e-6)
    assert float(row["Kidnap Displacement (m)"]) == pytest.approx(5.0, abs=1e-6)


def test_destination_yaw_reaches_csv(sourced, tmp_path):  # noqa: F811
    """Ambiguity is an entropy over (x, y, theta); a destination without theta is unscoreable."""
    ep = _plain_episode(14)
    rows, _ = _run(
        tmp_path,
        ep,
        _kidnap(
            ep.goal_id,
            target_xy=(1.0, 0.0),
            target_yaw=math.radians(90.0),
            reference_xy=(0.0, 0.0),
            reference_yaw=math.radians(30.0),
        ),
    )
    row = rows[0]
    assert float(row["Kidnap Target_Yaw (deg)"]) == pytest.approx(90.0, abs=1e-4)
    assert float(row["Kidnap Reference_Yaw (deg)"]) == pytest.approx(30.0, abs=1e-4)
    assert float(row["Kidnap Yaw Change (deg)"]) == pytest.approx(60.0, abs=1e-4)


def test_yaw_change_takes_the_shorter_arc(sourced, tmp_path):  # noqa: F811
    """A rotation across the pi boundary is 20 deg, not 340."""
    ep = _plain_episode(15)
    rows, _ = _run(
        tmp_path,
        ep,
        _kidnap(
            ep.goal_id,
            target_xy=(1.0, 0.0),
            target_yaw=math.radians(-170.0),
            reference_xy=(0.0, 0.0),
            reference_yaw=math.radians(170.0),
        ),
    )
    assert float(rows[0]["Kidnap Yaw Change (deg)"]) == pytest.approx(20.0, abs=1e-4)


def test_missing_reference_is_not_a_zero_displacement(sourced, tmp_path):  # noqa: F811
    """An uncaptured reference reports -1, never a distance computed against (0, 0).

    (0, 0) is inside the free space of the campaign map, so a defaulted reference would
    produce a plausible displacement rather than an obviously broken one, and the
    distance-vs-ambiguity comparison would silently run on fabricated numbers.
    """
    ep = _plain_episode(16)
    rows, _ = _run(
        tmp_path, ep, _kidnap(ep.goal_id, target_xy=(3.0, 4.0), reference_xy=None)
    )
    row = rows[0]
    assert row["Kidnap Applied"] == "1"
    assert row["Kidnap Reference Available"] == "0"
    assert float(row["Kidnap Displacement (m)"]) == -1.0
    assert float(row["Kidnap Yaw Change (deg)"]) == -1.0


def test_commanded_magnitude_is_recorded_separately_from_the_realised_one(
    sourced, tmp_path
):  # noqa: F811
    """The curve is fitted on what was asked for; the predictor comparison on what happened.

    Free space rarely offers a valid destination at exactly the commanded radius, so the
    two differ. Collapsing them would either hide that error or throw away the design
    variable the sweep is built around.
    """
    ep = _plain_episode(19)
    rows, _ = _run(
        tmp_path,
        ep,
        _kidnap(
            ep.goal_id,
            target_xy=(4.0, 5.0),
            reference_xy=(1.0, 1.0),
            commanded_magnitude=4.8,
        ),
    )
    row = rows[0]
    assert float(row["Kidnap Commanded Magnitude (m)"]) == pytest.approx(4.8, abs=1e-6)
    assert float(row["Kidnap Displacement (m)"]) == pytest.approx(5.0, abs=1e-6)


def test_fixed_severity_reports_no_commanded_magnitude(sourced, tmp_path):  # noqa: F811
    """Outside a continuous sweep there is no commanded magnitude; -1, never 0.

    Zero would enter a curve fit as a real datum at the bottom of the range.
    """
    ep = _plain_episode(20)
    rows, _ = _run(
        tmp_path, ep, _kidnap(ep.goal_id, target_xy=(1.0, 1.0), reference_xy=(0.0, 0.0))
    )
    assert float(rows[0]["Kidnap Commanded Magnitude (m)"]) == -1.0


def test_unkidnapped_goal_reports_no_reference(sourced, tmp_path):  # noqa: F811
    """A clean or TTC goal carries no kidnap event and must not report a displacement."""
    rows, _ = _run(tmp_path, _plain_episode(17), None)
    row = rows[0]
    assert row["Kidnap Attempted"] == "0"
    assert row["Kidnap Reference Available"] == "0"
    assert float(row["Kidnap Displacement (m)"]) == -1.0


def test_failed_kidnap_reports_reference_but_no_displacement(
    sourced, tmp_path
):  # noqa: F811
    """A sampled-but-unapplied kidnap never moved the robot, so displacement is undefined.

    The event is still published (the harness records the failed attempt so the goal can
    be excluded), and the reference pose is still known — but the robot stayed where it
    was, and writing the commanded target's distance here would invent a perturbation
    that never happened.
    """
    ep = _plain_episode(18)
    rows, _ = _run(
        tmp_path,
        ep,
        _kidnap(
            ep.goal_id, target_xy=(4.0, 5.0), reference_xy=(1.0, 1.0), success=False
        ),
    )
    row = rows[0]
    assert row["Kidnap Attempted"] == "1"
    assert row["Kidnap Applied"] == "0"
    assert float(row["Kidnap Displacement (m)"]) == -1.0
