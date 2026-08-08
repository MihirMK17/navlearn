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

"""Cross-language agreement for seed derivation.

The C++ header draws the campaign's conditions; the Python module re-derives them during
analysis. The golden vectors below were produced by the C++ implementation and are also
asserted from C++ in test_navlearn_seed.cpp, so the two are pinned to the same values
rather than merely to each other. A silent divergence would have analysis reasoning about
perturbations the robot never actually received.
"""

import pytest


@pytest.fixture(scope="module")
def seed():
    """The Python seed reference module."""
    from navlearn_analysis import navlearn_seed

    return navlearn_seed


# (campaign_seed, stream name, run_index, goal_index) -> seed, generated from the C++.
GOLDEN = [
    (42, "GOAL_POSITION", 0, 0, 11244777001763019517),
    (42, "INITIAL_POSE", 0, 0, 4228068372608098218),
    (42, "INITIAL_POSE", 1, 0, 11275064635349857629),
    (42, "INITIAL_POSE", 0, 1, 3350397314887148275),
    (42, "KIDNAP_TARGET", 4, 4, 10896568657172245899),
    (43, "INITIAL_POSE", 0, 0, 10156579724707918578),
]


@pytest.mark.parametrize("campaign,stream_name,run,goal,expected", GOLDEN)
def test_matches_cpp_golden_vectors(seed, campaign, stream_name, run, goal, expected):
    """Python must reproduce exactly what the C++ drew."""
    stream = getattr(seed.Stream, stream_name)
    assert seed.derive(campaign, stream, run, goal) == expected


def test_output_is_64_bit(seed):
    """Derived seeds must stay inside 64 bits regardless of input magnitude."""
    for run in (0, 1, 2**32, 2**63, 2**64 - 1):
        value = seed.derive(42, seed.Stream.INITIAL_POSE, run, 3)
        assert 0 <= value < 2**64


def test_distinct_across_runs_and_goals(seed):
    """n episodes must be n conditions — the property the old scheme lacked."""
    values = {
        seed.derive(42, seed.Stream.INITIAL_POSE, run, goal)
        for run in range(100)
        for goal in range(25)
    }
    assert len(values) == 2500


def test_run_index_participates(seed):
    """Goal 0 must differ between runs.

    Regression guard for the exact defect this replaced: the previous scheme was
    `1337 + goal_index`, which ignored the run entirely, so goal 0 of every run in the
    campaign drew a byte-identical perturbation.
    """
    values = {seed.derive(42, seed.Stream.INITIAL_POSE, run, 0) for run in range(50)}
    assert len(values) == 50


def test_streams_do_not_collide(seed):
    """Different purposes at the same (run, goal) must not share a seed."""
    values = {seed.derive(42, stream, 3, 7) for stream in seed.Stream}
    assert len(values) == len(seed.Stream)


def test_identical_across_controllers(seed):
    """The derivation takes no controller argument, so all arms get the same conditions."""
    import inspect

    parameters = list(inspect.signature(seed.derive).parameters)
    assert parameters == ["campaign_seed", "stream", "run_index", "goal_index"], (
        "the derivation signature changed; adding a controller-dependent argument would "
        "break the paired-comparison invariant the analysis plan depends on"
    )
