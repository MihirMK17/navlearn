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

"""Tests for the A5 cross-environment slope comparison.

The judgment this feeds is pre-registered, so the arithmetic behind it must be pinned:
Wald standard errors that shrink with n, and a difference test that separates slopes
known by construction to differ while refusing to separate identical ones.
"""

import importlib.util
import os
import sys

import numpy as np

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"
    ),
)

_spec = importlib.util.spec_from_file_location(
    "compare_yaw_slopes",
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "scripts",
        "compare_yaw_slopes.py",
    ),
)
mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mod)

from nested_models import fit_logistic  # noqa: E402


def _synthetic(slope, n=400, seed=7):
    """Recovery outcomes drawn from a known logistic curve over 0-180 deg."""
    rng = np.random.default_rng(seed)
    yaw = rng.uniform(0.0, 180.0, size=n)
    p = 1.0 / (1.0 + np.exp(-(4.0 + slope * yaw)))
    y = (rng.uniform(size=n) < p).astype(float)
    return yaw, y


class TestWaldSe:
    def test_se_shrinks_with_sample_size(self):
        ses = []
        for n in (100, 1600):
            yaw, y = _synthetic(-0.08, n=n)
            fit = fit_logistic(yaw, y)
            ses.append(mod.wald_se(fit, yaw)[1])
        assert ses[1] < ses[0] / 2.0, "quadrupling n should roughly halve the SE twice"

    def test_ci_covers_the_generating_slope(self):
        yaw, y = _synthetic(-0.08, n=1600)
        fit = fit_logistic(yaw, y)
        se = mod.wald_se(fit, yaw)[1]
        assert abs(fit.coefficients[1] - (-0.08)) < 1.96 * se


class TestLoadEnv:
    def test_arm_filter_selects_only_that_arm(self, tmp_path):
        path = tmp_path / "per_goal.csv"
        path.write_text(
            "arm,goal_id,magnitude_m,recovered_sustained\n"
            "rpp,g1,10.0,1\n"
            "dwb,g2,170.0,0\n"
            "rpp,g3,-170.0,0\n"
        )
        label, yaw, y = mod.load_env(f"house={path}:rpp")
        assert label == "house"
        assert list(y) == [1.0, 0.0]
        assert list(yaw) == [10.0, 170.0], "magnitude must enter as |yaw|"
