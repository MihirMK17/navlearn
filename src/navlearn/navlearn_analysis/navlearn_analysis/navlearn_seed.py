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

"""Python reference for benchmark seed derivation.

Mirrors ``include/navlearn_benchmarks/navlearn_seed.hpp`` exactly. Analysis needs to
answer "which perturbation did episode 3, goal 2 of this cell actually receive?" without
linking C++ or re-running the simulator, and a run's manifest records the campaign seed
and run index precisely so that question is answerable after the fact.

The two implementations are pinned to shared golden vectors in test_navlearn_seed.cpp and
test_seed_reference.py. If either drifts, both test suites fail — a silent divergence
would mean analysis reasoning about conditions the robot never experienced.

Any edit here must be mirrored in the header, and vice versa.
"""

from enum import IntEnum

MASK64 = (1 << 64) - 1


class Stream(IntEnum):
    """Distinct random streams. Must match the Stream enum in navlearn_seed.hpp."""

    GOAL_POSITION = 0x243F6A8885A308D3
    GOAL_ORIENTATION = 0x13198A2E03707344
    INITIAL_POSE = 0xA4093822299F31D0
    KIDNAP_TARGET = 0x082EFA98EC4E6C89
    KIDNAP_DELAY = 0x452821E638D01377
    ATTEMPT_ID = 0xBE5466CF34E90C6C


def splitmix64(x: int) -> int:
    """splitmix64 mixing function; the constants are load-bearing, do not adjust."""
    x = (x + 0x9E3779B97F4A7C15) & MASK64
    z = x
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK64
    return (z ^ (z >> 31)) & MASK64


def derive(campaign_seed: int, stream: Stream, run_index: int, goal_index: int) -> int:
    """Derive the seed for one draw.

    The controller is deliberately not an argument: every arm must face identical
    conditions for the campaign's paired comparisons to be valid.
    """
    h = splitmix64(campaign_seed & MASK64)
    h = splitmix64(h ^ int(stream))
    h = splitmix64(h ^ (run_index & MASK64))
    h = splitmix64(h ^ (goal_index & MASK64))
    return h
