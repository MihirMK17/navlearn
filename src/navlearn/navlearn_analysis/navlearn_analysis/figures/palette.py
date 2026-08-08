# Copyright 2026 Mihir Kulkarni
# SPDX-License-Identifier: Apache-2.0
"""The repository's figure palette — the validated reference set, light mode.

Slots come from a categorical palette whose adjacent-pair separation was validated
for color-vision deficiency (CVD dE >= 8, normal-vision dE >= 15, OKLab x100); the
first two slots carry every two-series figure in the README. Do not reorder or
substitute hues ad hoc — the order is part of the validated artifact.
"""

SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3e0"

SERIES_1 = "#2a78d6"  # blue — small_house / ground truth
SERIES_2 = "#eb6834"  # orange — bookstore / AMCL belief
EVENT = "#e34948"  # red — the kidnap moment (event accent, labeled, never color-alone)

OCCUPIED = "#26251f"
FREE = "#ffffff"
UNKNOWN = "#d8d7d2"
