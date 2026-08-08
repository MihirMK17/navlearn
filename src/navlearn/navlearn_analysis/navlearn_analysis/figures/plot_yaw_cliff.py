#!/usr/bin/env python3
# Copyright 2026 Mihir Kulkarni
# SPDX-License-Identifier: Apache-2.0
"""The yaw cliff: kidnap recovery vs |heading change|, both environments.

The repository's headline figure. Binned recovery rates (points, with n per bin) and
the fitted logistic curves for the small_house RPP arm and the bookstore replication.

Regenerate with:
    ros2 run navlearn_analysis plot_yaw_cliff \\
        --env small_house=results/leg_yaw_recompute/per_goal_ttr_recomputed.csv:rpp \\
        --env bookstore=results/leg7_bookstore_recompute_yaw/per_goal_ttr_recomputed.csv \\
        --out media/generated/yaw_cliff.png
"""

import argparse
import csv
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from navlearn_analysis.figures import palette  # noqa: E402
from navlearn_analysis.nested_models import fit_logistic, predict_proba  # noqa: E402

SERIES = [palette.SERIES_1, palette.SERIES_2]


def load_env(spec):
    """LABEL=CSV[:arm] -> (label, |yaw| deg array, recovered array)."""
    label, _, rest = spec.partition("=")
    path, _, arm = rest.partition(":")
    rows = list(csv.DictReader(open(path)))
    if arm:
        rows = [r for r in rows if r.get("arm") == arm]
    yaw = np.array([abs(float(r["magnitude_m"])) for r in rows])
    y = np.array([int(r["recovered_sustained"]) for r in rows], dtype=float)
    return label, yaw, y


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--env", action="append", required=True, metavar="LABEL=CSV[:ARM]"
    )
    parser.add_argument("--bins", type=int, default=6)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    fig, ax = plt.subplots(figsize=(7.6, 4.6), dpi=150)
    fig.patch.set_facecolor(palette.SURFACE)
    ax.set_facecolor(palette.SURFACE)

    edges = np.linspace(0.0, 180.0, args.bins + 1)
    centers = (edges[:-1] + edges[1:]) / 2
    grid = np.linspace(0.0, 180.0, 300)

    for spec, color in zip(args.env, SERIES):
        label, yaw, y = load_env(spec)
        fit = fit_logistic(yaw, y)
        ax.plot(grid, 100 * predict_proba(fit, grid), color=color, lw=2, zorder=3)
        rates, ns = [], []
        for lo, hi in zip(edges[:-1], edges[1:]):
            sel = (yaw >= lo) & (yaw < hi) | ((hi == 180.0) & (yaw == hi))
            ns.append(int(sel.sum()))
            rates.append(100 * y[sel].mean() if sel.any() else np.nan)
        ax.scatter(
            centers,
            rates,
            s=42,
            color=color,
            zorder=4,
            edgecolors=palette.SURFACE,
            linewidths=1.5,
        )
        # Direct label at the curve's left end, offset per series to avoid collision.
        y0 = 100 * predict_proba(fit, np.array([12.0]))[0]
        ax.annotate(
            label,
            (14.0, y0),
            textcoords="offset points",
            xytext=(6, 8 if color == palette.SERIES_1 else -14),
            color=palette.TEXT_PRIMARY,
            fontsize=10,
            fontweight="bold",
        )

    ax.axhspan(0, 5, color=palette.GRID, alpha=0.5, zorder=1)
    ax.set_xlim(0, 180)
    ax.set_ylim(-2, 102)
    ax.set_xticks(edges)
    ax.set_xlabel(
        "|heading change| injected by the kidnap (deg)", color=palette.TEXT_SECONDARY
    )
    ax.set_ylabel("episode-window recovery (%)", color=palette.TEXT_SECONDARY)
    ax.set_title(
        "Rotation kills AMCL kidnap recovery — in both environments",
        color=palette.TEXT_PRIMARY,
        fontsize=12,
        loc="left",
        pad=12,
    )
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(palette.GRID)
    ax.tick_params(colors=palette.TEXT_SECONDARY)
    ax.grid(axis="y", color=palette.GRID, lw=0.6, zorder=0)
    ax.legend(
        handles=[
            plt.Line2D([], [], color=c, lw=2, label=load_env(s)[0])
            for s, c in zip(args.env, SERIES)
        ],
        frameon=False,
        loc="upper right",
        labelcolor=palette.TEXT_PRIMARY,
    )

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, bbox_inches="tight", facecolor=palette.SURFACE)
    plt.close(fig)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
