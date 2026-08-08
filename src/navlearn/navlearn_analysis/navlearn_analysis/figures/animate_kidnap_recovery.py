#!/usr/bin/env python3
# Copyright 2026 Mihir Kulkarni
# SPDX-License-Identifier: Apache-2.0
"""Animate one kidnap episode from its rosbag: ground truth vs AMCL belief on the map.

The README's hero image. Picks (or is told) a goal from a yaw-curve cell where the
filter regained its own pre-kidnap confidence while facing the wrong way — the
confident-and-wrong signature the paper measures — and renders truth (blue) against
belief (orange) with heading arrows, the kidnap flash, and a verdict caption.

Regenerate with:
    ros2 run navlearn_analysis animate_kidnap_recovery \\
        --cell results/leg_yaw_rpp \\
        --recompute results/leg_yaw_recompute/per_goal_ttr_recomputed.csv \\
        --mechanism results/leg_yaw_mechanism_v2/per_goal_mechanism.csv \\
        --map <bumperbot_mapping share>/maps/small_house/map.yaml \\
        --out media/generated/kidnap_recovery.gif
"""

import argparse
import csv
import math
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

from navlearn_analysis.figures import palette  # noqa: E402
from navlearn_analysis.figures.render_map import load_map  # noqa: E402
from navlearn_analysis.rate_mechanism import read_bag  # noqa: E402


def pick_goal(recompute_csv, mechanism_csv, arm, min_yaw_deg):
    """A non-recovered goal above min_yaw whose filter still reached confidence.

    Returns (run name, goal_id, |yaw| deg). Deterministic: the largest usable |yaw|,
    so reruns regenerate the same episode.
    """
    confident = set()
    for r in csv.DictReader(open(mechanism_csv)):
        if r.get("t_confident"):
            confident.add(r["goal_id"])
    candidates = []
    for r in csv.DictReader(open(recompute_csv)):
        yaw = abs(float(r["magnitude_m"]))
        if (
            r.get("arm", arm) == arm
            and int(r["recovered_sustained"]) == 0
            and yaw >= min_yaw_deg
            and r["goal_id"] in confident
        ):
            candidates.append((yaw, r["run"], r["goal_id"]))
    if not candidates:
        raise SystemExit("no goal matches the confident-and-wrong criteria")
    yaw, run, goal = max(candidates)
    return run, goal, yaw


def world_to_cell(x, y, origin, res, height):
    """Map frame (m) -> image cell (col, row)."""
    return (x - origin[0]) / res, height - (y - origin[1]) / res


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cell", required=True, help="campaign cell dir with rosbags")
    parser.add_argument("--recompute", required=True)
    parser.add_argument("--mechanism", required=True)
    parser.add_argument("--map", required=True, help="map.yaml of the environment")
    parser.add_argument("--arm", default="rpp")
    parser.add_argument("--min-yaw", type=float, default=120.0)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    run, goal, yaw_deg = pick_goal(
        args.recompute, args.mechanism, args.arm, args.min_yaw
    )
    print(f"episode: {run} goal {goal[:8]}…  |yaw| = {yaw_deg:.0f} deg")

    bag = next(pathlib.Path(args.cell).glob(f"{run}/*.db3"))
    amcl, gt, kidnaps, ends = read_bag(bag)
    k_t, end_t = kidnaps[goal], ends[goal]
    t0, t1 = k_t - 6.0, min(end_t, k_t + 24.0)

    amcl_w = [(t, x, y, yw) for (t, x, y, yw, _) in amcl if t0 <= t <= t1]
    gt_w = [(t, x, y, yw) for (t, x, y, yw) in gt if t0 <= t <= t1]

    import yaml as _yaml

    meta = _yaml.safe_load(pathlib.Path(args.map).read_text())
    img, res = load_map(args.map)
    origin = meta["origin"]
    h, w = img.shape

    rgb = np.empty((*img.shape, 3))
    rgb[:] = matplotlib.colors.to_rgb(palette.UNKNOWN)
    rgb[img > 250] = matplotlib.colors.to_rgb(palette.FREE)
    rgb[img < 255 * 0.35] = matplotlib.colors.to_rgb(palette.OCCUPIED)

    # Zoom to the action. A rotation kidnap teleports in place, so at full-map zoom the
    # whole story is two dots — frame the episode's bounding box instead, padded enough
    # to show the room around it.
    xs = [p[1] for p in gt_w + amcl_w]
    ys = [p[2] for p in gt_w + amcl_w]
    pad = 2.5
    cx0, cy0 = world_to_cell(min(xs) - pad, min(ys) - pad, origin, res, h)
    cx1, cy1 = world_to_cell(max(xs) + pad, max(ys) + pad, origin, res, h)
    view_w, view_h = abs(cx1 - cx0), abs(cy0 - cy1)

    fig, ax = plt.subplots(figsize=(7.2, 7.2 * view_h / view_w), dpi=110)
    fig.patch.set_facecolor(palette.SURFACE)
    ax.imshow(rgb, interpolation="nearest")
    ax.set_xlim(min(cx0, cx1), max(cx0, cx1))
    ax.set_ylim(max(cy0, cy1), min(cy0, cy1))  # image rows grow downward
    ax.set_axis_off()

    (gt_line,) = ax.plot([], [], color=palette.SERIES_1, lw=2, label="ground truth")
    (amcl_line,) = ax.plot(
        [], [], color=palette.SERIES_2, lw=2, ls="--", label="AMCL belief"
    )
    gt_arrow = ax.annotate(
        "",
        xy=(0, 0),
        xytext=(0, 0),
        arrowprops=dict(color=palette.SERIES_1, width=2, headwidth=9),
    )
    amcl_arrow = ax.annotate(
        "",
        xy=(0, 0),
        xytext=(0, 0),
        arrowprops=dict(color=palette.SERIES_2, width=2, headwidth=9),
    )
    caption = ax.text(
        0.02,
        0.02,
        "",
        transform=ax.transAxes,
        fontsize=10,
        color=palette.TEXT_PRIMARY,
        va="bottom",
    )
    ax.legend(frameon=False, loc="upper right", labelcolor=palette.TEXT_PRIMARY)
    ax.set_title(
        f"Kidnap with {yaw_deg:.0f}° rotation — the filter recovers its confidence,"
        " not its heading",
        color=palette.TEXT_PRIMARY,
        fontsize=10.5,
        loc="left",
        pad=8,
    )

    times = np.arange(t0, t1, 1.0 / 3.0)  # 3 sim-Hz keyframes

    def series_until(series, t):
        pts = [(x, y, yw) for (ts, x, y, yw) in series if ts <= t]
        return pts

    def arrow_at(annot, pts, arrow_len_m=0.55):
        if not pts:
            return
        x, y, yw = pts[-1]
        cx, cy = world_to_cell(x, y, origin, res, h)
        tx, ty = world_to_cell(
            x + arrow_len_m * math.cos(yw),
            y + arrow_len_m * math.sin(yw),
            origin,
            res,
            h,
        )
        annot.set_position((cx, cy))
        annot.xy = (tx, ty)

    def frame(i):
        t = times[i]
        g = series_until(gt_w, t)
        a = series_until(amcl_w, t)
        if g:
            xs, ys = zip(*[world_to_cell(x, y, origin, res, h) for x, y, _ in g])
            gt_line.set_data(xs, ys)
        if a:
            xs, ys = zip(*[world_to_cell(x, y, origin, res, h) for x, y, _ in a])
            amcl_line.set_data(xs, ys)
        arrow_at(gt_arrow, g)
        arrow_at(amcl_arrow, a)
        if t < k_t:
            caption.set_text("driving normally — belief tracks truth")
            caption.set_color(palette.TEXT_SECONDARY)
        elif t < k_t + 3.0:
            caption.set_text(f"KIDNAP: teleported in place, rotated {yaw_deg:.0f}°")
            caption.set_color(palette.EVENT)
        else:
            caption.set_text(
                "heading never corrects — belief mirrors reality as the robot drives.\n"
                "AMCL's covariance reports nothing wrong the entire time."
            )
            caption.set_color(palette.TEXT_PRIMARY)
        return gt_line, amcl_line

    anim = FuncAnimation(fig, frame, frames=len(times), blit=False)
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    anim.save(out, writer=PillowWriter(fps=args.fps))
    plt.close(fig)
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
