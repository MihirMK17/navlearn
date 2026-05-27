#!/usr/bin/env python3
# Copyright 2026 NavLearn Contributors
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
"""Reproducible spatial-zone Fisher analysis for the substrate-coupling diagnostic.

Replaces the manual zone polygons used in the first SUBSTRATE_FINDING.md
draft with a reproducible metric: distance to nearest large-wall connected
component (small furniture pixels excluded via connected-component size
filter), then a quartile-based split into NARROW (bottom quartile of
wall-distance over pooled-MPPI extreme-TTC goals) vs OPEN (top quartile).

Outputs:
  results/spatial_bin_analysis/zone_quartile_fisher.csv
  results/spatial_bin_analysis/distance_to_wall_overlay.png
  results/spatial_bin_analysis/goal_scatter_overlay.png

Reproducible knobs:
  WALL_MIN_PIX = 100      -- connected-component size threshold for "wall"
  QUARTILES = (0.25, 0.75) -- split percentiles for NARROW / OPEN

Usage:
  python3 src/navlearn_benchmarks/scripts/spatial_zone_fisher.py
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import yaml
from PIL import Image
from scipy import ndimage
from scipy.stats import fisher_exact

ROOT = Path("/home/mihirmk/robot_ws")
RESULTS = ROOT / "results/spatial_bin_analysis"
MAP_YAML = ROOT / "src/bumperbot_mapping/maps/small_house/map.yaml"
MAP_PGM = ROOT / "src/bumperbot_mapping/maps/small_house/map.pgm"
GOALS_CSV = RESULTS / "all_goals_pose_outcome.csv"

WALL_MIN_PIX = 100
QUARTILE_LOW, QUARTILE_HIGH = 0.25, 0.75
FAIL_TOKENS = {"FAILED", "ABORTED", "CANCELED", "CANCELLED", "TIMED_OUT"}

FOCUS_3 = ["mppi_baseline", "mppi_baseline_high_tolerance", "mppi_aggressive"]
FOCUS_5 = FOCUS_3 + ["mppi_baseline_fixed_bt", "mppi_baseline_high_vx"]


def load_map_and_wall_distance():
    """Return (map_img, extent, dist_wall_m, res, ox, oy)."""
    with open(MAP_YAML) as f:
        meta = yaml.safe_load(f)
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    img = np.array(Image.open(MAP_PGM))
    prob = (255.0 - img.astype(float)) / 255.0
    occupied = prob >= occupied_thresh
    structure = ndimage.generate_binary_structure(2, 2)
    labeled, n_labels = ndimage.label(occupied, structure=structure)
    sizes = ndimage.sum(occupied, labeled, range(1, n_labels + 1))
    wall_label_ids = np.where(sizes >= WALL_MIN_PIX)[0] + 1
    wall_mask = np.isin(labeled, wall_label_ids)
    dist_wall_m = ndimage.distance_transform_edt(~wall_mask) * res
    H, W = img.shape
    extent = [ox, ox + W * res, oy, oy + H * res]
    print(f"  map shape: {img.shape}, resolution: {res} m/pix")
    print(
        f"  connected components: {n_labels} -> {len(wall_label_ids)} kept "
        f">= {WALL_MIN_PIX} pix"
    )
    print(
        f"  d_wall: min={dist_wall_m.min():.2f}, max={dist_wall_m.max():.2f}, "
        f"mean={dist_wall_m.mean():.2f}"
    )
    return img, extent, dist_wall_m, res, ox, oy


def world_to_rowcol(xs, ys, res, ox, oy, shape):
    H, W = shape
    col = ((xs - ox) / res).astype(int).clip(0, W - 1)
    row = ((H - 1) - ((ys - oy) / res).astype(int)).clip(0, H - 1)
    return row, col


def wilson_ci(k, n, z=1.96):
    if n == 0:
        return float("nan"), float("nan")
    phat = k / n
    denom = 1 + z * z / n
    centre = (phat + z * z / (2 * n)) / denom
    spread = z * np.sqrt(phat * (1 - phat) / n + z * z / (4 * n * n)) / denom
    return centre - spread, centre + spread


def fisher_pair(of_, on_, nf_, nn_):
    table = [[of_, on_ - of_], [nf_, nn_ - nf_]]
    odds, p1 = fisher_exact(table, alternative="greater")
    _, p2 = fisher_exact(table, alternative="two-sided")
    return odds, p1, p2


def render_distance_overlay(img, extent, dist_m, q_lo, q_hi, out_path):
    fig, ax = plt.subplots(figsize=(11, 9))
    ax.imshow(
        img, extent=extent, origin="lower", cmap="gray", alpha=0.6, vmin=0, vmax=255
    )
    masked = np.where(dist_m > 0, dist_m, np.nan)
    im = ax.imshow(
        masked,
        extent=extent,
        origin="lower",
        cmap="viridis",
        alpha=0.5,
        vmin=0,
        vmax=3.0,
    )
    ax.set_xlabel("x (m, map frame)")
    ax.set_ylabel("y (m, map frame)")
    title = (
        f"Distance to nearest wall-component (>={WALL_MIN_PIX} pix) | "
        f"NARROW <= {q_lo:.2f} m | OPEN >= {q_hi:.2f} m"
    )
    ax.set_title(title, fontsize=10)
    cbar = plt.colorbar(im, ax=ax, fraction=0.04, pad=0.04)
    cbar.set_label("Distance to nearest wall (m)")
    plt.tight_layout()
    plt.savefig(out_path, dpi=140)
    plt.close()


def render_scatter_overlay(img, extent, dist_m, goals_df, q_lo, q_hi, out_path):
    fig, ax = plt.subplots(figsize=(11, 9))
    ax.imshow(
        img, extent=extent, origin="lower", cmap="gray", alpha=0.6, vmin=0, vmax=255
    )
    masked = np.where(dist_m > 0, dist_m, np.nan)
    ax.imshow(
        masked,
        extent=extent,
        origin="lower",
        cmap="viridis",
        alpha=0.35,
        vmin=0,
        vmax=3.0,
    )

    sub = goals_df[
        (goals_df["mode"] == "ttc")
        & (goals_df["level"] == "extreme")
        & goals_df["profile"].isin(FOCUS_3)
    ]
    fail = sub[sub["fail"]]
    succ = sub[~sub["fail"]]
    ax.scatter(
        succ["Start Pose_X (m)"],
        succ["Start Pose_Y (m)"],
        c="#1a9850",
        s=22,
        edgecolors="black",
        linewidths=0.4,
        label=f"TTC success (n={len(succ)})",
        alpha=0.85,
    )
    ax.scatter(
        fail["Start Pose_X (m)"],
        fail["Start Pose_Y (m)"],
        c="#d73027",
        s=22,
        marker="x",
        linewidths=1.2,
        label=f"TTC failure (n={len(fail)})",
    )
    ax.set_xlabel("x (m, map frame)")
    ax.set_ylabel("y (m, map frame)")
    ax.set_title(f"Goal-start poses | MPPI {{{', '.join(FOCUS_3)}}} | TTC extreme")
    ax.legend(loc="upper right", fontsize=9)
    plt.tight_layout()
    plt.savefig(out_path, dpi=140)
    plt.close()


def main():
    if not GOALS_CSV.exists():
        print(f"ERROR: {GOALS_CSV} missing -- run spatial_bin_failures.py first.")
        sys.exit(1)

    print("Loading map and wall-distance transform...")
    img, extent, dist_wall_m, res, ox, oy = load_map_and_wall_distance()

    print("Loading goals...")
    df = pd.read_csv(GOALS_CSV)
    df["fail"] = df["Goal Result"].astype(str).str.upper().isin(FAIL_TOKENS)
    rows, cols = world_to_rowcol(
        df["Start Pose_X (m)"].values,
        df["Start Pose_Y (m)"].values,
        res,
        ox,
        oy,
        img.shape,
    )
    df["d_wall_m"] = dist_wall_m[rows, cols]

    pooled = df[
        df["profile"].isin(FOCUS_5) & (df["mode"] == "ttc") & (df["level"] == "extreme")
    ].copy()
    q_lo = float(pooled["d_wall_m"].quantile(QUARTILE_LOW))
    q_hi = float(pooled["d_wall_m"].quantile(QUARTILE_HIGH))
    print(f"  Quartile thresholds (from n={len(pooled)} pooled-5MPPI extreme TTC):")
    print(f"    NARROW <= {q_lo:.2f} m  (bottom 25%)")
    print(f"    OPEN   >= {q_hi:.2f} m  (top 25%)")

    render_distance_overlay(
        img, extent, dist_wall_m, q_lo, q_hi, RESULTS / "distance_to_wall_overlay.png"
    )
    render_scatter_overlay(
        img, extent, dist_wall_m, df, q_lo, q_hi, RESULTS / "goal_scatter_overlay.png"
    )

    def label_zone(d):
        if d <= q_lo:
            return "NARROW"
        if d >= q_hi:
            return "OPEN"
        return "MID"

    df["zone_q"] = df["d_wall_m"].apply(label_zone)

    summary_rows = []

    def summarise(name, sub_df):
        op = sub_df[sub_df["zone_q"] == "OPEN"]
        na = sub_df[sub_df["zone_q"] == "NARROW"]
        mi = sub_df[sub_df["zone_q"] == "MID"]
        if len(op) < 2 or len(na) < 2:
            return None
        of_, on_ = int(op["fail"].sum()), len(op)
        nf_, nn_ = int(na["fail"].sum()), len(na)
        mf_, mn_ = int(mi["fail"].sum()), len(mi)
        odds, p1, p2 = fisher_pair(of_, on_, nf_, nn_)
        olo, ohi = wilson_ci(of_, on_)
        nlo, nhi = wilson_ci(nf_, nn_)
        return {
            "scope": name,
            "n_pooled": len(sub_df),
            "open_fail": of_,
            "open_n": on_,
            "open_pct": round(100 * of_ / on_, 1),
            "open_wilson_lo_pct": round(100 * olo, 1),
            "open_wilson_hi_pct": round(100 * ohi, 1),
            "narrow_fail": nf_,
            "narrow_n": nn_,
            "narrow_pct": round(100 * nf_ / nn_, 1),
            "narrow_wilson_lo_pct": round(100 * nlo, 1),
            "narrow_wilson_hi_pct": round(100 * nhi, 1),
            "mid_fail": mf_,
            "mid_n": mn_,
            "mid_pct": round(100 * mf_ / mn_, 1) if mn_ > 0 else None,
            "fisher_one_sided": round(p1, 5),
            "fisher_two_sided": round(p2, 5),
            "odds_ratio": round(odds, 3),
        }

    print()
    print("=== Per-profile and pooled NARROW vs OPEN at TTC extreme ===")
    print()

    extreme_ttc = df[(df["mode"] == "ttc") & (df["level"] == "extreme")]

    for prof in FOCUS_5:
        sub = extreme_ttc[extreme_ttc["profile"] == prof]
        row = summarise(prof, sub)
        if row is None:
            continue
        summary_rows.append(row)
        print(
            f"  {prof:<32}  open {row['open_fail']}/{row['open_n']} "
            f"({row['open_pct']}%)  narrow {row['narrow_fail']}/{row['narrow_n']} "
            f"({row['narrow_pct']}%)  p1={row['fisher_one_sided']:.4f}  "
            f"p2={row['fisher_two_sided']:.4f}  OR={row['odds_ratio']:.2f}"
        )

    pooled3 = extreme_ttc[extreme_ttc["profile"].isin(FOCUS_3)]
    row3 = summarise("POOLED_3MPPI", pooled3)
    if row3:
        summary_rows.append(row3)
        print(
            f"  {'POOLED_3MPPI':<32}  open {row3['open_fail']}/{row3['open_n']} "
            f"({row3['open_pct']}%)  narrow {row3['narrow_fail']}/{row3['narrow_n']} "
            f"({row3['narrow_pct']}%)  p1={row3['fisher_one_sided']:.4f}  "
            f"p2={row3['fisher_two_sided']:.4f}  OR={row3['odds_ratio']:.2f}"
        )

    pooled5 = extreme_ttc[extreme_ttc["profile"].isin(FOCUS_5)]
    row5 = summarise("POOLED_5MPPI", pooled5)
    if row5:
        summary_rows.append(row5)
        print(
            f"  {'POOLED_5MPPI':<32}  open {row5['open_fail']}/{row5['open_n']} "
            f"({row5['open_pct']}%)  narrow {row5['narrow_fail']}/{row5['narrow_n']} "
            f"({row5['narrow_pct']}%)  p1={row5['fisher_one_sided']:.4f}  "
            f"p2={row5['fisher_two_sided']:.4f}  OR={row5['odds_ratio']:.2f}"
        )

    out_csv = RESULTS / "zone_quartile_fisher.csv"
    pd.DataFrame(summary_rows).to_csv(out_csv, index=False)
    print()
    print(f"Wrote {out_csv}")
    print(f"Wrote {RESULTS / 'distance_to_wall_overlay.png'}")
    print(f"Wrote {RESULTS / 'goal_scatter_overlay.png'}")


if __name__ == "__main__":
    main()
