#!/usr/bin/env python3
"""Spatial-bin failure analysis across all NavLearn phases + profiles + levels.

Walks `results/` tree, parses navlearn_metrics_*.csv wide-format sections,
extracts (Start Pose X/Y, Goal Result) per goal, bins into 1 m × 1 m grid
over the small_house map, computes failure rate per cell, and emits
multi-panel heatmaps overlaid on the occupancy grid plus per-phase
summary CSVs.
"""

import re
import sys
import warnings
from io import StringIO
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import yaml
from matplotlib.colors import LinearSegmentedColormap
from PIL import Image

warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")

ROOT = Path("/home/mihirmk/robot_ws")
RESULTS = ROOT / "results"
MAP_YAML = ROOT / "src/bumperbot_mapping/maps/small_house/map.yaml"
MAP_PGM = ROOT / "src/bumperbot_mapping/maps/small_house/map.pgm"
OUT = RESULTS / "spatial_bin_analysis"
OUT.mkdir(exist_ok=True, parents=True)

PHASE_DIR_PATTERNS = [
    (re.compile(r"^phase1_(ttc|ttr)$"), "phase1", "rpp_pre_tune"),
    (re.compile(r"^phase2_(ttc|ttr)$"), "phase2", "rpp_phase2_tuned"),
    (re.compile(r"^phase2b_(ttc|ttr)$"), "phase2b", "rpp_phase2b"),
    (re.compile(r"^phase3_mppi_baseline_(ttc|ttr)$"), "phase3", "mppi_baseline"),
    (re.compile(r"^phase3_mppi_aggressive_(ttc|ttr)$"), "phase3", "mppi_aggressive"),
    (
        re.compile(r"^phase3_mppi_baseline_fixed_bt_(ttc|ttr)$"),
        "phase3",
        "mppi_baseline_fixed_bt",
    ),
]
LEVEL_DIRS = {"easy", "medium", "hard", "extreme"}
ABLATION_PAT = re.compile(
    r"^(mppi_baseline_high_(?:tolerance|vx))_(ttc|ttr)_(easy|medium|hard|extreme)_seed(\d+)$"
)
SEEDREPL_PAT = re.compile(
    r"^(mppi_(?:baseline|aggressive))_(ttc|ttr)_(easy|medium|hard|extreme)_seed(\d+)$"
)

FAIL_TOKENS = {"FAILED", "ABORTED", "CANCELED", "CANCELLED", "TIMED_OUT"}


def find_wide_section(csv_path: Path) -> pd.DataFrame:
    try:
        text = csv_path.read_text(errors="replace")
    except Exception:
        return pd.DataFrame()
    lines = text.splitlines()
    header_idx = None
    for i, line in enumerate(lines):
        if line.startswith("Goal_ID,"):
            header_idx = i
            break
    if header_idx is None:
        return pd.DataFrame()
    try:
        return pd.read_csv(StringIO("\n".join(lines[header_idx:])))
    except Exception as exc:
        print(f"  WARN: failed parse {csv_path.name}: {exc}", file=sys.stderr)
        return pd.DataFrame()


def parse_phase_dir(name: str):
    for pat, phase, profile in PHASE_DIR_PATTERNS:
        m = pat.match(name)
        if m:
            return phase, profile, m.group(1)
    return None


def collect_all_goals() -> pd.DataFrame:
    chunks = []

    for sub in sorted(RESULTS.iterdir()):
        if not sub.is_dir():
            continue
        info = parse_phase_dir(sub.name)
        if info is None:
            continue
        phase, profile, mode = info
        for level_dir in sub.iterdir():
            if not level_dir.is_dir() or level_dir.name not in LEVEL_DIRS:
                continue
            level = level_dir.name
            for csv in level_dir.glob("navlearn_metrics_*.csv"):
                df = find_wide_section(csv)
                if df.empty:
                    continue
                df = df.assign(
                    phase=phase,
                    profile=profile,
                    mode=mode,
                    level=level,
                    seed=42,
                    source_csv=csv.name,
                )
                chunks.append(df)

    for root_name, pat, default_phase in [
        ("phase3_ablation_tolerance", ABLATION_PAT, "phase3"),
        ("phase3_ablation_vx", ABLATION_PAT, "phase3"),
        ("phase3_seed_repl", SEEDREPL_PAT, "phase3"),
    ]:
        root = RESULTS / root_name
        if not root.exists():
            continue
        for sub in sorted(root.iterdir()):
            if not sub.is_dir():
                continue
            m = pat.match(sub.name)
            if not m:
                continue
            profile, mode, level, seed = (
                m.group(1),
                m.group(2),
                m.group(3),
                int(m.group(4)),
            )
            for csv in sub.glob("navlearn_metrics_*.csv"):
                df = find_wide_section(csv)
                if df.empty:
                    continue
                df = df.assign(
                    phase=default_phase,
                    profile=profile,
                    mode=mode,
                    level=level,
                    seed=seed,
                    source_csv=csv.name,
                )
                chunks.append(df)

    if not chunks:
        return pd.DataFrame()

    big = pd.concat(chunks, ignore_index=True)
    for col in ("Start Pose_X (m)", "Start Pose_Y (m)"):
        big[col] = pd.to_numeric(big[col], errors="coerce")
    big = big.dropna(subset=["Start Pose_X (m)", "Start Pose_Y (m)", "Goal Result"])
    big["is_failure"] = big["Goal Result"].astype(str).str.upper().isin(FAIL_TOKENS)
    return big


def bin_grid(df: pd.DataFrame, bin_size: float = 1.0) -> pd.DataFrame:
    d = df.copy()
    d["bx"] = np.floor(d["Start Pose_X (m)"] / bin_size) * bin_size
    d["by"] = np.floor(d["Start Pose_Y (m)"] / bin_size) * bin_size
    g = d.groupby(["bx", "by"], as_index=False).agg(
        attempts=("is_failure", "size"),
        failures=("is_failure", "sum"),
    )
    g["failure_rate"] = g["failures"] / g["attempts"]
    return g


def load_map_meta():
    with open(MAP_YAML) as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(MAP_PGM))
    H, W = img.shape
    res = meta["resolution"]
    ox, oy = meta["origin"][0], meta["origin"][1]
    extent = [ox, ox + W * res, oy, oy + H * res]
    return img, extent


def render_heatmap(
    df_goals: pd.DataFrame,
    title: str,
    out_path: Path,
    bin_size: float = 1.0,
    min_attempts: int = 2,
):
    img, extent = load_map_meta()
    binned = bin_grid(df_goals, bin_size=bin_size)
    if binned.empty:
        return
    binned = binned[binned["attempts"] >= min_attempts]
    if binned.empty:
        return

    fig, ax = plt.subplots(figsize=(11, 9))
    ax.imshow(
        img, extent=extent, origin="lower", cmap="gray", alpha=0.55, vmin=0, vmax=255
    )

    cmap = LinearSegmentedColormap.from_list("fail", ["#1a9850", "#fee08b", "#d73027"])
    for _, row in binned.iterrows():
        rect = patches.Rectangle(
            (row["bx"], row["by"]),
            bin_size,
            bin_size,
            linewidth=0.4,
            edgecolor="black",
            facecolor=cmap(row["failure_rate"]),
            alpha=0.78,
        )
        ax.add_patch(rect)
        ax.text(
            row["bx"] + bin_size / 2,
            row["by"] + bin_size / 2,
            f"{int(100 * row['failure_rate'])}%\nn={int(row['attempts'])}",
            ha="center",
            va="center",
            fontsize=6.5,
            color="black",
            fontweight="bold",
        )

    extent_data = (
        df_goals["Start Pose_X (m)"].min() - 1.0,
        df_goals["Start Pose_X (m)"].max() + 1.0,
        df_goals["Start Pose_Y (m)"].min() - 1.0,
        df_goals["Start Pose_Y (m)"].max() + 1.0,
    )
    pad_x = max(extent_data[1] - extent_data[0], 12)
    pad_y = max(extent_data[3] - extent_data[2], 8)
    cx = 0.5 * (extent_data[0] + extent_data[1])
    cy = 0.5 * (extent_data[2] + extent_data[3])
    ax.set_xlim(cx - pad_x / 2, cx + pad_x / 2)
    ax.set_ylim(cy - pad_y / 2, cy + pad_y / 2)
    ax.set_xlabel("x (m, map frame)")
    ax.set_ylabel("y (m, map frame)")
    ax.set_title(title, fontsize=11)
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=0, vmax=1))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, fraction=0.04, pad=0.04)
    cbar.set_label("Failure rate per 1 m × 1 m cell")
    plt.tight_layout()
    plt.savefig(out_path, dpi=140)
    plt.close()


def main():
    print("Walking results tree...")
    big = collect_all_goals()
    if big.empty:
        print("No data found.")
        return
    print(f"Loaded {len(big)} goal-level records")
    print(f"  phases   = {sorted(big['phase'].unique())}")
    print(f"  profiles = {sorted(big['profile'].unique())}")
    print(f"  modes    = {sorted(big['mode'].unique())}")
    print(f"  levels   = {sorted(big['level'].unique())}")
    print(f"  seeds    = {sorted(big['seed'].unique())}")

    summary = (
        big.groupby(["phase", "profile", "mode", "level"])
        .agg(
            n_goals=("is_failure", "size"),
            n_failures=("is_failure", "sum"),
        )
        .reset_index()
    )
    summary["failure_rate_pct"] = (
        100.0 * summary["n_failures"] / summary["n_goals"]
    ).round(1)
    summary_path = OUT / "spatial_bin_summary.csv"
    summary.to_csv(summary_path, index=False)
    print(f"\nWrote {summary_path}")
    print(summary.to_string(index=False))

    print("\nRendering heatmaps...")
    # 1) All profiles × mode × level (the punchline panel)
    for mode in ("ttc", "ttr"):
        for level in ("extreme", "hard", "medium", "easy"):
            subset = big[(big["mode"] == mode) & (big["level"] == level)]
            if not subset.empty:
                render_heatmap(
                    subset,
                    f"ALL profiles · mode={mode} · level={level} · n={len(subset)}",
                    OUT / f"heatmap_ALL_{mode}_{level}.png",
                )

    # 2) Per (profile, mode), all levels combined
    for (profile, mode), grp in big.groupby(["profile", "mode"]):
        if len(grp) < 5:
            continue
        render_heatmap(
            grp,
            f"profile={profile} · mode={mode} · all levels · n={len(grp)}",
            OUT / f"heatmap_{profile}_{mode}_alllevels.png",
        )

    # 3) Per (profile, mode, level)
    for (profile, mode, level), grp in big.groupby(["profile", "mode", "level"]):
        if len(grp) < 5:
            continue
        render_heatmap(
            grp,
            f"profile={profile} · mode={mode} · level={level} · n={len(grp)}",
            OUT / f"heatmap_{profile}_{mode}_{level}.png",
        )

    big_out = OUT / "all_goals_pose_outcome.csv"
    keep_cols = [
        "phase",
        "profile",
        "mode",
        "level",
        "seed",
        "Goal_ID",
        "Start Pose_X (m)",
        "Start Pose_Y (m)",
        "Start Pose_Yaw (deg)",
        "Goal Result",
        "Goal Result Code",
        "Nav Time (sec)",
        "source_csv",
    ]
    keep_cols = [c for c in keep_cols if c in big.columns]
    big[keep_cols].to_csv(big_out, index=False)
    print(f"Wrote per-goal CSV: {big_out}  ({len(big)} rows)")

    print(f"\nDone. Output in {OUT}")


if __name__ == "__main__":
    main()
