#!/usr/bin/env python3
# Copyright 2026 Mihir Kulkarni
# SPDX-License-Identifier: Apache-2.0
"""Render an occupancy map (map.yaml + PGM) as a styled PNG for the README.

Regenerate any map figure with:
    ros2 run navlearn_analysis render_map --map <map.yaml> --title "..." --out <png>
"""

import argparse
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from PIL import Image  # noqa: E402

from navlearn_analysis.figures import palette  # noqa: E402


def load_map(map_yaml):
    """(image array, resolution m/cell) from a map_server YAML."""
    meta = yaml.safe_load(pathlib.Path(map_yaml).read_text())
    img = np.array(Image.open(pathlib.Path(map_yaml).parent / meta["image"]))
    return img, float(meta["resolution"])


def render(map_yaml, title, out_png):
    img, res = load_map(map_yaml)
    occupied_thresh = 255 * 0.35  # PGM: 0 = occupied, 254 = free, 205 = unknown

    rgb = np.empty((*img.shape, 3))
    rgb[:] = matplotlib.colors.to_rgb(palette.UNKNOWN)
    rgb[img > 250] = matplotlib.colors.to_rgb(palette.FREE)
    rgb[img < occupied_thresh] = matplotlib.colors.to_rgb(palette.OCCUPIED)

    h, w = img.shape
    fig, ax = plt.subplots(figsize=(w / 60, h / 60), dpi=150)
    fig.patch.set_facecolor(palette.SURFACE)
    ax.imshow(rgb, interpolation="nearest")
    ax.set_axis_off()
    ax.set_title(title, color=palette.TEXT_PRIMARY, fontsize=13, pad=10)

    # 5 m scale bar, bottom-left, sized in cells from the map's own resolution.
    bar_cells = 5.0 / res
    x0, y0 = w * 0.05, h * 0.95
    ax.plot([x0, x0 + bar_cells], [y0, y0], color=palette.TEXT_PRIMARY, lw=2.5)
    ax.text(
        x0 + bar_cells / 2,
        y0 - h * 0.02,
        "5 m",
        ha="center",
        color=palette.TEXT_SECONDARY,
        fontsize=9,
    )

    out = pathlib.Path(out_png)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, bbox_inches="tight", facecolor=palette.SURFACE)
    plt.close(fig)
    print(f"wrote {out}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--map", required=True, help="path to map.yaml")
    parser.add_argument("--title", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    render(args.map, args.title, args.out)


if __name__ == "__main__":
    main()
