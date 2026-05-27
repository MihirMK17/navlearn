#!/usr/bin/env python3
"""Aggregate the Phase-0 sensor-rate degradation sweep for Paper 1 Section 6.

Reads the per-run NavLearn metric CSVs under
``results/phase3_sensorrate_baseline/{10,5,2,1}hz/`` (RPP baseline profile,
amcl_phase2, clean regime: no bad-init, no kidnap) and reports per-rate
aggregates over the 25 episodes (5 runs x 5 seeded goals) at each LiDAR
publish rate.

Because the run is clean-mode, Nav2-reported success is the trivial axis; the
informative axis is the *quality* degradation (navigation time, SPL, estimator
accuracy, clearance) as the scan rate is starved from 10 Hz to 1 Hz.

Subscribers: none (offline CSV reader).
Publishers: none.
Parameters:
    --root  results directory holding the per-rate subdirectories.
"""

import argparse
import csv
import glob
import math
import os
import statistics

RATES = ["10hz", "5hz", "2hz", "1hz"]

# CSV header -> short key. Column names are matched verbatim against the header
# row emitted by metrics_compiler.
COLUMNS = {
    "Goal Result": "result",
    "Nav Time (sec)": "nav_time",
    "Path Length (m)": "path_len",
    "SPL": "spl",
    "Control Energy": "energy",
    "Tracking RMS_V (m/s)": "trk_v",
    "Tracking RMS_W (rad/s)": "trk_w",
    "Absolute Path Error RMS (m)": "ate",
    "Relative Pose Error (Drift)": "rpe",
    "Min Clearance (m)": "clearance",
    "Collision Count": "collisions",
}


def _to_float(value):
    """Parse a CSV cell to float, returning None on blank/non-numeric."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_rate(rate_dir):
    """Return a list of per-episode dicts for one rate directory."""
    episodes = []
    for path in sorted(glob.glob(os.path.join(rate_dir, "*.csv"))):
        with open(path, newline="") as handle:
            for row in csv.DictReader(handle):
                episode = {"result": row.get("Goal Result", "").strip()}
                for header, key in COLUMNS.items():
                    if key == "result":
                        continue
                    episode[key] = _to_float(row.get(header))
                episodes.append(episode)
    return episodes


def summarize(episodes):
    """Compute success rate plus mean/std for each numeric quality metric."""
    n = len(episodes)
    successes = sum(1 for e in episodes if e["result"] == "SUCCEEDED")
    out = {"n": n, "success_pct": 100.0 * successes / n if n else float("nan")}
    for key in ("nav_time", "path_len", "spl", "energy", "trk_v", "trk_w",
                "ate", "rpe", "clearance", "collisions"):
        vals = [e[key] for e in episodes if e[key] is not None]
        if vals:
            out[key] = (statistics.mean(vals),
                        statistics.pstdev(vals) if len(vals) > 1 else 0.0)
        else:
            out[key] = (float("nan"), float("nan"))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        default="results/phase3_sensorrate_baseline",
        help="directory containing the per-rate subdirectories")
    args = parser.parse_args()

    print(f"{'Rate':>6} {'n':>3} {'Succ%':>6} {'NavTime[s]':>16} "
          f"{'SPL':>14} {'ATE[m]':>14} {'RPE':>14} {'Clear[m]':>14} "
          f"{'Energy':>16} {'Coll':>5}")
    summaries = {}
    for rate in RATES:
        rate_dir = os.path.join(args.root, rate)
        if not os.path.isdir(rate_dir):
            print(f"{rate:>6}  (missing: {rate_dir})")
            continue
        s = summarize(load_rate(rate_dir))
        summaries[rate] = s

        def cell(key, fmt="{:.2f}"):
            m, sd = s[key]
            if math.isnan(m):
                return f"{'--':>14}"
            return f"{fmt.format(m)}±{fmt.format(sd)}"

        print(f"{rate:>6} {s['n']:>3} {s['success_pct']:>6.1f} "
              f"{cell('nav_time'):>16} {cell('spl','{:.3f}'):>14} "
              f"{cell('ate','{:.3f}'):>14} {cell('rpe','{:.3f}'):>14} "
              f"{cell('clearance','{:.3f}'):>14} {cell('energy','{:.1f}'):>16} "
              f"{int(s['collisions'][0]) if not math.isnan(s['collisions'][0]) else '--':>5}")

    # Relative degradation 10 Hz -> 1 Hz for the headline metrics.
    if "10hz" in summaries and "1hz" in summaries:
        print("\nDegradation 10 Hz -> 1 Hz (mean):")
        for key, label in (("nav_time", "Nav time"), ("spl", "SPL"),
                           ("ate", "ATE"), ("clearance", "Min clearance"),
                           ("energy", "Control energy")):
            a = summaries["10hz"][key][0]
            b = summaries["1hz"][key][0]
            if a and not math.isnan(a) and not math.isnan(b):
                pct = 100.0 * (b - a) / a
                print(f"  {label:>14}: {a:.3f} -> {b:.3f}  ({pct:+.1f}%)")


if __name__ == "__main__":
    main()
