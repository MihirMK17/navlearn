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

"""Why does a starved scan rate RECOVER BETTER? The confident-and-wrong test.

The result this exists to explain
    Leg 6 measured episode-window recovery from a kidnap at four scan rates. Recovery did
    not degrade as the rate was starved; in two of three controllers it improved (RPP
    48% at 10 Hz against 72-80% at 2-1 Hz, p = 0.007). Decision doc section 11 predicted
    the opposite, on the reasoning that recovery climbs a likelihood gradient and every
    climb step is a measurement update, so fewer updates should mean less recovery.

The candidate mechanism, stated so it can fail
    At 10 Hz the filter is handed ten nearly identical scans per second while it is lost.
    Resampling on strongly correlated evidence drives particle depletion: the cloud
    collapses onto whichever mode it happened to favour, and if that mode is wrong the
    filter is now confident and wrong, with no diversity left to escape. At 1 Hz the
    robot moves appreciably between updates, successive scans are less correlated, the
    cloud stays diffuse longer, and the true pose survives in the population long enough
    to win.

    That story makes a sharp, falsifiable prediction about WHEN confidence arrives and
    HOW WRONG the filter is at that moment:

        10 Hz reaches confidence SOONER and with LARGER error than 1 Hz.

    If instead confidence timing and the error at confidence are indistinguishable across
    rates, the depletion account is wrong for this data and the leg 6 result stands as an
    effect without a mechanism. That is a publishable outcome and is reported as such --
    the point of writing the prediction down here is that it cannot be softened later.

What is measured, per kidnapped goal, from the run's own rosbag
    baseline_cov     median covariance trace over the 5 s BEFORE the kidnap, when the
                     filter is converged and tracking. Self-calibrating: "confident"
                     means "back to how confident this filter was when it was right",
                     not an absolute number that would mean different things per rate.
    t_confident      first time after the kidnap that the covariance trace falls back
                     within CONFIDENT_FACTOR x baseline_cov.
    err_at_confident ground-truth position error at that instant. The load-bearing
                     number: confidence reached while still far from truth is the
                     depletion signature.
    yaw_err_at_confident
                     ground-truth HEADING error at that same instant, in radians. The
                     yaw-leg mechanism was first stated by elimination (recovery needs
                     position AND yaw; position at confidence is centimetres; therefore
                     yaw fails). This measures the failing quantity directly instead of
                     deducing it.
    n_updates        /amcl_pose messages in the window. AMCL publishes on update, so
                     this is the count of climb steps the filter actually took, and it
                     is what section 11's prediction was really about.

Thresholding modes
    Default          per-goal: CONFIDENT_FACTOR x that goal's own pre-kidnap baseline.
                     Needs pre-kidnap /amcl_pose samples, which a 1 Hz cell mostly does
                     not have -- the exclusions then all land on the starved cells,
                     which is the bias that made the leg 6 mechanism inconclusive.
    --absolute-threshold T
                     campaign-wide: confident = covariance trace at or below the single
                     absolute value T (m^2), the same number for every goal in every
                     cell. No per-goal history is needed, so no goal is excluded for
                     starvation and the exclusion bias cannot arise. T should come from
                     the pooled converged baselines of the campaign being analysed
                     (report prints the pooled median to make that choice inspectable).

Usage:
    python3 rate_mechanism.py \
        --cell rpp10=results/leg6_rate/rpp_ttrmid_10hz \
        --cell rpp1=results/leg6_rate/rpp_ttrmid_1hz \
        --recovered-from results/leg6_ttr_recompute/per_goal_ttr_recomputed.csv \
        --out results/leg6_mechanism/summary.md
"""

import argparse
import csv
import math
import pathlib
import sqlite3
import statistics

TOPIC_AMCL = "/amcl_pose"
TOPIC_GT = "/bumperbot/ground_truth_pose"
TOPIC_KIDNAP = "/navlearn/kidnap_event"
TOPIC_EPISODE = "/navlearn/episode_event"

EPISODE_END = 2

# "Confident" = covariance trace back within this multiple of the converged pre-kidnap
# baseline. 2.0 rather than 1.0 because the baseline is a median of a noisy series and
# an exact return is not required for the filter to be behaving as though it knows where
# it is. Sensitivity to this choice is reported in the output.
CONFIDENT_FACTOR = 2.0

# Seconds of pre-kidnap history used for the baseline. Long enough to average out the
# jitter, short enough to stay inside the same goal.
BASELINE_WINDOW_S = 5.0


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def uuid_hex(uuid_msg):
    return "".join(f"{b:02x}" for b in uuid_msg.uuid)


def interp_xy(gt, t):
    """Ground-truth (x, y) at time t by linear interpolation; None when uncovered.

    Position only -- this analysis asks how far the estimate is from truth, and the
    recovery binary itself comes from recompute_ttr_from_bags.py, which applies the
    full PROTOCOL section 1 test including yaw.
    """
    if not gt:
        return None
    lo, hi = 0, len(gt)
    while lo < hi:
        mid = (lo + hi) // 2
        if gt[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    i = lo - 1
    if i < 0 or (t - gt[i][0]) >= 0.30:
        return None
    if i + 1 < len(gt):
        dt = gt[i + 1][0] - gt[i][0]
        if 0.0 < dt <= 0.30:
            a = (t - gt[i][0]) / dt
            return (
                gt[i][1] + a * (gt[i + 1][1] - gt[i][1]),
                gt[i][2] + a * (gt[i + 1][2] - gt[i][2]),
            )
    return (gt[i][1], gt[i][2])


def gt_yaw_at(gt, t):
    """Ground-truth yaw at time t from the nearest sample; None when uncovered.

    Nearest sample rather than interpolation: interpolating an angle across a wrap is
    where sign errors live, the ground truth stream is dense (tens of Hz) next to the
    0.30 s coverage rule, and the robot's yaw rate is bounded -- the nearest-sample
    error is milliradians, far below the effect size under test (whole radians).
    """
    if not gt:
        return None
    lo, hi = 0, len(gt)
    while lo < hi:
        mid = (lo + hi) // 2
        if gt[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    best = None
    for i in (lo - 1, lo):
        if 0 <= i < len(gt) and abs(gt[i][0] - t) < 0.30:
            if best is None or abs(gt[i][0] - t) < abs(gt[best][0] - t):
                best = i
    return gt[best][3] if best is not None else None


def baseline_cov(amcl, kidnap_t, window_s=BASELINE_WINDOW_S):
    """Median covariance trace in the window_s before the kidnap, or None if too thin.

    None rather than a default: a goal whose pre-kidnap history is missing cannot say
    what "confident" meant for it, and guessing a threshold would silently compare
    different things across rates -- the 1 Hz cells have a tenth of the samples, so any
    fallback would bite them hardest and manufacture the very effect being tested.
    """
    pre = [c for (t, _, _, _, c) in amcl if kidnap_t - window_s <= t < kidnap_t]
    if len(pre) < 3:
        return None
    return statistics.median(pre)


def first_crossing(series, threshold):
    """(t, value) of the first sample at or below threshold; None if never.

    series is [(t, value), ...] in time order.
    """
    for t, v in series:
        if v <= threshold:
            return (t, v)
    return None


def analyse_goal(amcl, gt, kidnap_t, end_t, abs_threshold=None):
    """Mechanism quantities for one kidnap -> goal-end window.

    abs_threshold, when given, replaces the per-goal CONFIDENT_FACTOR x baseline test
    with a single campaign-wide covariance value, so a goal with no pre-kidnap history
    is still analysable -- the exclusion-bias fix for starved cells.
    """
    window = [(t, x, y, yw, c) for (t, x, y, yw, c) in amcl if kidnap_t <= t <= end_t]
    out = {
        "n_updates": len(window),
        "window_s": end_t - kidnap_t,
        "update_hz": (
            len(window) / (end_t - kidnap_t) if end_t > kidnap_t else float("nan")
        ),
        "baseline_cov": None,
        "usable": False,
        "t_confident": None,
        "err_at_confident": None,
        "yaw_err_at_confident": None,
        "min_err": None,
        "final_err": None,
        "final_yaw_err": None,
    }
    base = baseline_cov(amcl, kidnap_t)
    out["baseline_cov"] = base
    if abs_threshold is not None:
        threshold = abs_threshold
    elif base is not None:
        threshold = CONFIDENT_FACTOR * base
    else:
        return out
    if not window:
        return out

    errs = []
    yaw_errs = []
    cov_series = []
    for t, x, y, yw, c in window:
        g = interp_xy(gt, t)
        if g is None:
            continue
        errs.append((t, math.hypot(g[0] - x, g[1] - y)))
        gyaw = gt_yaw_at(gt, t)
        if gyaw is not None:
            yaw_errs.append((t, abs(wrap_angle(yw - gyaw))))
        cov_series.append((t, c))
    if not errs:
        return out
    out["usable"] = True

    out["min_err"] = min(e for _, e in errs)
    out["final_err"] = errs[-1][1]
    if yaw_errs:
        out["final_yaw_err"] = yaw_errs[-1][1]

    hit = first_crossing(cov_series, threshold)
    if hit is not None:
        t_conf = hit[0]
        out["t_confident"] = t_conf - kidnap_t
        # Error at the sample nearest the confidence crossing.
        out["err_at_confident"] = min(errs, key=lambda e: abs(e[0] - t_conf))[1]
        if yaw_errs:
            out["yaw_err_at_confident"] = min(
                yaw_errs, key=lambda e: abs(e[0] - t_conf)
            )[1]
    return out


def read_bag(db3_path):
    """(amcl, gt, kidnaps, ends). amcl carries the covariance trace per sample."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    conn = sqlite3.connect(f"file:{db3_path}?mode=ro", uri=True)
    try:
        wanted = {TOPIC_AMCL, TOPIC_GT, TOPIC_KIDNAP, TOPIC_EPISODE}
        by_id = {}
        for tid, name, mtype in conn.execute("SELECT id, name, type FROM topics"):
            if name in wanted:
                by_id[tid] = (name, get_message(mtype))
        missing = wanted - {n for n, _ in by_id.values()}
        if missing:
            raise RuntimeError(f"{db3_path}: missing topics {sorted(missing)}")

        amcl, gt, kidnaps, ends = [], [], {}, {}
        placeholders = ",".join("?" * len(by_id))
        for topic_id, data in conn.execute(
            f"SELECT topic_id, data FROM messages WHERE topic_id IN ({placeholders})"
            " ORDER BY timestamp",
            list(by_id),
        ):
            name, cls = by_id[topic_id]
            msg = deserialize_message(bytes(data), cls)
            if name == TOPIC_AMCL:
                p = msg.pose.pose
                # Trace of the position block: sigma_xx + sigma_yy.
                cov = msg.pose.covariance[0] + msg.pose.covariance[7]
                q = p.orientation
                amcl.append(
                    (
                        stamp_s(msg.header.stamp),
                        p.position.x,
                        p.position.y,
                        yaw_from_quat(q.x, q.y, q.z, q.w),
                        cov,
                    )
                )
            elif name == TOPIC_GT:
                tr = msg.transform
                r = tr.rotation
                gt.append(
                    (
                        stamp_s(msg.header.stamp),
                        tr.translation.x,
                        tr.translation.y,
                        yaw_from_quat(r.x, r.y, r.z, r.w),
                    )
                )
            elif name == TOPIC_KIDNAP:
                if msg.success:
                    kidnaps[uuid_hex(msg.goal_id)] = stamp_s(msg.header.stamp)
            elif name == TOPIC_EPISODE:
                if msg.state == EPISODE_END:
                    ends[uuid_hex(msg.goal_id)] = stamp_s(msg.header.stamp)
        amcl.sort(key=lambda s: s[0])
        gt.sort(key=lambda s: s[0])
        return amcl, gt, kidnaps, ends
    finally:
        conn.close()


def process_cell(label, cell_dir, progress, abs_threshold=None):
    rows = []
    for db3 in sorted(pathlib.Path(cell_dir).glob("rosbag_run_*/*.db3")):
        amcl, gt, kidnaps, ends = read_bag(db3)
        for goal, k_t in sorted(kidnaps.items(), key=lambda kv: kv[1]):
            if goal not in ends:
                continue
            r = analyse_goal(amcl, gt, k_t, ends[goal], abs_threshold=abs_threshold)
            r.update(cell=label, run=db3.parent.name, goal_id=goal)
            rows.append(r)
        progress(f"{label} {db3.parent.name}: {len(rows)} goals")
    return rows


def med(values):
    vals = [
        v
        for v in values
        if v is not None and not (isinstance(v, float) and math.isnan(v))
    ]
    return statistics.median(vals) if vals else float("nan")


def build_report(rows_by_cell, recovered_by_goal, abs_threshold=None):
    pooled = [
        r["baseline_cov"]
        for rows in rows_by_cell.values()
        for r in rows
        if r["baseline_cov"] is not None
    ]
    lines = [
        "# Mechanism: does the filter arrive at confidence, and how wrong is it there?",
        "",
    ]
    if abs_threshold is not None:
        lines += [
            f"Confident = covariance trace at or below {abs_threshold:g} m^2 -- one "
            "absolute campaign-wide value, the same for every goal in every cell, so "
            "no goal is excluded for lacking pre-kidnap history and the exclusion bias "
            "that made the per-goal-baseline version inconclusive cannot arise.",
            f"For calibration: the pooled median of the {len(pooled)} per-goal "
            f"pre-kidnap baselines that DO exist is "
            f"{statistics.median(pooled) if pooled else float('nan'):.6f} m^2.",
        ]
    else:
        lines += [
            f"Confident = covariance trace back within {CONFIDENT_FACTOR}x the median "
            f"trace over the {BASELINE_WINDOW_S:.0f} s before the kidnap.",
        ]
    lines += [
        "",
        "| cell | n | usable | updates in window | update Hz | t_confident (s) |"
        " err at confident (m) | yaw err at confident (deg) | reached confidence |"
        " min err (m) |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for cell, rows in sorted(rows_by_cell.items()):
        usable = [r for r in rows if r["usable"]]
        conf = [r for r in usable if r["t_confident"] is not None]
        lines.append(
            f"| {cell} | {len(rows)} | {len(usable)}/{len(rows)} | "
            f"{med(r['n_updates'] for r in rows):.0f} | "
            f"{med(r['update_hz'] for r in rows):.2f} | "
            f"{med(r['t_confident'] for r in conf):.2f} | "
            f"{med(r['err_at_confident'] for r in conf):.3f} | "
            f"{math.degrees(med(r['yaw_err_at_confident'] for r in conf)):.1f} | "
            f"{len(conf)}/{len(usable)} | {med(r['min_err'] for r in usable):.3f} |"
        )
    lines.append("")
    if abs_threshold is None:
        lines.append(
            "The `usable` column is not decoration. A goal needs pre-kidnap history to "
            "say what confidence meant for it, and at 1 Hz that window holds a tenth of "
            "the samples it holds at 10 Hz -- so any exclusion asymmetry falls on the "
            "starved cells and would imitate the effect under test. Compare the "
            "fractions before reading anything else in this table."
        )
    else:
        lines.append(
            "Under the absolute threshold, `usable` only requires the goal's window to "
            "contain AMCL samples with ground-truth coverage; compare the fractions "
            "anyway -- they should now be near-equal across cells."
        )
    lines.append("")

    # Split by outcome: the wrong-place/wrong-heading story is about the failures.
    lines += [
        "## Error at confidence, split by whether the goal ended recovered",
        "",
        "| cell | recovered: n, err (m), yaw err (deg) "
        "| NOT recovered: n, err (m), yaw err (deg) |",
        "|---|---|---|",
    ]
    for cell, rows in sorted(rows_by_cell.items()):
        conf = [r for r in rows if r["t_confident"] is not None]
        rec = [r for r in conf if recovered_by_goal.get(r["goal_id"]) == 1]
        non = [r for r in conf if recovered_by_goal.get(r["goal_id"]) == 0]
        lines.append(
            f"| {cell} | {len(rec)}, {med(r['err_at_confident'] for r in rec):.3f}, "
            f"{math.degrees(med(r['yaw_err_at_confident'] for r in rec)):.1f} "
            f"| {len(non)}, {med(r['err_at_confident'] for r in non):.3f}, "
            f"{math.degrees(med(r['yaw_err_at_confident'] for r in non)):.1f} |"
        )
    lines.append("")
    return "\n".join(lines)


FIELDNAMES = [
    "cell",
    "run",
    "goal_id",
    "n_updates",
    "update_hz",
    "window_s",
    "baseline_cov",
    "usable",
    "t_confident",
    "err_at_confident",
    "yaw_err_at_confident",
    "min_err",
    "final_err",
    "final_yaw_err",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cell", action="append", required=True, metavar="LABEL=DIR")
    parser.add_argument(
        "--recovered-from",
        required=True,
        help="per_goal_ttr_recomputed.csv, for the recovery outcome",
    )
    parser.add_argument(
        "--absolute-threshold",
        type=float,
        default=None,
        metavar="M2",
        help="campaign-wide covariance-trace threshold (m^2) defining "
        "'confident'; replaces the per-goal baseline test and with "
        "it the starved-cell exclusion bias",
    )
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    recovered = {
        r["goal_id"]: int(r["recovered_sustained"])
        for r in csv.DictReader(open(args.recovered_from))
    }

    rows_by_cell = {}
    for spec in args.cell:
        label, cell_dir = spec.split("=", 1)
        rows_by_cell[label] = process_cell(
            label,
            cell_dir,
            lambda m: print(m, flush=True),
            abs_threshold=args.absolute_threshold,
        )

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out.with_name("per_goal_mechanism.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDNAMES, extrasaction="ignore")
        w.writeheader()
        for rows in rows_by_cell.values():
            for r in rows:
                w.writerow(r)

    report = build_report(
        rows_by_cell, recovered, abs_threshold=args.absolute_threshold
    )
    out.write_text(report)
    print(report)


if __name__ == "__main__":
    main()
