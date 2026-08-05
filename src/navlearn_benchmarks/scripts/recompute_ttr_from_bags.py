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

"""Recompute TTR from the leg 2 rosbags at the full episode window.

Why it exists
    The online evaluator ran with ttr_timeout_sec = 10.0 (launch argument, overriding the
    15.0 in the YAML), so every "TTR Outcome = 0" in the campaign CSVs means "not recovered
    within 10 s" and nothing more -- 80 of the 84 top-bin goals carry exactly that code.
    PROTOCOL section 1 recorded the gap ("the timeout extended to the episode") and it was
    never closed online. The bags hold /amcl_pose, /bumperbot/ground_truth_pose and both
    event topics for the whole episode, so the sustained-to-goal-end outcome the PROTOCOL
    actually defines is computable offline without re-running anything.

Definitions (PROTOCOL section 1; thresholds and the per-sample test replicate
navlearn_localization_eval/src/localization_metrics.cpp)

    ok(sample)      GT interpolated to the amcl stamp; requires |err_x| < 0.20 m,
                    |err_y| < 0.20 m, hypot(err_x, err_y) < 0.20 m and
                    |wrap_to_pi(err_yaw)| < 0.10 rad. Same conjunction as the node.
    recovered       binary, the outcome claim 2 is judged on: the estimate is inside
                    threshold at episode end AND stayed inside from its final entry --
                    i.e. the trailing run of ok samples reaches the last sample before
                    goal end. A filter that touches and drifts away has not recovered.
    ttr_sustained_s kidnap -> start of that trailing ok run. Defined only when recovered.
    first_touch_s   kidnap -> first ok sample. Reported to show how much first-touch
                    flatters the sustained number; never used as an outcome.
    ttr_hold_s      kidnap -> start of the first ok run lasting >= hold seconds (2.0,
                    the node's ttr_hold_sec). This is the node's online quantity, here
                    evaluated over the full episode instead of 10 s.

Fidelity deviations from the node, both deliberate:
    * GT interpolation is plain linear at the amcl stamp. The node shifts the query back
      by half the latest GT inter-sample gap to compensate live pipeline latency; offline
      both stamps are sim time from the same bag, there is no latency to compensate, and
      at ~30 Hz GT the shift is ~17 ms (< 8 mm at campaign speeds).
    * The node stops watching after hold completion or 10 s; this script watches the whole
      kidnap -> goal-end window. That difference is the point.

Exclusions mirror analyze_curve_campaign.py: kidnap events with success == 0 are attrition
(no perturbation delivered, not a TTR trial); goals whose amcl/GT coverage is too thin to
judge are reported, never silently counted either way.

Usage:
    python3 recompute_ttr_from_bags.py \
        --arm rpp=results/leg2_final_rpp --arm dwb=results/leg2_final_dwb \
        --arm mppi=results/leg2_final_mppi \
        --old-per-goal results/leg2_analysis/per_goal.csv \
        --range 0.01:3.0 --output-dir results/leg2_ttr_recompute
"""

import argparse
import csv
import math
import pathlib
import sqlite3
import sys

TOPIC_AMCL = "/amcl_pose"
TOPIC_GT = "/bumperbot/ground_truth_pose"
TOPIC_KIDNAP = "/navlearn/kidnap_event"
TOPIC_EPISODE = "/navlearn/episode_event"

POS_THRESHOLD_M = 0.20
YAW_THRESHOLD_RAD = 0.10
HOLD_SEC = 2.0
GT_MAX_GAP_S = 0.30
GT_MAX_STALE_S = 0.30

EPISODE_END = 2  # EpisodeEvent.END


def wrap_to_pi(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Planar yaw from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def interp_gt(gt, t):
    """GT (x, y, yaw) at time t, or None when coverage does not support it.

    gt is a list of (t, x, y, yaw) sorted by t. Mirrors the node's linear_interpolate:
    take the last sample at or before t; None if it is >= 0.30 s stale; interpolate
    toward the next sample when the gap is <= 0.30 s, else hold the earlier sample.
    """
    if not gt:
        return None
    # Binary search for the last sample with time <= t.
    lo, hi = 0, len(gt)
    while lo < hi:
        mid = (lo + hi) // 2
        if gt[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    i = lo - 1
    if i < 0:
        return None
    if (t - gt[i][0]) >= GT_MAX_STALE_S:
        return None
    if i + 1 < len(gt):
        dt = gt[i + 1][0] - gt[i][0]
        if 0.0 < dt <= GT_MAX_GAP_S:
            alpha = (t - gt[i][0]) / dt
            x = gt[i][1] + alpha * (gt[i + 1][1] - gt[i][1])
            y = gt[i][2] + alpha * (gt[i + 1][2] - gt[i][2])
            dyaw = wrap_to_pi(gt[i + 1][3] - gt[i][3])
            return (x, y, wrap_to_pi(gt[i][3] + alpha * dyaw))
    return (gt[i][1], gt[i][2], gt[i][3])


def sample_ok(err_x: float, err_y: float, err_yaw: float) -> bool:
    """The node's per-sample recovery test, verbatim conjunction."""
    return (
        abs(err_x) < POS_THRESHOLD_M
        and abs(err_y) < POS_THRESHOLD_M
        and math.hypot(err_x, err_y) < POS_THRESHOLD_M
        and abs(wrap_to_pi(err_yaw)) < YAW_THRESHOLD_RAD
    )


def evaluate_window(amcl, gt, kidnap_t: float, end_t: float) -> dict:
    """All recovery quantities for one kidnap -> goal-end window.

    amcl is a list of (t, x, y, yaw); gt as in interp_gt. Returns a dict with the
    quantities in the module docstring plus coverage numbers that let a thin window be
    excluded rather than trusted.
    """
    marks = []  # (t, ok)
    skipped_no_gt = 0
    for (t, ax, ay, ayaw) in amcl:
        if t < kidnap_t or t > end_t:
            continue
        g = interp_gt(gt, t)
        if g is None:
            skipped_no_gt += 1
            continue
        marks.append((t, sample_ok(g[0] - ax, g[1] - ay, g[2] - ayaw)))

    out = {
        "n_samples": len(marks),
        "n_skipped_no_gt": skipped_no_gt,
        "window_s": end_t - kidnap_t,
        "last_sample_gap_s": (end_t - marks[-1][0]) if marks else float("nan"),
        "first_touch_s": None,
        "recovered_sustained": 0,
        "ttr_sustained_s": None,
        "ttr_hold_s": None,
    }
    if not marks:
        return out

    for t, ok in marks:
        if ok:
            out["first_touch_s"] = t - kidnap_t
            break

    # Trailing run of ok samples -> the PROTOCOL binary and its entry time.
    if marks[-1][1]:
        i = len(marks) - 1
        while i > 0 and marks[i - 1][1]:
            i -= 1
        out["recovered_sustained"] = 1
        out["ttr_sustained_s"] = marks[i][0] - kidnap_t

    # First ok run whose span reaches HOLD_SEC -> the node's online quantity.
    run_start = None
    for t, ok in marks:
        if ok:
            if run_start is None:
                run_start = t
            if t - run_start >= HOLD_SEC:
                out["ttr_hold_s"] = run_start - kidnap_t
                break
        else:
            run_start = None
    return out


def uuid_hex(uuid_msg) -> str:
    """unique_identifier_msgs/UUID -> 32-char lowercase hex, per_goal.csv convention."""
    return "".join(f"{b:02x}" for b in uuid_msg.uuid)


def stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def read_bag(db3_path):
    """One pass over a bag: (amcl, gt, kidnaps, episode_ends).

    kidnaps: goal_id_hex -> dict(t, success, realised_m, commanded_m, land_x, land_y,
    land_yaw, reference_available). episode_ends: goal_id_hex -> end time. Header stamps
    (sim time) throughout, matching the node.
    """
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    conn = sqlite3.connect(f"file:{db3_path}?mode=ro", uri=True)
    try:
        topics = {
            name: (tid, get_message(mtype.replace("/msg/", "/msg/")))
            for tid, name, mtype in conn.execute("SELECT id, name, type FROM topics")
            if name in (TOPIC_AMCL, TOPIC_GT, TOPIC_KIDNAP, TOPIC_EPISODE)
        }
        missing = {TOPIC_AMCL, TOPIC_GT, TOPIC_KIDNAP, TOPIC_EPISODE} - set(topics)
        if missing:
            raise RuntimeError(f"{db3_path}: missing topics {sorted(missing)}")

        by_id = {tid: (name, msg_cls) for name, (tid, msg_cls) in topics.items()}
        amcl, gt = [], []
        kidnaps, episode_ends = {}, {}
        placeholders = ",".join("?" * len(by_id))
        cur = conn.execute(
            f"SELECT topic_id, data FROM messages WHERE topic_id IN ({placeholders})"
            " ORDER BY timestamp",
            list(by_id),
        )
        for topic_id, data in cur:
            name, msg_cls = by_id[topic_id]
            msg = deserialize_message(bytes(data), msg_cls)
            if name == TOPIC_AMCL:
                p = msg.pose.pose
                amcl.append((
                    stamp_s(msg.header.stamp), p.position.x, p.position.y,
                    yaw_from_quat(p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w),
                ))
            elif name == TOPIC_GT:
                tr = msg.transform
                gt.append((
                    stamp_s(msg.header.stamp), tr.translation.x, tr.translation.y,
                    yaw_from_quat(tr.rotation.x, tr.rotation.y,
                                  tr.rotation.z, tr.rotation.w),
                ))
            elif name == TOPIC_KIDNAP:
                realised = float("nan")
                if msg.reference_available:
                    realised = math.hypot(
                        msg.kidnap_pose.position.x - msg.reference_pose.position.x,
                        msg.kidnap_pose.position.y - msg.reference_pose.position.y,
                    )
                yaw_change = float("nan")
                if msg.reference_available:
                    yaw_change = wrap_to_pi(
                        yaw_from_quat(
                            msg.kidnap_pose.orientation.x, msg.kidnap_pose.orientation.y,
                            msg.kidnap_pose.orientation.z, msg.kidnap_pose.orientation.w)
                        - yaw_from_quat(
                            msg.reference_pose.orientation.x,
                            msg.reference_pose.orientation.y,
                            msg.reference_pose.orientation.z,
                            msg.reference_pose.orientation.w))
                kidnaps[uuid_hex(msg.goal_id)] = {
                    "t": stamp_s(msg.header.stamp),
                    "success": bool(msg.success),
                    "realised_m": realised,
                    "yaw_change_rad": yaw_change,
                    "commanded_m": msg.commanded_magnitude_m,
                    "land_x": msg.kidnap_pose.position.x,
                    "land_y": msg.kidnap_pose.position.y,
                    "land_yaw": yaw_from_quat(
                        msg.kidnap_pose.orientation.x, msg.kidnap_pose.orientation.y,
                        msg.kidnap_pose.orientation.z, msg.kidnap_pose.orientation.w),
                    "reference_available": bool(msg.reference_available),
                }
            elif name == TOPIC_EPISODE:
                if msg.state == EPISODE_END:
                    episode_ends[uuid_hex(msg.goal_id)] = stamp_s(msg.header.stamp)
        # amcl/gt arrive ordered by bag receive time; sort by header stamp to be exact.
        amcl.sort(key=lambda s: s[0])
        gt.sort(key=lambda s: s[0])
        return amcl, gt, kidnaps, episode_ends
    finally:
        conn.close()


def process_arm(arm: str, arm_dir: pathlib.Path, progress=lambda s: None):
    """Every kidnap goal in every bag of one arm -> list of per-goal dicts."""
    rows, attrition, unmatched = [], 0, 0
    bag_files = sorted(arm_dir.glob("rosbag_run_*/*.db3"))
    if not bag_files:
        raise RuntimeError(f"{arm_dir}: no rosbag_run_*/*.db3 found")
    for db3 in bag_files:
        run = db3.parent.name  # rosbag_run_<N>_<stamp>
        amcl, gt, kidnaps, ends = read_bag(db3)
        for goal_hex, k in sorted(kidnaps.items(), key=lambda kv: kv[1]["t"]):
            if not k["success"]:
                attrition += 1
                continue
            if goal_hex not in ends:
                unmatched += 1
                continue
            r = evaluate_window(amcl, gt, k["t"], ends[goal_hex])
            r.update(
                arm=arm, run=run, goal_id=goal_hex,
                magnitude_m=k["realised_m"], displacement_m=k["realised_m"],
                commanded_m=k["commanded_m"],
                yaw_change_rad=k["yaw_change_rad"],
                land_x=k["land_x"], land_y=k["land_y"], land_yaw=k["land_yaw"],
            )
            rows.append(r)
        progress(f"{arm} {run}: {len(rows)} goals so far "
                 f"(attrition={attrition}, unmatched={unmatched})")
    return rows, attrition, unmatched


def log_bin_edges(lo: float, hi: float, bins: int):
    ratio = (hi / lo) ** (1.0 / bins)
    return [lo * ratio ** k for k in range(bins + 1)]


def linear_bin_edges(lo: float, hi: float, bins: int):
    """The yaw leg samples linearly from zero, which has no logarithm."""
    step = (hi - lo) / bins
    return [lo + step * k for k in range(bins + 1)]


def binned_recovery(rows, edges, key):
    """Per-bin (n, recovered %) for a binary column. Last bin closed."""
    out = []
    for b in range(len(edges) - 1):
        lo, hi = edges[b], edges[b + 1]
        last = b == len(edges) - 2
        sel = [r for r in rows
               if lo <= r["magnitude_m"] < hi or (last and r["magnitude_m"] == hi)]
        n = len(sel)
        pct = 100.0 * sum(r[key] for r in sel) / n if n else float("nan")
        out.append((lo, hi, n, pct))
    return out


def median(values):
    values = sorted(values)
    if not values:
        return float("nan")
    n = len(values)
    return values[n // 2] if n % 2 else 0.5 * (values[n // 2 - 1] + values[n // 2])


def build_report(all_rows, old_by_goal, lo, hi, bins=4, bin_scale="log"):
    edges = (linear_bin_edges if bin_scale == "linear" else log_bin_edges)(lo, hi, bins)
    lines = [
        "# Leg 2 TTR recomputed from rosbags (episode-length window)",
        "",
        "Recovery rule: PROTOCOL section 1 -- inside 0.20 m / 0.10 rad of ground truth,",
        "sustained to goal end. Old numbers were censored at ttr_timeout_sec = 10.0.",
        "",
    ]
    arms = sorted({r["arm"] for r in all_rows})
    for arm in arms:
        rows = [r for r in all_rows if r["arm"] == arm]
        joined = [r for r in rows if r["goal_id"] in old_by_goal]
        old_rec = sum(old_by_goal[r["goal_id"]]["recovered"] for r in joined)
        new_rec = sum(r["recovered_sustained"] for r in rows)
        lines.append(f"## {arm} -- n={len(rows)} kidnapped goals")
        lines.append("")
        lines.append(
            f"overall recovered: censored-10s {100.0 * old_rec / len(joined):.1f}% "
            f"-> episode-window {100.0 * new_rec / len(rows):.1f}%")
        ttrs = [r["ttr_sustained_s"] for r in rows if r["ttr_sustained_s"] is not None]
        touch = [r["first_touch_s"] for r in rows if r["first_touch_s"] is not None]
        lines.append(
            f"median ttr_sustained {median(ttrs):.2f} s over {len(ttrs)} recoveries; "
            f"median first touch {median(touch):.2f} s over {len(touch)} goals; "
            f"median window {median([r['window_s'] for r in rows]):.1f} s")
        lines.append("")
        lines.append("| bin (m) | n | recovered old (10 s) | recovered new (episode) |")
        lines.append("|---|---|---|---|")
        new_b = binned_recovery(rows, edges, "recovered_sustained")
        old_rows = [
            {"magnitude_m": r["magnitude_m"],
             "rec": old_by_goal[r["goal_id"]]["recovered"]}
            for r in joined]
        old_b = binned_recovery(
            [{"magnitude_m": r["magnitude_m"], "recovered_sustained": r["rec"]}
             for r in old_rows], edges, "recovered_sustained")
        for (blo, bhi, n_new, pct_new), (_, _, n_old, pct_old) in zip(new_b, old_b):
            lines.append(
                f"| {blo:.3f}-{bhi:.3f} | {n_new} | {pct_old:.1f}% (n={n_old}) "
                f"| {pct_new:.1f}% |")
        lines.append("")
    return "\n".join(lines)


FIELDNAMES = [
    "arm", "run", "goal_id", "magnitude_m", "displacement_m", "commanded_m",
    "yaw_change_rad",
    "land_x", "land_y", "land_yaw",
    "recovered_sustained", "ttr_sustained_s", "first_touch_s", "ttr_hold_s",
    "window_s", "n_samples", "n_skipped_no_gt", "last_sample_gap_s",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--arm", action="append", required=True,
                        metavar="NAME=DIR", help="arm name and its campaign directory")
    parser.add_argument("--old-per-goal", required=True,
                        help="per_goal.csv from analyze_curve_campaign.py, for the "
                             "censored numbers and the magnitude cross-check")
    parser.add_argument("--range", default="0.01:3.0")
    parser.add_argument("--bins", type=int, default=4)
    parser.add_argument(
        "--magnitude-source", choices=("displacement", "yaw_change_deg"),
        default="displacement",
        help="Which bag quantity is the sweep's independent variable. The yaw-curve leg "
             "(PROTOCOL A3) bins and cross-checks on |yaw change| in degrees; its "
             "realised displacement is zero by design and checking it against a degrees "
             "column would fail every goal.")
    parser.add_argument(
        "--bin-scale", choices=("log", "linear"), default="log",
        help="Bin spacing; the yaw range starts at zero, which has no log")
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    lo, hi = (float(v) for v in args.range.split(":"))
    out_dir = pathlib.Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    old_by_goal = {}
    with open(args.old_per_goal) as f:
        for row in csv.DictReader(f):
            old_by_goal[row["goal_id"]] = {
                "recovered": int(row["recovered"]),
                "magnitude_m": float(row["magnitude_m"]),
            }

    def progress(msg):
        print(msg, flush=True)

    all_rows = []
    counts = {}
    for spec in args.arm:
        arm, arm_dir = spec.split("=", 1)
        rows, attrition, unmatched = process_arm(arm, pathlib.Path(arm_dir), progress)
        counts[arm] = {"goals": len(rows), "attrition": attrition,
                       "unmatched": unmatched}
        all_rows.extend(rows)

    # The yaw leg's independent variable is the rotation: bin and cross-check on
    # |yaw change| in degrees. displacement_m keeps the realised displacement either way.
    if args.magnitude_source == "yaw_change_deg":
        for r in all_rows:
            r["magnitude_m"] = abs(math.degrees(r["yaw_change_rad"]))

    # Cross-check: realised magnitude from the bag must match the campaign CSV.
    mismatches = [
        (r["arm"], r["goal_id"], r["magnitude_m"],
         old_by_goal[r["goal_id"]]["magnitude_m"])
        for r in all_rows
        if r["goal_id"] in old_by_goal
        and not math.isnan(r["magnitude_m"])
        and abs(r["magnitude_m"] - old_by_goal[r["goal_id"]]["magnitude_m"]) > 1e-3
    ]
    joined = sum(r["goal_id"] in old_by_goal for r in all_rows)

    per_goal_path = out_dir / "per_goal_ttr_recomputed.csv"
    with open(per_goal_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES, extrasaction="ignore")
        writer.writeheader()
        for r in sorted(all_rows, key=lambda r: (r["arm"], r["run"], r["goal_id"])):
            writer.writerow(r)

    report = build_report(all_rows, old_by_goal, lo, hi, args.bins, args.bin_scale)
    report += "\n## Bookkeeping\n\n"
    for arm, c in sorted(counts.items()):
        report += (f"- {arm}: {c['goals']} kidnapped goals, "
                   f"{c['attrition']} attrition (kidnap never applied), "
                   f"{c['unmatched']} without a goal END event\n")
    report += (f"- joined to old per_goal.csv: {joined}/{len(all_rows)}\n"
               f"- magnitude cross-check mismatches (>1 mm): {len(mismatches)}\n")
    if mismatches:
        report += "\nMISMATCHES (bag vs campaign CSV) -- do not trust the join:\n"
        for arm, goal, bag_m, csv_m in mismatches[:20]:
            report += f"  {arm} {goal}: bag {bag_m:.4f} vs csv {csv_m:.4f}\n"

    (out_dir / "summary.md").write_text(report)
    print(report)
    print(f"\nwrote {per_goal_path}", flush=True)
    if mismatches:
        print(f"FATAL: {len(mismatches)} magnitude mismatches", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
