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

"""Measures how far the global costmap has diverged from the static map.

Why this is measured rather than assumed
    While AMCL is wrong, every laser scan is inserted into the map-frame global costmap at
    the wrong coordinates. Two things happen at once: phantom obstacles appear in open
    floor, and real walls are raytraced away as free space. Nav2 does not undo either. An
    obstacle cell clears only when the robot later raytraces through it with correct
    localization, so corruption outlives the goal that produced it.

    The severe case is a phantom in space the robot can never reach or see through —
    typically beyond a wall. Those cells can never be raytraced clear, so they persist for
    the lifetime of the costmap node and their inflation keeps bleeding into traversable
    space. Over a 72-cell campaign sharing one bringup this accumulates monotonically: the
    last cell would run against a map carrying every earlier cell's damage, a systematic
    bias correlated with nothing but run order.

    Reported as counts so the effect is a number rather than an impression, and so the
    costmap-clearing fix can be shown to work instead of assumed to.

Categories
    phantom_clearable    costmap says blocked, static map says free. Blocks planning, but
                         the robot could in principle raytrace it away.
    phantom_permanent    costmap says blocked, static map says unknown — beyond the mapped
                         area. Unreachable, therefore never clears. The damaging kind.
    erased_wall          costmap says free, static map says occupied. A real wall the robot
                         has deleted; the planner will happily route through it.

Outputs
    <prefix>_costmap.json — per-sample series plus peak and final counts.

Usage
    costmap_corruption_monitor.py --output-prefix DIR/run_1 [--interval 2.0]
"""

import argparse
import json
import os
import signal
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# nav2_costmap_2d publishes costs translated into OccupancyGrid values:
# 0 free, 99 inscribed, 100 lethal, -1 unknown, 1..98 scaled intermediate.
BLOCKED = 99      # inscribed or lethal
STATIC_FREE = 0
STATIC_OCCUPIED = 100
STATIC_UNKNOWN = -1


def latched_qos():
    """Map and costmap topics are latched (transient local), so match that or get nothing."""
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class CostmapCorruptionMonitor(Node):
    """Compares the live global costmap against the static map it was built from."""

    def __init__(self, output_prefix, map_topic, costmap_topic, interval):
        super().__init__("costmap_corruption_monitor")
        self.output_prefix = output_prefix
        self.interval = interval
        self.static = None
        self.costmap = None
        self.samples = []
        self.started = time.monotonic()

        self.create_subscription(OccupancyGrid, map_topic, self._on_map, latched_qos())
        self.create_subscription(
            OccupancyGrid, costmap_topic, self._on_costmap, latched_qos()
        )
        self.create_timer(interval, self._sample)
        self.get_logger().info(
            f"comparing {costmap_topic} against {map_topic} every {interval}s"
        )

    def _on_map(self, msg):
        self.static = msg

    def _on_costmap(self, msg):
        self.costmap = msg

    def _compare(self):
        """Count disagreeing cells by category, or None if either grid is missing."""
        if self.static is None or self.costmap is None:
            return None

        s, c = self.static, self.costmap
        # Only a like-for-like grid comparison is meaningful. Nav2 sizes the global costmap
        # from the map, so a mismatch means something is misconfigured and a cell-by-cell
        # diff would be nonsense rather than merely imprecise.
        if (s.info.width != c.info.width or s.info.height != c.info.height
                or abs(s.info.resolution - c.info.resolution) > 1e-9):
            return {"error": (
                f"grid mismatch: map {s.info.width}x{s.info.height}@{s.info.resolution} "
                f"vs costmap {c.info.width}x{c.info.height}@{c.info.resolution}")}

        phantom_clearable = phantom_permanent = erased_wall = 0
        for sv, cv in zip(s.data, c.data):
            if cv >= BLOCKED:
                if sv == STATIC_FREE:
                    phantom_clearable += 1
                elif sv == STATIC_UNKNOWN:
                    phantom_permanent += 1
            elif 0 <= cv < BLOCKED and sv == STATIC_OCCUPIED:
                erased_wall += 1

        area = s.info.resolution ** 2
        return {
            "phantom_clearable": phantom_clearable,
            "phantom_permanent": phantom_permanent,
            "erased_wall": erased_wall,
            "phantom_total": phantom_clearable + phantom_permanent,
            "phantom_area_m2": round((phantom_clearable + phantom_permanent) * area, 4),
            "erased_area_m2": round(erased_wall * area, 4),
        }

    def _sample(self):
        result = self._compare()
        if result is None:
            return
        result["t"] = round(time.monotonic() - self.started, 2)
        self.samples.append(result)

    def write(self):
        """Write the series with peak and final counts, and an explicit verdict."""
        path = f"{self.output_prefix}_costmap.json"
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)

        valid = [s for s in self.samples if "error" not in s]
        errors = [s for s in self.samples if "error" in s]

        def peak(key):
            return max((s[key] for s in valid), default=None)

        summary = {
            "schema": "navlearn.costmap_corruption/1",
            "samples": len(self.samples),
            "sample_interval_s": self.interval,
            "series": valid[-200:],
            "peak": {
                "phantom_clearable": peak("phantom_clearable"),
                "phantom_permanent": peak("phantom_permanent"),
                "erased_wall": peak("erased_wall"),
                "phantom_area_m2": peak("phantom_area_m2"),
            },
            "final": valid[-1] if valid else None,
            "errors": errors[:3],
            "notes": {
                "phantom_permanent": (
                    "Blocked cells where the static map is unknown — beyond the mapped "
                    "area. The robot can never raytrace through them, so they never clear "
                    "and their inflation keeps intruding on traversable space."
                ),
                "erased_wall": (
                    "Real walls the robot has deleted from its costmap. The planner will "
                    "route through them."
                ),
            },
        }
        if valid:
            f = valid[-1]
            summary["verdict"] = (
                "CLEAN — costmap matches the static map"
                if f["phantom_total"] == 0 and f["erased_wall"] == 0 else
                f"CORRUPTED — {f['phantom_total']} phantom cells "
                f"({f['phantom_permanent']} permanent), {f['erased_wall']} erased wall cells"
            )
        else:
            summary["verdict"] = "no comparable samples (map or costmap never received)"

        with open(path, "w") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
            handle.write("\n")
        self.get_logger().info(f"costmap -> {path} | {summary['verdict']}")
        return path


def main():
    """Sample until signalled, then write the record."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--output-prefix", required=True)
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--costmap-topic", default="/global_costmap/costmap")
    parser.add_argument("--interval", type=float, default=2.0)
    args, ros_args = parser.parse_known_args()

    sys.stdout.reconfigure(line_buffering=True)
    rclpy.init(args=ros_args)
    node = CostmapCorruptionMonitor(
        args.output_prefix, args.map_topic, args.costmap_topic, args.interval
    )

    running = {"value": True}

    def stop(_s, _f):
        running["value"] = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        while running["value"] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        try:
            node.write()
        except OSError as exc:
            node.get_logger().error(f"could not write costmap record: {exc}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
