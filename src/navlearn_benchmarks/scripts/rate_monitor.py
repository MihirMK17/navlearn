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

"""Measures what the stack actually achieved, as opposed to what it was configured for.

Three quantities, each of which silently invalidates results when it is assumed rather
than measured.

Achieved sensor rate
    The sensor-starve leg is twelve cells that differ only in LiDAR rate: 10, 5, 2 and
    1 Hz. Every conclusion from those cells rests on the robot having actually received
    the rate that was requested. A configured rate is a request to the simulator, not a
    guarantee — under load, or with a slow physics step, the delivered rate can fall
    below it. Measuring inter-arrival times is the difference between "we tested the
    stack at 2 Hz" and "we asked for 2 Hz and assumed we got it".

Control loop rate
    controller_server is configured at 20 Hz. Whether it achieves that is the property
    that transfers to real hardware: a controller that cannot hold its loop rate on a
    laptop will not hold it on a Jetson. Measured from /cmd_vel inter-arrival times.

Real-time factor
    Sim time divided by wall time. When RTF falls below 1 the simulator is running slower
    than real time, and everything timed in sim seconds is distorted relative to the
    compute that produced it. A cell whose RTF differs materially from its neighbours is
    not comparable with them, so RTF is recorded per run rather than assumed to be 1.

Rates are computed from inter-arrival deltas, never from a message count divided by
duration. A count-based average hides a stall: a topic that delivers at 20 Hz for half
the run and stops for the other half averages 10 Hz and looks merely slow, while the
percentiles here show it plainly.

Writes <output_prefix>_rates.json on shutdown. Exits cleanly on SIGINT/SIGTERM so the
harness can stop it with the run.
"""

import argparse
import json
import os
import signal
import statistics
import sys

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


def sensor_qos(depth=10):
    """QoS matching a sensor publisher: best-effort, volatile, keep-last."""
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class RateMonitor(Node):
    """Records inter-arrival statistics for the topics a run's validity depends on."""

    def __init__(self, output_prefix, scan_topic, cmd_vel_topic, expected_scan_hz):
        super().__init__("rate_monitor")
        self.output_prefix = output_prefix
        self.expected_scan_hz = expected_scan_hz

        # Wall-clock arrival times. Deliberately monotonic wall time rather than sim time:
        # the point is to measure delivery as it happened, and using sim time would fold
        # the simulator's own pacing into the very number meant to expose it.
        self._arrivals = {"scan": [], "cmd_vel": []}

        self.create_subscription(
            LaserScan, scan_topic, lambda _m: self._record("scan"), sensor_qos()
        )
        self.create_subscription(
            TwistStamped, cmd_vel_topic, lambda _m: self._record("cmd_vel"), 10
        )

        # RTF anchors. ROS time follows /clock when use_sim_time is set.
        self._wall_start = self._monotonic()
        self._sim_start = None
        self.create_timer(0.5, self._sample_clock)

        self.get_logger().info(
            f"rate_monitor watching {scan_topic} (expect {expected_scan_hz} Hz) "
            f"and {cmd_vel_topic}"
        )

    @staticmethod
    def _monotonic():
        """Wall-clock seconds, unaffected by sim time."""
        import time

        return time.monotonic()

    def _record(self, key):
        self._arrivals[key].append(self._monotonic())

    def _sample_clock(self):
        """Latch the first valid sim time; ROS time reads zero until /clock arrives."""
        if self._sim_start is None:
            now = self.get_clock().now().nanoseconds
            if now > 0:
                self._sim_start = now
                self._wall_start = self._monotonic()

    def _rate_stats(self, key):
        """Summarise a topic's delivery from inter-arrival deltas."""
        arrivals = self._arrivals[key]
        if len(arrivals) < 3:
            return {
                "messages": len(arrivals),
                "note": "too few messages to characterise; topic may never have published",
            }

        deltas = [b - a for a, b in zip(arrivals, arrivals[1:]) if b > a]
        if not deltas:
            return {"messages": len(arrivals), "note": "no positive inter-arrival deltas"}

        rates = sorted(1.0 / d for d in deltas)

        def pct(fraction):
            return rates[min(int(round(fraction * (len(rates) - 1))), len(rates) - 1)]

        return {
            "messages": len(arrivals),
            "span_sec": round(arrivals[-1] - arrivals[0], 3),
            "rate_hz_mean": round(len(deltas) / sum(deltas), 3),
            "rate_hz_median": round(statistics.median(rates), 3),
            "rate_hz_p05": round(pct(0.05), 3),
            "rate_hz_p95": round(pct(0.95), 3),
            "rate_hz_min": round(rates[0], 3),
            "rate_hz_max": round(rates[-1], 3),
            "max_gap_sec": round(max(deltas), 4),
        }

    def summary(self):
        """Build the run's rate and RTF record, including an explicit verdict."""
        wall_elapsed = self._monotonic() - self._wall_start
        sim_elapsed = None
        rtf = None
        if self._sim_start is not None:
            sim_elapsed = (self.get_clock().now().nanoseconds - self._sim_start) / 1e9
            if wall_elapsed > 0:
                rtf = round(sim_elapsed / wall_elapsed, 4)

        scan = self._rate_stats("scan")
        cmd_vel = self._rate_stats("cmd_vel")

        # A delivered rate materially below the requested one means the cell did not test
        # the condition it is labelled with. Flagged explicitly so it cannot be missed.
        scan_ok = None
        if self.expected_scan_hz > 0 and "rate_hz_median" in scan:
            scan_ok = scan["rate_hz_median"] >= 0.9 * self.expected_scan_hz

        return {
            "schema": "navlearn.rates/1",
            "wall_elapsed_sec": round(wall_elapsed, 3),
            "sim_elapsed_sec": None if sim_elapsed is None else round(sim_elapsed, 3),
            "real_time_factor": rtf,
            "rtf_note": (
                "sim seconds per wall second. Below 1 means the simulator ran slower than "
                "real time; cells with materially different RTF are not directly "
                "comparable on any time-based metric."
            ),
            "scan": scan,
            "cmd_vel": cmd_vel,
            "expected_scan_hz": self.expected_scan_hz,
            "scan_rate_met": scan_ok,
            "verdict": (
                "scan rate BELOW request — this cell did not test the rate it is labelled "
                "with" if scan_ok is False else "ok"
            ),
        }

    def write(self):
        """Write the rate record beside the run's other artifacts."""
        path = f"{self.output_prefix}_rates.json"
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        summary = self.summary()
        with open(path, "w") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
            handle.write("\n")

        rtf = summary["real_time_factor"]
        self.get_logger().info(
            f"rates -> {path} | RTF={rtf} "
            f"scan={summary['scan'].get('rate_hz_median')} Hz "
            f"cmd_vel={summary['cmd_vel'].get('rate_hz_median')} Hz"
        )
        if summary["scan_rate_met"] is False:
            self.get_logger().error(summary["verdict"])
        return path


def main():
    """Run the monitor until signalled, then write its record."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--output-prefix", required=True)
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--cmd-vel-topic", default="/bumperbot_controller/cmd_vel")
    parser.add_argument("--expected-scan-hz", type=float, default=10.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = RateMonitor(
        args.output_prefix, args.scan_topic, args.cmd_vel_topic, args.expected_scan_hz
    )

    running = {"value": True}

    def stop(_signum, _frame):
        running["value"] = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        while running["value"] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        # Written on every exit path. A run that ended badly is exactly the one whose
        # delivered rates are worth inspecting.
        try:
            node.write()
        except OSError as exc:
            node.get_logger().error(f"could not write rate record: {exc}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
