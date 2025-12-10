#!/usr/bin/env python
import subprocess
import pathlib
import time

EPISODES_NUM = 2
GOALS_NUM = 2
GOAL_SOURCE = "map_random"
REPORT_DIR = pathlib.Path.home() / "robot_ws/src/navlearn_benchmarks/benchmark_reports/runs"
REPORT_DIR.mkdir(parents=True, exist_ok=True)

def run_benchmark(episode_id_: int):
    episode_id_ = episode_id_ + 1
    stamp = time.strftime("%Y%m%d_%H%M%S")

    csv_path = REPORT_DIR / f"navlearn_metrics_run_{episode_id_}_{stamp}.csv"
    json_path = REPORT_DIR / f"navlearn_run_report_run_{episode_id_}_{stamp}.json"

    print(f"\n=== RUN {episode_id_} / {EPISODES_NUM} ====")
    print(f"CSV --> {csv_path}")
    print(f"JSON --> {json_path}")

    cmd = [
        "ros2", "launch", "navlearn_benchmarks", "benchmarks.launch.py",
        f"goals_num:={GOALS_NUM}",
        f"goal_source:={GOAL_SOURCE}",
        f"csv_path:={csv_path}",
        f"json_path:={json_path}"
    ]

    result = subprocess.run(cmd)

    if result.returncode != 0:
        print(f"[WARN] Run {episode_id_} exited with code {result.returncode}")

    time.sleep(5.0)

def main():
    for i in range(EPISODES_NUM):
        run_benchmark(i)


if __name__ == "__main__":
    main()