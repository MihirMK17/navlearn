#!usr/bin/env python
import json 
import pathlib
import sys

def load_run_summaries(directory: pathlib.Path):
    summaries = []
    for path in sorted(directory.glob("navlearn_run_report_run_*.json")):
        with open(path, "r") as f:
            data = json.load(f)
        summaries.append((path.name, data))
    return summaries

def print_summary_table(summaries):
    if not summaries:
        print("No run summaries found.")
        return

    header = [
        "Run",
        "Goals",
        "Success",
        "Fail",
        "NavTime_mean[s]",
        "Path_mean[m]",
        "CtrlEnergy_mean",
    ]
    print("\t".join(header))

    total_goals = 0
    total_succ = 0
    total_fail = 0
    sum_nav_time = 0.0
    sum_path = 0.0
    sum_ctrl_energy = 0.0

    for filename, d in summaries:
        goals = int(d["goals_total"])
        succ = int(d["goals_succeeded"])
        fail = int(d["goals_failed"])
        nav_mean = float(d["nav_time_mean"])
        path_mean = float(d["path_length_mean"])
        ctrl_mean = float(d["control_energy_mean"])

        total_goals += goals
        total_succ += succ
        total_fail += fail
        sum_nav_time += nav_mean * goals
        sum_path += path_mean * goals
        sum_ctrl_energy += ctrl_mean * goals

        print("\t".join([
            filename,
            str(goals),
            str(succ),
            str(fail),
            f"{nav_mean:.2f}",
            f"{path_mean:.2f}",
            f"{ctrl_mean:.1f}",
        ]))

    # Global averages across *all goals in all runs*
    if total_goals > 0:
        nav_global = sum_nav_time / total_goals
        path_global = sum_path / total_goals
        ctrl_global = sum_ctrl_energy / total_goals
    else:
        nav_global = path_global = ctrl_global = 0.0

    print("")  # blank line
    print("\t".join([
        "ALL_RUNS",
        str(total_goals),
        str(total_succ),
        str(total_fail),
        f"{nav_global:.2f}",
        f"{path_global:.2f}",
        f"{ctrl_global:.1f}",
    ]))

def main():
    if len(sys.argv) > 1:
        directory = pathlib.Path(sys.argv[1])
    else:
        directory = (
            pathlib.Path.home()
            / "robot_ws/src/navlearn_benchmarks/benchmark_reports/runs/baseline"
        )

    if not directory.exists():
        print(f"Directory {directory} does not exist.")
        sys.exit(1)

    summaries = load_run_summaries(directory)
    print_summary_table(summaries)

if __name__ == "__main__":
    main()