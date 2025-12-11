# NavLearn: 

_Reproducible Nav2-based navigation and benchmarking stack for real and simulated robots_

![CI](https://github.com/MihirMK17/navlearn/actions/workflows/ci.yml/badge.svg)

NavLearn is a modular ROS 2 project for building and **benchmarking** autonomous mobile robot navigation pipelines across Gazebo and NVIDIA Isaac Sim, and mirroring them on real robots. It bundles SLAM, localization, planning, control, and a reproducible Nav2 benchmarking harness with planned extensions for reinforcement learning and multi-robot systems.

---

## 📸 Project Highlights
<p align="center">
  <img src="media/hero.JPG" width="600"/>
</p>

---

## Overview / Motivation

Nav2 is a powerful, production-grade navigation framework, but turning it into a **reliable, well-tuned stack for your own robot** is still painful. You juggle hundreds of parameters, multiple planners/controllers, different simulators, and a real robot — and there’s no single, repeatable way to say “this configuration is better than that one.”

NavLearn is meant to be that missing piece: a single ROS 2 repo that couples a solid Nav2 stack with a **reproducible benchmarking workflow** for both simulation and real robots.

It provides:

- A **reference Nav2-based navigation stack** (SLAM/localization, planning, control) wired for a Turtlebot-style differential-drive robot.
- A **sim-first workflow** that keeps Gazebo and NVIDIA Isaac Sim setups aligned so you can iterate quickly before touching hardware.
- A **benchmarking harness** that drives scripted navigation goals, records per-goal metrics, and exports CSV/JSON summaries for comparing parameter sets, maps, and robots.
- A foundation for **future work**: reinforcement-learning policies on top of Nav2, multi-robot navigation experiments, and safety layers.

The goal is not “yet another navigation demo,” but a **repeatable way to answer** questions like:

- Did this controller or planner change actually improve navigation performance?
- How does a given Nav2 configuration behave on a simple 1 m square test map versus a cluttered apartment layout?
- How close is my real-robot behavior to what I see in Isaac Sim?

If you care about moving from “it kind of navigates” to “I can quantify and iterate on navigation performance,” NavLearn is the stack and tooling this repo provides.

---

## Key Features

- **End-to-end Nav2 stack, wired and tuned**
  - SLAM / mapping, localization, planning, and control set up for a Turtlebot-style differential-drive robot.
  - Designed so you can swap maps and robots without rewriting the whole launch zoo.

- **Simulation-first workflow (Gazebo / NVIDIA Isaac Sim)**
  - Matching Nav2 configuration across Gazebo and Isaac Sim so you can debug behavior in sim before touching hardware.
  - Same navigation pipeline, different backends: helps you catch modeling and tuning issues early.

- **Real-robot mirroring**
  - Launch files and configs intended to mirror the simulation stack on a real robot.
  - Lets you take a Nav2 config from “works in sim” to “actually runs on wheels” instead of stopping at RViz demos.

- **Reproducible benchmarking harness**
  - Scripted navigation goals (random or predefined) driven through NavigateToPose.
  - Per-goal metrics (success/failure, timing, path-level metrics, control metrics) logged to CSV and JSON.
  - Multi-run harness + aggregator to compare parameter sets, maps, or robots across batches of runs.

- **Config-first, ROS 2 native design**
  - All major behaviors controlled through ROS 2 parameters and YAML (no magic constants buried in code).
  - Clean separation between bringup, navigation, and benchmarking packages.

- **Built to be extended**
  - Room to plug in reinforcement-learning policies on top of Nav2, experiment with multi-robot scenarios, and add new metrics without rewriting the core stack.
  - CI on GitHub Actions to keep the project building and testable as it grows.
  
---

## Quickstart (TL;DR)

### 0. Prerequisites

- Ubuntu 22.04 with **ROS 2 Humble**
- **Nav2** installed (e.g. `sudo apt install ros-humble-nav2-bringup`)
- **Gazebo** (Fortress/Garden) and the **Bumperbot** stack available in your workspace  
  (from the Learn-By-Doing courses by Antonio Brandi / `bumperbot_bringup` package)
- A working ROS 2 workspace (colcon)  

> The examples below assume a Bumperbot-style differential-drive robot. Adapting to other robots is covered later.

---

### 1. Clone and build

```bash
mkdir -p ~/navlearn_ws/src
cd ~/navlearn_ws/src

# Clone NavLearn
git clone https://github.com/MihirMK17/navlearn.git

cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 3. Bring up simulation + Nav2 (Gazebo + Bumperbot)
Terminal 1 - Gazebo + Robot Bring up
```
cd ~/navlearn_ws
source install/setup.bash

ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house
```
This launches the Bumperbot robot in a small house Gazebo world and starts the Nav2-based navigation stack used by NavLearn (planner, controller, costmaps, BT, etc). 

You should be able to send Nav2 goals from RViz and see the robot navigate.

### 4. Run a single NavLearn benchmark run
With the sim and Nav2 stack running
```
cd ~/navlearn_ws
source install/setup.bash

ros2 launch navlearn_benchmarks benchmarks.launch.py goal_nums:=4 goal_source:=map_random
```
This will:
- Sample 4 random navigation goals on the current map
- Drive them through `NavigateToPose`
- Record per-goal metrics and a per-run JSON summary under `benchmark_reports/` (default path; details in the *Usage* section)


The full pipeline should now be running:
Gazebo world --> Bumperbot --> Nav2 Stack --> NavLearn benchmarking harness --> CSV / JSON metrics

---

## Usage

Assuming the following is already done
- Built the workspace and sourced it (`source install/setup.bash`)
- Got a robot + Nav2 stack running (Bumperbot, Gazebo / Isaac Sim, or real robot)

### 1. Navigation / Bringup Workflows

#### 1.1 Gazebo + Bumperbot (recommended starting point)
1. Start Gazebo + Bumperbot bringup
```
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house
```

2. Verify in RViz that 
- TF tree is stable (`map --> odom --> base_link --> laser ...`)
- You can send a normal Nav2 goal and the robot movees

Once that's true, the NavLearn's benchmarking harness is ready to be plugged on top.

### 2. Benchmarking Workflows
This benchmarking stack is in:
- `navlearn_msgs`: interface types (ControlMetric, TrajectoryMetric, ControlMetric, EpisodeEvent)
- `navlearn_benchmarks`: C++ nodes and Python tooling
	- `episode_manager`
	- `trajectory_metric`
	- `control_metric`
	- `metrics_compiler`
	- `srcipts/multi_run_harness.py`
	- `srcipts/aggregate_runs.py`

#### 2.1 Build the benchmarking stack (optional)
Build the whole workspace or only the navlearn_benchmarks package for just benchmarking pieces
```
cd ~/navlearn_ws
colcon build --packages-select navlearn_msgs navlearn_benchmarks
source install/setup.bash
```

#### 2.2 Single-run benchmark
With Bumperbot + Nav2 already running (sim or real)

```
cd ~/navlearn_ws
source install/setup.bash

ros2 launch navlearn_benchmarks benchmarks.launch.py
```

By default this will:
- Use the configs in `src/navlearn_benchmarks/config/`:
	- `episode_manager_1mSquare.yaml`
	- `trajectory_metric.yaml`
	- `control_metric.yaml`
	- `metrics_compiler.yaml`
- Run the cannonical 1m Square benchmark (if `episode_manager_1mSquare.yaml` is selected)
- Log:
	- Per-goal metrics --> `benchmark_reports/navlearn_metrics.csv`
	- Per-run summary --> `benchmark_reports/navlearn_run_report.json`

This can be treated as the "sanity-check" benchmark; if this doesn't work, the Nav2 / TF / Robot setup might be broken.

#### 2.3 Choosing goal sources (static vs random)
The `episode_manager` node controls which navigation goals are sent. It has a key parameter:
- `goal_source` – one of:
	- `static`: uses a fixed set of goals from a YAML file (canonical 1 m square, hand-picked poses, etc.)
	- `map_random`: sample random free-cell goals from the current /map

High-level behavior:
```
# Example (inside episode_manager_*.yaml)

episode_manager:
  ros__parameters:
    goal_source: static         # or "map_random"
    goals_num: 4                # used when goal_source == "map_random"
    # For static:
    goal_x: [0.5, 0.5, -0.5, -0.5]
    goal_y: [0.5, -0.5, -0.5, 0.5]
    goal_yaw: [0.0, 1.57, 3.14, -1.57]
```
- `goal_source: static`
	- Uses YAML-defined arrays of `goal_x', `goal_y', `goal_yaw'
	- Arrays must be the same length (one entry per goal)
- `goal_source: map_random`
	- Ignores explicit coordinates
	- Samples `goals_num` random free cells from `/map` and uses those as goal positions\

To switch between them, launch the `benchmark.launch.py` with the following launch arguments:
- `episode_manager_config:=episode_manager_1mSquare.yaml` + `goal_source:=static` --> For pre-defined goals. Change the `episode_manager_config` argument with the YAML file with the required changes
-  `goal_source:=map_random` + `goals_num:=4` --> For random goals

#### 2.4 Multi-run harness (batch experiments)
For running many benchmarks back-to-back (e.g., different parameter sets, goal counts, or maps), use:
```
cd ~/navlearn_ws
source install/setup.bash

python3 src/navlearn_benchmarks/scripts/multi_run_harness.py
```

This harness script is designed to:
- Define a list of runs (different configs, maps, or goal counts) in the script itself
- For each run:
	- Call the `navlearn_benchmarks` launch file with the appropriate parameters
	- Save that run's output into a separate directory under:
	```
	src/navlearn_benchmarks/benchmark_reports/runs/
	run_0001/
	navlearn_metrics.csv
	navlearn_run_report.json
	run_0002/
	```
To customize the experiments to run:
- Open `scripts/multi_run_harness.py`
- Edit the run definitions at the top (maps, `goal_source`, `goals_nums`, `episode_nums`, etc)
- Re-run the script

This is the tool to use for actual sweeps, not one-off runs.

#### 2.5 Aggregating runs (compare experiments)
After running multiple benchmarks, the runs are saved under `benchmark_reports/runs`, they can be aggregate. To do so follow below steps
```
cd ~/navlearn_ws
source install/setup.bash

python3 src/navlearn_benchmarks/scripts/aggregate_runs.py
```

Typical behavior:
- Scans `benchmark_reports/runs/**/navlearn_run_report.json`
- Builds a table with one row per run, including metrics like:
	- number of goals
	success rate
	mean/median navigation time
	mean path length
- Prints that table to the terminal in a readable format.
- Optionally writes a summary CSV (e.g. `benchmark_reports/navlearn_runs_summary.csv`) you can open in a notebook or spreadsheet.

This script can be used to compare the configuration, map, robot, etc across multiple runs

---

## 📊 Benchmarking Framework (navlearn_benchmarks + navlearn_msgs)


This repo also includes a **navigation benchmarking framework** for Nav2 

The goal is to:

- Run multiple navigation **episodes** in a controlled environment.
- Log **per-episode metrics** (success, time-to-goal, collisions, path length, etc.).
- Compare different planners/controllers/maps on the **same benchmark**.

### Packages

- `navlearn_msgs`: custom message definitions for benchmarking:
  - `GoalMetric.msg`
  - `TrajectoryMetric.msg`
  - `ControlMetric.msg`
  - `EpisodeEvent.msg`

- `navlearn_benchmarks`: C++ nodes that compute and aggregate metrics:
  - `episode_manager` – orchestrates episodes, sends goals, tracks start/end.
  - `trajectory_metric` – path length and basic trajectory stats.
  - `control_metric` – logs `/cmd_vel` for control statistics.
  - `metrics_compiler` – writes CSV / JSON reports.
  
### Episode Manager
Role: 
- Loads navigation goals and sends them to the Robot sequentially
- Tracks episode state, navigation time, start pose, goal pose (and collisions --> planned) for each goal
- Publishes them as EpisodeEvent messages
- Accuracy in goal_xy and goal_yaw is decided by controller_server (0.25m and 0.25 rad/s)
- Accuracy in path length measurements is decided by trajectory metric node's parameters

Configurations: EpisodeManger has the ability to use two goal sources - fixed or randomized as selected by changing the goal_source pararmeter
```bash
goal_source: static
```
Uses YAML defined goals (cannonical 1m demo). The X, Y and Yaw coordinates of the goals configured need to match in size

```bash
- goal_source: map_random
```
Samples `goals_num` size of random free cells from `/map`. Only needs `goals_num` parameter defined


## 🔧 Building the Benchmarking Components

If you follow the standard workspace build (see Quick Start), benchmarking packages are built automatically.

To explicitly build just the benchmarking stack:

```bash
cd ~/navlearn_ws
colcon build --packages-select navlearn_msgs navlearn_benchmarks
source install/setup.bash

```

### Launch the Benchmarking Stack
```bash
cd ~/navlearn_ws
source install/setup.bash

ros2 launch navlearn_benchmarks benchmarks.launch.py
```

By default, the launch file uses the configs in:

```bash
src/navlearn_benchmarks/config/
  episode_manager_1mSquare.yaml
  metrics_compiler.yaml
  control_metric.yaml
  trajectory_metric.yaml
```

### 📂 Benchmark Outputs

After the benchmark run finishes, reports are generated in:

```bash
src/navlearn_benchmarks/benchmark_reports/
  navlearn_metrics.csv
  navlearn_run_report.json
```

### 📐 Metrics Definitions (High-Level)

Goal Success
A goal is considered successful if the robot reaches inside Nav2’s goal tolerances:

Position within xy_tolerance

Orientation within yaw_tolerance

Time-to-Goal

```bash
t_goal = t_goal_reached − t_goal_sent
```

Measured per episode (per goal sequence if multiple goals are chained).

- Path Length: Sum of Euclidean distances between consecutive robot poses along the executed trajectory, as estimated by odometry/localization.

- Collisions: Number of collision events detected in an episode.

- Control Metrics: Statistics of the control commands (`/cmd_vel`):
	Mean / max linear velocity (`|v|`)
	Mean / max angular velocity (`|ω|`)

Potentially extendable to tracking error if reference vs. executed trajectories are logged.

The framework is designed to be extendable—add more metrics as required.

---

## 🎥 Demos

### 📡 SLAM Mapping Demo

> Demonstrates full mapping in an indoor environment.

<p align="center">
  <img src="media/slam_mapping.gif" width="75%" alt="Slam Mapping"/>
</p>

---

### 🚧 Navigation with Static and Dynamic Obstacles

> Shows side-by-side RViz and Gazebo with real-time path replanning.

<p align="center">
  <img src="media/nav_demo.gif" width="75%" alt="Nav demo"/>
</p>

---

### 🧭 Navigation Snapshot

> Local and global costmaps with AMCL localization
<p align="center">
  <img src="media/nav.png" width="600"/>
</p>

---

### 🧪 Gazebo Simulation

> Simulated robot running the full navigation stack

<p align="center">
  <img src="media/gazebo_demo.gif" width="75%" alt="Gazebo demo"/>
</p>

---

### 🗂 Canonical Benchmark Demo — 1m Square

> The canonical NavLearn benchmark is a robot running repeated 1m square navigation episodes in Isaac Sim while metrics are logged

High-level flow:
- Isaac Sim runs the world and robot with ROS 2 bridge.
- Nav2 runs the navigation stack on the ROS side.
- navlearn_benchmarks nodes run in parallel to log metrics and compile reports.

<p align="centre">
  <img src="media/navlearn_benchmark_demo.gif" width="75%" at="Navlean Benchmarking Demo"/>
</p> 

---

### 🧱 TF Tree

> Frame visualization after `bumperbot_bringup`

<img src="media/tf_tree.png" width="500"/>

---

## 🚀  Quick Start

### Clone & Build

```bash
# 1) create a workspace & clone the repo
mkdir -p ~/navlearn_ws/src && cd ~/navlearn_ws/src
git clone https://github.com/MihirMK17/navlearn.git
cd ~/navlearn_ws

# 2) resolve deps & build
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

### Simulation (Gazebo)
```bash
ros2 launch navlearn_bringup simulated_robot.launch.py world_name:=small_house use_slam:false
```

### Real Robot
```bash
ros2 launch navlearn_bringup real_robot.launch.py world_name:=small_house use_slam:false
```

## 📚 Project Documentation

### ✅ Completed

* [x] Real-world SLAM & Navigation setup
* [x] Simulation setup & ROS2 bridge
* [x] Teleop + autonomous navigation
* [x] CI workflow for ROS2 Humble

### 🐞 Common Issues & Fixes

**1. L298N H-Bridge Partial Failure**
Only one motor would work in forward motion due to a damaged internal transistor. Diagnosis was done using joystick teleop commands.

**Fix:** Switched to a reliable driver (e.g., TB6612FNG), added flyback protection, improved ventilation, and verified stall current specs.

**2. HRB 3S Battery Power Drop**
Old battery dropped to 5.04V leading to inconsistent motor behavior and RPi brownout.

**Fix:** Replaced with a rechargeable 3S G-Tech 5000mAh LiPo, ensuring consistent voltage and capacity.

**3. TF Tree Frame Drops / Misalignment**
During SLAM and Navigation, unexpected frame loss occurred.

**Fix:** Verified static\_transform\_publisher configurations, frame\_id consistency in URDF, and launched the correct state\_publisher order.

More fixes and logs documented in [`Project Documentation`](Project%20Documentation.docx)

---

## 🚀 Coming Soon

* ✅ Calibration of wheel base and motor gains
* 🧠 Reinforcement Learning for local planning
* 🤖 Multi-agent navigation support
* ☁️ Edge-cloud updates for policy deployment
* 📈 More advanced benchmarking metrics and automated test scenarios

---

## 👤 Author

* Mihir Kulkarni
* [LinkedIn](https://www.linkedin.com/in/kulkarnimihir17/)
* [mihir.kulkarni17@gmail.com](mailto:mihir.kulkarni17@gmail.com)

---

## 📜 License

This project is based on open-source work by Antonio Brandi ([BumperBot](https://github.com/AntoBrandi/Bumper-Bot)) under Apache 2.0 License.
All modifications and extensions in NavLearn are released under the same license.

---

> 🚧 Built for learning. Made for real-world autonomy.

---


