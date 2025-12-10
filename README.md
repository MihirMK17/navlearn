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


