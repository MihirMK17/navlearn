# NavLearn: Autonomous Navigation Stack for Real & Simulated Robots

![CI](https://github.com/MihirMK17/navlearn/actions/workflows/ci.yml/badge.svg)

NavLearn is a modular ROS2-based project built for learning and experimenting with autonomous mobile robot navigation, both in Gazebo simulation and real hardware. This stack includes SLAM, localization, path planning, and navigation with future extensions for reinforcement learning and multi-robot systems.

---

## 📸 Project Highlights

### Hero Shot

<img src="media/hero.JPG" width="600"/>

---

## 🎯 Features

* 🗺️ **SLAM with Lidar**: Real-time mapping of unknown environments
* 📍 **Localization**: AMCL-based probabilistic localization
* 🧭 **Navigation**: Nav2 stack integration with obstacle avoidance
* 🧪 **Gazebo Simulation**: Full support for testing in simulation
* 🤖 **Real Robot Ready**: Deployed on differential drive robot with calibrated control
* 🔜 **Upcoming**: RL-based planning, multi-agent exploration

---

## 🧠 Architecture Overview

```mermaid
graph TD
  subgraph Sensors & Inputs
    IMU
    Lidar
    WheelEncoders
    Joypad
  end

  subgraph Perception
    SLAM
    Localization
  end

  subgraph Planning
    GlobalPlanner
    LocalPlanner
    Costmaps
  end

  subgraph Control
    PIDController
    MotorDriver
  end

  subgraph Interfaces
    ROS2Nodes
    RViz
    Gazebo
  end

  IMU --> Localization
  Lidar --> SLAM
  WheelEncoders --> Localization
  Joypad --> ROS2Nodes

  SLAM --> Costmaps
  Localization --> Costmaps
  Costmaps --> GlobalPlanner
  GlobalPlanner --> LocalPlanner
  LocalPlanner --> PIDController
  PIDController --> MotorDriver

  ROS2Nodes --> RViz
  ROS2Nodes --> Gazebo
```

---

## 🎥 Demos

### 📡 SLAM Mapping Demo

> Demonstrates full mapping in an indoor environment.

[https://user-images.githubusercontent.com/123456789/your-slug/slam\_mapping.mp4](https://user-images.githubusercontent.com/123456789/your-slug/slam_mapping.mp4)

---

### 🚧 Navigation with Static and Dynamic Obstacles

> Shows side-by-side RViz and Gazebo with real-time path replanning.

[https://user-images.githubusercontent.com/123456789/your-slug/nav\_demo.mp4](https://user-images.githubusercontent.com/123456789/your-slug/nav_demo.mp4)

---

### 🧭 Navigation Snapshot

> Local and global costmaps with AMCL localization

<img src="media/nav.png" width="600"/>

---

### 🧪 Gazebo Simulation

> Simulated robot running the full navigation stack

[https://user-images.githubusercontent.com/123456789/your-slug/gazebo\_demo.mp4](https://user-images.githubusercontent.com/123456789/your-slug/gazebo_demo.mp4)

---

### 🧱 TF Tree

> Frame visualization after `bumperbot_bringup`

<img src="media/tf_tree.png" width="500"/>

---

## 📁 Directory Structure (robot\_ws)

```bash
robot_ws/
├── src/
│   ├── bumperbot_controller/
│   ├── bumperbot_description/
│   ├── bumperbot_firmware/
│   ├── bumperbot_localization/
│   ├── bumperbot_navigation/
│   └── bumperbot_bringup/
├── media/
│   └── *.mp4, *.png, *.JPG
├── .github/workflows/
│   └── ci.yml
└── README.md
```

---

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

---

## 👤 Author

* Mihir Kulkarni
* [LinkedIn](https://www.linkedin.com/in/mihirmk/)
* [mihir.kulkarni17@gmail.com](mailto:mihir.kulkarni17@gmail.com)

---

## 📜 License

This project is based on open-source work by Antonio Brandi ([BumperBot](https://github.com/AntoBrandi/Bumper-Bot)) under Apache 2.0 License.
All modifications and extensions in NavLearn are released under the same license.

---

> 🚧 Built for learning. Made for real-world autonomy.

---

