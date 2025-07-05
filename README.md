![CI](https://github.com/MihirMK17/navlearn/actions/workflows/ci.yml/badge.svg)

# navlearn

**navlearn** is my ROS 2 Humble mobile-robot stack that started from Antonio Brandi’s open-source *Bumper-Bot* template and is now evolving toward RL-enhanced, multi-robot autonomy.  
Everything here runs both in real hardware (Raspberry Pi 4 + Arduino motor driver) and in simulation (Gazebo / Isaac Sim).

<p align="center">
  <img src="media/hero.JPG" width="60%" alt="navlearn robot photo"/>
</p>

---

## Key features

| ✓ | Capability |
|---|-------------|
| **URDF + simulation** – full robot model with lidar, IMU, wheels |
| **Localization** – EKF (wheel odom + IMU) and AMCL on a 2-D lidar map |
| **Mapping & Nav2** – SLAM Toolbox / Nav2 tuned for small-room navigation |
| **Real-robot calibration** – wheel-radius & separation tuning, PID gains |
| **CI pipeline** – GitHub Actions builds & tests every push |
| **Ready for RL / Fleet** – placeholders for a RL local planner and multi-robot coordination packages |

---

## Quick start (simulation)

```bash
git clone https://github.com/MihirMK17/navlearn.git
cd navlearn && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Gazebo demo
ros2 launch robot_bringup simulation.launch.py

