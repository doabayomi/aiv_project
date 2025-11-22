# Autonomous Indoor Vehicle (AIV) Navigation Stack
### B.Eng Final Year Project | Bells University of Technology

This repository contains the source code and ROS 2 configuration for the **Design and Implementation of an Autonomous Indoor Vehicle**, developed as a final year undergraduate thesis.

The project implements **LiDAR-based Simultaneous Localization and Mapping (SLAM)** and **Frontier Exploration** on a differential-drive mobile robot. It features a custom navigation stack built on **ROS 2 Humble**, capable of mapping indoor environments.

---

## 🤖 System Overview

The system is built upon the **Waveshare WAVE ROVER** chassis, utilizing a **Raspberry Pi 4B** as the central processing unit. It bridges high-level path planning with low-level motor control via a custom serial interface to an ESP32 driver board.

### Key Features
* **LiDAR-Based SLAM:** Real-time 2D mapping and localization using the `slam_toolbox` and **LD19 ToF LiDAR**.
* **Frontier Exploration:** Autonomous navigation of unknown map frontiers to complete mapping of an environment.
* **Custom Hardware Interface:** A Python-based ROS 2 hardware abstraction layer converting `geometry_msgs/Twist` commands into serial packets for the Waveshare chassis.
* **Sensor Integration:** Fusion of LiDAR and IMU data for robust odometry.

---

## 🛠️ Hardware Architecture

* **Chassis:** Waveshare WAVE ROVER (4WD Metal Body, Differential Drive).
* **Onboard Computer:** Raspberry Pi 4B (4GB RAM) running Ubuntu 22.04 Server.
* **LiDAR:** LD19 D-ToF (12m range, 4500Hz sampling).
* **Vision:** A200 RGB-D Camera (used for depth estimation benchmarks).
* **Low-Level Control:** ESP32-WROOM-32 Driver Board (handles Motor PID and 9-axis IMU data).

---

## 📊 Research Findings \& Benchmarks

As part of the thesis, this platform was used to **benchmark and analyze** SLAM performance across varying indoor conditions using a novel **"Clutter Index"** to quantify environmental complexity. The primary contribution was the *documentation* of performance degradation rather than a coded solution for it.

* **Trajectory Accuracy (RMSE):** 8.24 cm.
* **Localization Accuracy:** $\sim$10 cm relative to ground truth.
* **Average Pose Error:** 4.78 degrees (Orientation).
* **Key Finding:** Identified a linear correlation between environmental clutter levels and LiDAR point cloud noise standard deviation.

---

## 📦 Dependencies \& Setup

### Prerequisites
* **OS:** Ubuntu 22.04 LTS.
* **Middleware:** ROS 2 Humble Hawksbill.

### Installation
```bash
# Clone the repository
git clone [https://github.com/doabayomi/aiv_project.git](https://github.com/doabayomi/aiv_project.git)

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
````

## 🚀 Usage

This stack supports three primary operational modes via the `wave_rover` package:

**1. System Initialization (Driver & Sensor Launch)**
This launches the LD19 LiDAR driver, IMU node, and the custom serial driver node for the Waveshare chassis.

```bash
ros2 launch wave_rover wave_rover
```

**2. Manual Control (Teleoperation)**
After initialization, use a keyboard or game controller to manually teleoperate the vehicle for mapping or debugging.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard # (Standard ROS2 teleop node)
```

**3. Autonomous Frontier Exploration**
Starts the SLAM process and launches the autonomous exploration behavior, directing the robot toward unknown frontiers.

```bash
ros2 launch wave_rover frontier_exploration# (Assuming this is the launch file name)
```

-----

## 👤 Authors

**Daniel Abayomi & Richard Okpara**
*B.Eng Mechatronics Engineering*
