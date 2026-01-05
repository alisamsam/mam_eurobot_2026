# Eurobot 2026 Simulation Project: "Let's Keep the Hazelnuts Warm!"

This repository contains the autonomous software stack for the Eurobot 2026 competition by RoboUN team. [cite_start]The system is developed using **ROS 2 Humble** and simulated in **Gazebo**[cite: 17, 18].

## 🎯 Project Objective
[cite_start]The primary goal is to create an autonomous vehicle capable of collecting hazelnut crates from the playing arena and transporting them to specific, pre-defined locations (pantries or the squirrel nest)[cite: 231].

**Key Capabilities:**
* [cite_start]**Perception:** Real-time detection of blue/yellow crates and green pantries using computer vision[cite: 4, 10, 11].
* [cite_start]**Navigation:** SLAM, AMCL localization, and collision-free path planning[cite: 345, 346, 347].
* [cite_start]**Decision Making:** A Task Manager that coordinates picking and placing sequences autonomously[cite: 42].

## ⚙️ Tech Stack
* [cite_start]**Middleware:** ROS 2 Humble[cite: 361].
* [cite_start]**Simulation:** Gazebo[cite: 362].
* **Languages:** Python (`rclpy`), C++.
* [cite_start]**Core Libraries:** `nav2` stack, `slam_toolbox`, `OpenCV`, `NumPy`[cite: 387, 32, 33].

## 🚀 Installation & Build

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/your-username/eurobot_perception.git](https://github.com/your-username/eurobot_perception.git)
    cd eurobot_perception
    ```

2.  **Install Dependencies:**
    ```bash
    # ROS 2 Standard Packages
    sudo apt install ros-humble-sensor-msgs ros-humble-cv-bridge ros-humble-nav2-bringup ros-humble-slam-toolbox
    # Python Dependencies
    pip3 install opencv-python numpy
    ```
    *[Reference: cite 208, 210]*

3.  **Build the Workspace:**
    ```bash
    colcon build --symlink-install --packages-select eurobot_interfaces eurobot_perception eurobot_navigation
    source install/setup.bash
    ```
    *[Reference: cite 214]*

## ▶️ Usage

To launch the full simulation loop (Simulation + Navigation + Perception + Task Manager):

```bash
ros2 launch eurobot_bringup main_simulation.launch.py