# ROSpider

English | [中文](README_cn.md)

<p align="center">
  ROSpider is our ROS2-based hexapod robot platform built for intelligent locomotion, AI vision, SLAM, autonomous navigation, voice interaction, and multi-robot experimentation on Jetson.
</p>

<p align="center">
  <img src="./sources/01.png" alt="ROSpider 1" width="600"/>
</p>

## Product Overview

### About ROSpider

ROSpider is our open-source hexapod robot platform built on ROS2 and designed for developers, educators, makers, and robotics learners who want more than a basic walking demo. It brings together motion control, perception, mapping, navigation, voice interaction, and application-level ROS workflows in one integrated workspace, so you can move from hardware bringup to advanced autonomous behaviors with far less setup effort.

We built ROSpider to solve a common problem in legged robotics: many projects can demonstrate motion, and many others can demonstrate perception, but very few connect the full chain from low-level servo control to practical ROS applications on real hardware. ROSpider closes that gap by providing a ready-to-extend software stack for a real six-legged robot platform.

<p align="center">
  <img src="./sources/02.gif" alt="ROSpider 2" width="600"/>
</p>

Whether you are teaching robotics, prototyping interactive AI applications, or exploring how locomotion and perception work together on Jetson-based hardware, ROSpider provides a complete and approachable foundation.

### The Core: A Hexapod Platform Designed for Real Deployment

ROSpider is built around a six-legged robot architecture with **18 bus servos** for the legs and an additional **camera pan joint**, giving the platform both expressive body motion and active visual tracking capability.

**Full-body motion control**: The `driver` packages provide servo control, kinematics, built-in poses, action-set execution, gait control, odometry publishing, and controller interfaces for the hexapod chassis.

**Rich onboard hardware integration**: The `bringup` and `peripherals` packages launch robot control, joystick and keyboard control, RGB lighting, IMU, camera, LiDAR, and other onboard devices together, providing a practical starting point for real robot deployment instead of an isolated algorithm demo.

**Jetson-oriented ROS2 design**: The workspace is organized around **ROS2 Humble** on Jetson-class devices, with launch files and environment scripts prepared for the official ROSpider system image.

<p align="center">
  <img src="./sources/03.gif" alt="ROSpider 3" width="600"/>
</p>

### The Software Stack: Motion, Vision, SLAM, and Interaction

ROSpider is designed as a complete ROS2 application platform rather than a single-purpose repository.

**AI vision applications**: `app`, `example`, `large_models`, and `large_models_examples` include object tracking, line following, hand gesture interaction, intelligent kicking, color and AprilTag demos, MediaPipe examples, YOLO-based examples, and large-model application demos.

**Self-balancing and reactive behaviors**: The application layer includes IMU-driven self-balancing, LiDAR behaviors, line following, object tracking, and other perception-to-motion examples that connect sensing directly to robot movement.

**SLAM and navigation**: `slam` provides ROS2 mapping workflows such as SLAM Toolbox and RTAB-Map related launch files. `navigation` provides localization, Nav2-based navigation, map loading, RViz launch files, and RTAB-Map navigation support.

**Offline voice interaction**: The `xf_mic_asr_offline` and `xf_mic_asr_offline_msgs` packages enable offline speech interaction and voice-controlled robot behaviors.

**Competition and multi-scene examples**: `competition` and `example` provide scenario-oriented demos such as crossing bridges, narrow-slit traversal, pick and place, intelligent transport, navigation transport, and other classroom or contest-style tasks.

<p align="center">
  <img src="./sources/04.png" alt="ROSpider 4" width="600"/>
</p>

### Built for Learning, Expansion, and Creative Development

ROSpider is not just a product platform. It is also a development and teaching platform.

**Comprehensive examples**: `example` includes a wide range of ROS2 scripts and launch files covering body control, gait control, OpenCV projects, MediaPipe demos, navigation transport, object classification, color sorting, and other creative interaction examples.

**Modular ROS2 package layout**: Core functions are split into bringup, driver, peripherals, interfaces, navigation, SLAM, applications, competition examples, large-model examples, simulations, and voice interaction packages, making it easier to understand, customize, and extend the system.

**Practical engineering workflow**: The included launch files and official image environment are designed around a working ROS2 robot workflow, helping you start from a ready-to-run robot platform instead of rebuilding infrastructure from scratch.

## Official Resources

### Official Hiwonder

- **Official Website**: [https://www.hiwonder.com/](https://www.hiwonder.com/)
- **Product Page**: [https://www.hiwonder.com/products/rospider](https://www.hiwonder.com/products/rospider)
- **Official Documentation**: [https://docs.hiwonder.com/projects/ROSpider/en/jetson-orin-nano-version/](https://docs.hiwonder.com/projects/ROSpider/en/jetson-orin-nano-version/)
- **Technical Support**: support@hiwonder.com

## Getting Started

### Recommended Environment

- Ubuntu 22.04
- ROS2 Humble
- Python 3
- Jetson-based controller board
- ROSpider hardware with LiDAR, RGB/depth camera, IMU, and serial bus servos

### Installation

1. Clone the repository as a ROS2 workspace:

```bash
git clone https://github.com/hiwonder/ROSPider.git ~/ros2_ws
cd ~/ros2_ws
```

2. Install ROS dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:

```bash
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/local_setup.bash
```

4. If you are using the official system image, load the preset ROSpider environment:

```bash
source ~/.robotrc
```

This script loads ROS2 Humble, the ROSpider workspace, and the device environment used by the official image.

### Typical Launch Commands

Bring up the base robot stack:

```bash
ros2 launch bringup bringup.launch.py
```

Bring up the full app stack:

```bash
ros2 launch app start_app.launch.py
```

Start SLAM:

```bash
ros2 launch slam slam.launch.py slam_method:=slam_toolbox
```

Start map-based navigation:

```bash
ros2 launch navigation navigation.launch.py map:=map_01
```

Start sample applications:

```bash
ros2 launch app object_tracking_node.launch.py
ros2 launch app hand_gesture.launch.py
ros2 launch app self_balancing_node.launch.py
```

Start peripheral visualization:

```bash
ros2 launch peripherals lidar_view.launch.py
ros2 launch peripherals depth_camera.launch.py
```

## Repository Structure

```text
ROSpider/
└── src/
    ├── app/                    # Object tracking, line following, self-balancing, gestures
    ├── bringup/                # Base robot bringup and startup checks
    ├── competition/            # Competition and scenario-oriented robot tasks
    ├── driver/                 # Controller, kinematics, servo, SDK, and hardware drivers
    ├── example/                # Body control, OpenCV, MediaPipe, transport, and tutorial demos
    ├── interfaces/             # ROS2 services and interfaces used by applications
    ├── large_models/           # Large-model runtime assets and related code
    ├── large_models_examples/  # Large-model application examples
    ├── navigation/             # Nav2, localization, map loading, and RTAB-Map navigation
    ├── peripherals/            # Camera, LiDAR, IMU, joystick, keyboard, and sensor support
    ├── simulations/            # Simulation-related files
    ├── slam/                   # SLAM Toolbox, RTAB-Map, RViz, and mapping launch files
    ├── xf_mic_asr_offline/     # Offline speech recognition and voice control
    └── xf_mic_asr_offline_msgs/# Messages for offline speech recognition
```

## Community & Support

- **GitHub Issues**: Report bugs and request features
- **Email Support**: support@hiwonder.com
- **Documentation**: Comprehensive guides and tutorials

## License

This project is open-source and available for educational and research purposes.

---

**Hiwonder** - Empowering Innovation in Robotics Education
