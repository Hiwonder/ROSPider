# ROSSpider

English | [中文](README_cn.md)

<p align="center">
  A ROS-based hexapod robot workspace for locomotion, vision, SLAM, navigation, voice interaction, and multi-robot coordination.
</p>

## Product Overview

### About ROSSpider

ROSpider is an open-source hexapod robot workspace built around ROS and real hardware deployment. Instead of being just a gait demo or a single perception example, this repository combines low-level servo control, whole-body kinematics, sensor drivers, AI vision applications, SLAM, autonomous navigation, and even multi-robot formation into one integrated development stack.

The pain point it solves is straightforward: legged robots are exciting, but getting one to move stably, perceive the environment, and run real ROS applications usually means stitching together many disconnected packages yourself. ROSSpider packages that work into a practical stack, so you can move from hardware bringup to vision tracking, mapping, route planning, and experimental interaction much faster.

If you are building a teaching robot, prototyping a Jetson-based legged platform, or learning how locomotion and perception fit together in ROS, ROSSpider gives you a codebase that is much closer to a complete robot product than a toy example.

### The Core: A Hexapod Platform Built for Real ROS Work

ROSpider is clearly structured as a six-legged robot system with **18 bus servos** for the legs plus a **camera pan joint**. The controller package exposes leg-level and body-level pose control, built-in poses, action-set playback, gait control, and velocity-based motion interfaces, making it suitable both for direct motion experiments and for higher-level autonomy.

**Hexapod motion control**: The `rospider_controller` package handles inverse kinematics, joint control, built-in poses, action groups, and gait generation. From the code, the platform supports tripod and ripple-style locomotion, body translation/rotation transforms, head yaw control, and raw odometry publication.

**Sensor-rich bringup**: The default bringup stack starts robot description, joystick control, RGB lighting, IMU, camera, LiDAR, OLED display, and the main controller together. This means the repository is designed around an actual robot with onboard sensing, not just simulation assets.

**Jetson-oriented deployment**: Multiple packages use `Jetson.GPIO`, and the provided startup scripts source **ROS Melodic** plus a robot workspace under `~/rospider`. That makes this repository especially relevant for NVIDIA Jetson based educational and research robots.

### The Software Stack: From Perception Demos to Autonomy

ROSpider is more than locomotion. The workspace includes a broad software ecosystem:

**Vision applications**: `rospider_app` and `rospider_tutorial` include color tracking, AprilTag tracking, hand gesture recognition, line following, face-related demos, pose-related demos, KCF tracking, AR overlays, color detection, and MediaPipe-based interaction experiments.

**Self-balancing and reactive behaviors**: The app layer contains a self-balancing node driven by IMU feedback and PID control, plus reactive behaviors such as line following and object tracking that connect perception directly to movement.

**SLAM and navigation**: `rospider_slam` supports multiple mapping backends including **GMapping**, **Karto**, **Hector**, and **Cartographer**, while `rospider_navigation` integrates map loading, AMCL localization, move_base, and multi-point navigation publishing.

**Voice interaction**: The `xf_mic_asr_offline` package adds offline speech interaction and voice-triggered behaviors, including voice-controlled movement, color tracking, and navigation workflows.

**Multi-robot experiments**: `rospider_multi` includes formation and coordinated navigation launch files, making the project useful for swarm-style or classroom multi-robot experiments as well.

### Learning & Extensibility: A Full Workspace, Not Just a Demo Folder

This repository is especially valuable as a learning platform.

**Tutorial coverage**: `rospider_tutorial` contains a large number of scripts and launch files covering board-level IO, servo control, gait basics, IK, OpenCV projects, AR demos, deep learning demos, and creative interaction examples.

**Modular ROS packages**: Core capabilities are separated into bringup, controller, SDK, peripherals, interfaces, navigation, SLAM, app, tutorial, and third-party dependency packages. That modularity makes it easier to swap sensors, add new nodes, or reuse parts of the stack in another robot.

**Practical integration path**: The included environment scripts define robot names, master names, LiDAR and camera types, and network settings, which is exactly the kind of infrastructure a real deployed ROS robot needs.

## Official Resources

### Official Hiwonder

- **Official Website**: [https://www.hiwonder.com/](https://www.hiwonder.com/)
- **Product Page**: [https://www.hiwonder.com/products/rospider](https://www.hiwonder.com/products/rospider)
- **Official Documentation**: [https://docs.hiwonder.com/en/latest/jetson/](https://docs.hiwonder.com/en/latest/jetson/)
- **Technical Support**: support@hiwonder.com

## Getting Started

### Recommended Environment

- Ubuntu 18.04
- ROS Melodic
- Python 3
- Jetson-based controller board
- ROSSpider hardware with LiDAR, RGB camera, IMU, and serial bus servos

### Installation

1. Clone the repository as a catkin workspace:

```bash
git clone https://github.com/hiwonder/ROSPider.git ~/rospider
cd ~/rospider
```

2. Install ROS dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:

```bash
catkin_make
source devel/setup.bash
```

4. If you are using the official robot image, you can also load the preset environment:

```bash
source ~/.hiwonderrc
```

This script configures variables such as `ROBOT_NAME`, `MASTER_NAME`, `LIDAR_TYPE`, `CAMERA_TYPE`, `ROS_HOSTNAME`, and `ROS_MASTER_URI`, then sources ROS Melodic and the workspace setup files.

### Typical Launch Commands

Bring up the base robot stack:

```bash
roslaunch rospider_bringup base.launch
```

Bring up the full app stack:

```bash
roslaunch rospider_bringup app_bringup.launch
```

Start SLAM:

```bash
roslaunch rospider_slam rospider_slam.launch slam_methods:=gmapping
```

Start map-based navigation:

```bash
roslaunch rospider_navigation rospider_navigation.launch map:=/path/to/map.yaml
```

Start a sample app:

```bash
roslaunch rospider_app object_tracking.launch
roslaunch rospider_app hand_gesture.launch
roslaunch rospider_app self_balancing.launch
```

Start multi-robot formation control:

```bash
roslaunch rospider_multi multi_formation/multi_formation.launch
```

## Repository Structure

```text
ROSPider/
└── src/
    ├── rospider_bringup/        # Base bringup, rosbridge, startup scripts
    ├── rospider_controller/     # Hexapod control, IK, gait, odometry, action sets
    ├── rospider_description/    # URDF/Xacro robot model and visualization assets
    ├── rospider_peripherals/    # Camera, LiDAR, IMU, OLED, joystick, RGB support
    ├── rospider_app/            # Object tracking, line following, self-balancing, gestures
    ├── rospider_navigation/     # AMCL, move_base, map loading, point publishing
    ├── rospider_slam/           # GMapping, Karto, Hector, Cartographer, RTAB-Map related launch
    ├── rospider_multi/          # Multi-robot formation and coordinated navigation
    ├── rospider_sdk/            # Hardware access helpers and robot SDK utilities
    ├── rospider_interfaces/     # ROS services and interfaces used by applications
    ├── rospider_tutorial/       # Example scripts and tutorial launch files
    ├── lab_config/              # LAB color-threshold configuration tools
    ├── dataset_capture/         # Dataset/image capture utilities
    ├── vision_utils/            # Shared vision helper functions
    ├── xf_mic_asr_offline/      # Offline speech recognition and voice control
    └── third_party/             # Bundled ROS dependencies and upstream packages
```

## Community & Support

- **GitHub Issues**: Report bugs, integration problems, and feature requests
- **Email Support**: support@hiwonder.com
- **Tutorial Packages**: Explore `rospider_tutorial` for hands-on learning examples

## License

This repository does not currently include a top-level license file, and several package manifests still use placeholder license fields. If you plan to redistribute or use the code commercially, it is best to confirm the licensing terms with the maintainer first.

---

**Hiwonder** - Empowering Innovation in Robotics Education
