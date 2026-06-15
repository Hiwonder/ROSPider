# ROSpider

[English](README.md) | 中文

<p align="center">
  ROSpider 是我们面向 Jetson 平台打造的 ROS2 六足机器人平台，集智能运动、AI 视觉、SLAM 建图、自主导航、语音交互与多场景实验于一体。
</p>

<p align="center">
  <img src="./sources/01.png" alt="ROSpider 1" width="600"/>
</p>

## 产品概述

### 关于 ROSpider

ROSpider 是我们基于 ROS2 打造的开源六足机器人平台，面向开发者、教育用户、创客和机器人学习者而设计。它不只是一个基础行走演示，而是把运动控制、环境感知、建图导航、语音交互和应用层 ROS 工作流整合在同一个工作区中，让你能用更少的准备工作，从硬件 bringup 进入高级自主行为开发。

我们做 ROSpider，是为了解决足式机器人开发中一个常见问题：很多项目能展示运动控制，很多项目也能展示感知能力，但真正把底层舵机控制到真实硬件上的 ROS 应用串成完整链路的项目并不多。ROSpider 通过一套可直接扩展的软件栈，把这条链路补齐。

<p align="center">
  <img src="./sources/02.gif" alt="ROSpider 2" width="600"/>
</p>

无论你是做机器人教学、原型验证，还是想探索 Jetson 硬件上的运动与感知融合，ROSpider 都能提供一套完整且容易上手的基础平台。

### 核心：面向真实部署的六足机器人平台

ROSpider 采用六足机器人结构，配备 **18 路腿部总线舵机**，并带有一个 **相机云台关节**，既能实现丰富的机体运动，也能支持主动视觉跟踪。

**完整的全身运动控制能力**：`driver` 相关功能包提供舵机控制、运动学、内置姿态、动作组执行、步态控制、里程计发布以及六足底盘控制接口。

**面向真机的一体化硬件接入**：`bringup` 和 `peripherals` 功能包会统一启动机器人控制、手柄与键盘控制、RGB 灯、IMU、相机、激光雷达和其他板载设备，提供接近真实部署的起点，而不是分散的算法演示。

**面向 Jetson 的 ROS2 设计**：工作区围绕 Jetson 设备上的 **ROS2 Humble** 组织，launch 文件与环境脚本也按照官方 ROSpider 系统镜像准备。

<p align="center">
  <img src="./sources/03.gif" alt="ROSpider 3" width="600"/>
</p>

### 软件栈：运动、视觉、SLAM 与交互

ROSpider 不是单一用途仓库，而是一套完整的 ROS2 应用平台。

**AI 视觉应用**：`app`、`example`、`large_models` 和 `large_models_examples` 包含目标跟踪、巡线、手势交互、智能踢球、颜色与 AprilTag 示例、MediaPipe 示例、YOLO 示例以及大模型应用示例。

**自平衡与反应式行为**：应用层包含基于 IMU 的自平衡、雷达行为、巡线、目标跟踪等从感知到运动的闭环示例，让机器人能根据环境实时做出反应。

**SLAM 与导航能力**：`slam` 提供 SLAM Toolbox、RTAB-Map 等 ROS2 建图流程相关 launch 文件。`navigation` 提供定位、Nav2 导航、地图加载、RViz 启动文件以及 RTAB-Map 导航支持。

**离线语音交互**：`xf_mic_asr_offline` 和 `xf_mic_asr_offline_msgs` 提供离线语音交互与语音控制能力。

**竞赛与多场景示例**：`competition` 和 `example` 提供过桥、窄缝通过、抓取放置、智能搬运、导航搬运等面向课堂、比赛或场景化任务的示例。

<p align="center">
  <img src="./sources/04.png" alt="ROSpider 4" width="600"/>
</p>

### 面向学习、扩展与创意开发

ROSpider 不只是产品平台，同时也是一套适合教学和二次开发的学习平台。

**示例体系完整**：`example` 提供大量 ROS2 脚本和 launch 文件，覆盖机体控制、步态控制、OpenCV 项目、MediaPipe 示例、导航搬运、目标分类、颜色分拣和创意交互等内容。

**ROS2 包结构清晰**：仓库将 bringup、driver、peripherals、interfaces、navigation、slam、applications、competition examples、large-model examples、simulations 和 voice interaction 等功能拆分清楚，方便理解、定制和扩展。

**工程化工作流更省心**：仓库中的 launch 文件和官方镜像环境围绕可运行的 ROS2 机器人工作流设计，帮助你从一套可用的平台开始，而不是从零搭建基础设施。

## 官方资源

### 幻尔科技官方

- **官方网站**: [https://www.hiwonder.com/](https://www.hiwonder.com/)
- **产品页面**: [https://www.hiwonder.com/products/rospider](https://www.hiwonder.com/products/rospider)
- **官方文档**: [https://docs.hiwonder.com/projects/ROSpider/en/jetson-orin-nano-version/](https://docs.hiwonder.com/projects/ROSpider/en/jetson-orin-nano-version/)
- **技术支持**: support@hiwonder.com

## 快速开始

### 推荐环境

- Ubuntu 22.04
- ROS2 Humble
- Python 3
- Jetson 系主控
- 搭载激光雷达、RGB/深度相机、IMU 和总线舵机的 ROSpider 硬件平台

### 安装

1. 将仓库克隆为 ROS2 工作区：

```bash
git clone https://github.com/hiwonder/ROSPider.git ~/ros2_ws
cd ~/ros2_ws
```

2. 安装 ROS 依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. 编译工作区：

```bash
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/local_setup.bash
```

4. 如果你使用官方系统镜像，可以加载 ROSpider 预设环境：

```bash
source ~/.robotrc
```

该脚本会加载 ROS2 Humble、ROSpider 工作区以及官方镜像使用的设备环境。

### 常用启动命令

启动基础机器人功能：

```bash
ros2 launch bringup bringup.launch.py
```

启动完整应用栈：

```bash
ros2 launch app start_app.launch.py
```

启动 SLAM：

```bash
ros2 launch slam slam.launch.py slam_method:=slam_toolbox
```

启动基于地图的导航：

```bash
ros2 launch navigation navigation.launch.py map:=map_01
```

启动典型应用：

```bash
ros2 launch app object_tracking_node.launch.py
ros2 launch app hand_gesture.launch.py
ros2 launch app self_balancing_node.launch.py
```

启动外设可视化：

```bash
ros2 launch peripherals lidar_view.launch.py
ros2 launch peripherals depth_camera.launch.py
```

## 仓库结构

```text
ROSpider/
└── src/
    ├── app/                    # 目标跟踪、巡线、自平衡、手势交互等应用
    ├── bringup/                # 基础机器人 bringup 和启动检查
    ├── competition/            # 竞赛和场景化机器人任务
    ├── driver/                 # 控制器、运动学、舵机、SDK 和硬件驱动
    ├── example/                # 机体控制、OpenCV、MediaPipe、搬运和教程示例
    ├── interfaces/             # 应用层使用的 ROS2 服务和接口
    ├── large_models/           # 大模型运行资源和相关代码
    ├── large_models_examples/  # 大模型应用示例
    ├── navigation/             # Nav2、定位、地图加载和 RTAB-Map 导航
    ├── peripherals/            # 相机、雷达、IMU、手柄、键盘和传感器支持
    ├── simulations/            # 仿真相关文件
    ├── slam/                   # SLAM Toolbox、RTAB-Map、RViz 和建图 launch
    ├── xf_mic_asr_offline/     # 离线语音识别和语音控制
    └── xf_mic_asr_offline_msgs/# 离线语音识别消息
```

## 社区与支持

- **GitHub Issues**: 提交问题反馈和功能建议
- **邮件支持**: support@hiwonder.com
- **文档资料**: 完整的教程指南

## 许可证

本项目开源，可用于教育和研究目的。

---

**幻尔科技** - 赋能机器人教育创新
