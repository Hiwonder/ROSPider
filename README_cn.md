# ROSpider

[English](README.md) | 中文

<p align="center">
  ROSpider 是我们面向 Jetson 平台打造的 ROS1 六足蜘蛛机器人开发平台，集运动控制、AI 视觉、SLAM 建图、自主导航与多机协同于一体。
</p>

## 产品概述

### 关于 ROSpider

ROSpider 是我们基于 ROS1 打造的开源六足蜘蛛机器人平台，面向开发者、教育用户、创客和机器人学习者而设计。它不只是一个“能走路”的六足底盘，也不只是一个单独的视觉 demo，而是一套把底层控制、环境感知、建图导航、语音交互和应用层 ROS 工作流整合在一起的完整开发栈。

我们做 ROSpider，解决的是足式机器人开发里一个非常常见的问题: 很多项目能展示运动控制，很多项目也能展示视觉识别，但真正把底层舵机控制、全身运动学、传感器接入、SLAM、导航和交互应用串成一套完整系统的项目并不多。ROSpider 的目标，就是把这一整条链路打通，让你能更快从硬件 bringup 进入到高级应用开发。

无论你是做机器人教学、做 Jetson 平台原型验证，还是想系统学习足式机器人在 ROS 中的完整开发流程，ROSpider 都能提供一套清晰、完整、可扩展的基础平台。

### 核心：面向真实部署的六足机器人平台

ROSpider 采用六足蜘蛛机器人结构，配备 **18 路腿部总线舵机**，并带有一个 **相机云台关节**，既能实现灵活稳定的全身运动，也能支持视觉跟踪和交互类应用。

**完整的六足运动控制能力**：`rospider_controller` 集成了逆运动学、关节控制、内置姿态、动作组执行、步态生成以及基于速度的运动控制接口，支持机体平移与旋转姿态变换、头部偏航控制，以及原始里程计发布，方便继续对接更高层 ROS 功能。

**面向真机的一体化硬件接入**：基础 bringup 会同时启动机器人模型、手柄控制、RGB 灯、IMU、相机、激光雷达、OLED 显示和主控制节点，开箱就是一套完整的机器人运行框架，而不是拆散的功能样例。

**针对 Jetson 平台优化**：多个功能包直接使用 `Jetson.GPIO`，环境脚本默认围绕 **ROS Melodic** 和 Jetson 工作区配置。对需要在 Jetson 平台上进行教育、开发和应用落地的用户来说，ROSpider 上手路径更直接。

### 软件栈：从运动控制到 AI 感知与自主导航

ROSpider 的价值不只是“能控制六足运动”，更在于它提供了完整的软件生态。

**AI 视觉应用**：`rospider_app` 与 `rospider_tutorial` 包含目标跟踪、颜色跟踪、AprilTag 跟踪、巡线、手势识别、AR 玩法、人脸相关 demo、姿态相关 demo、KCF 跟踪以及基于 MediaPipe 的交互示例。

**自平衡与反应式行为**：应用层提供了基于 IMU + PID 的自平衡能力，也提供了巡线、目标跟踪这类“感知驱动运动”的闭环行为，让机器人不仅能动，还能根据环境实时做出反应。

**SLAM 与导航能力**：`rospider_slam` 支持 **GMapping、Karto、Hector、Cartographer** 等多种建图方案，`rospider_navigation` 则进一步整合了地图加载、AMCL 定位、move_base 和多点导航能力。

**离线语音交互**：`xf_mic_asr_offline` 提供离线语音识别与语音控制能力，可用于控制机器人运动、颜色相关任务和导航类功能。

**多机协同扩展**：`rospider_multi` 提供编队和协同导航启动文件，让 ROSpider 不只是单机平台，也能支持多机演示、教学实验和协同研究。

### 面向学习、扩展与创意开发

ROSpider 不只是产品平台，同时也是一套非常适合教学和二次开发的学习平台。

**教程体系完整**：`rospider_tutorial` 提供大量脚本和 launch 文件，覆盖舵机控制、板载 IO、基础步态、逆运动学、OpenCV 项目、AR demo、深度学习 demo 和创意交互玩法。

**ROS 包结构清晰**：仓库将 bringup、controller、SDK、peripherals、interfaces、navigation、slam、applications、tutorials 以及 third-party 依赖做了明确拆分，方便理解整体架构，也方便后续扩展。

**工程化配置更省心**：我们在环境脚本中预设了机器人命名、主机命名、雷达类型、相机类型和 ROS 网络参数，帮助你从一套更接近真实部署的工作流开始开发，而不是从零拼装环境。

## 官方资源

### 幻尔科技官方

- **官方网站**: [https://www.hiwonder.com/](https://www.hiwonder.com/)
- **产品页面**: [https://www.hiwonder.com/products/rospider](https://www.hiwonder.com/products/rospider)
- **官方文档**: [https://docs.hiwonder.com/en/latest/jetson/](https://docs.hiwonder.com/en/latest/jetson/)
- **技术支持**: support@hiwonder.com

## 快速开始

### 推荐环境

- Ubuntu 18.04
- ROS Melodic
- Python 3
- Jetson 系主控
- 搭载激光雷达、RGB 相机、IMU 和总线舵机的 ROSpider 硬件平台

### 安装

1. 将仓库克隆为 catkin 工作区：

```bash
git clone https://github.com/hiwonder/ROSPider.git ~/rospider
cd ~/rospider
```

2. 安装 ROS 依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. 编译工作区：

```bash
catkin_make
source devel/setup.bash
```

4. 如果你使用官方系统镜像，也可以直接加载预设环境：

```bash
source ~/.hiwonderrc
```

该脚本会自动设置 `ROBOT_NAME`、`MASTER_NAME`、`LIDAR_TYPE`、`CAMERA_TYPE`、`ROS_HOSTNAME`、`ROS_MASTER_URI`，并加载 ROS Melodic 与 ROSpider 工作区环境。

### 常用启动命令

启动基础机器人功能：

```bash
roslaunch rospider_bringup base.launch
```

启动完整应用栈：

```bash
roslaunch rospider_bringup app_bringup.launch
```

启动 SLAM：

```bash
roslaunch rospider_slam rospider_slam.launch slam_methods:=gmapping
```

启动基于地图的导航：

```bash
roslaunch rospider_navigation rospider_navigation.launch map:=/path/to/map.yaml
```

启动一个典型应用：

```bash
roslaunch rospider_app object_tracking.launch
roslaunch rospider_app hand_gesture.launch
roslaunch rospider_app self_balancing.launch
```

启动多机编队控制：

```bash
roslaunch rospider_multi multi_formation/multi_formation.launch
```

## 仓库结构

```text
ROSpider/
└── src/
    ├── rospider_bringup/        # 基础 bringup、rosbridge、启动脚本
    ├── rospider_controller/     # 六足控制、逆运动学、步态、里程计、动作组
    ├── rospider_description/    # URDF/Xacro 模型和可视化资源
    ├── rospider_peripherals/    # 相机、雷达、IMU、OLED、手柄、RGB 等外设支持
    ├── rospider_app/            # 目标跟踪、巡线、自平衡、手势交互等应用
    ├── rospider_navigation/     # AMCL、move_base、地图加载、多点导航
    ├── rospider_slam/           # GMapping、Karto、Hector、Cartographer、RTAB-Map 相关启动
    ├── rospider_multi/          # 多机编队和协同导航
    ├── rospider_sdk/            # 硬件访问封装和 SDK 工具
    ├── rospider_interfaces/     # 应用层使用的 ROS 接口定义
    ├── rospider_tutorial/       # 示例脚本和教程 launch
    ├── lab_config/              # LAB 颜色阈值配置工具
    ├── dataset_capture/         # 数据采集与图像采集工具
    ├── vision_utils/            # 通用视觉工具函数
    ├── xf_mic_asr_offline/      # 离线语音识别和语音控制
    └── third_party/             # 仓库附带的第三方 ROS 依赖
```

## 社区与支持

- **GitHub Issues**: 提交问题反馈和功能建议
- **邮件支持**: support@hiwonder.com
- **文档资料**: 完整的教程指南

## 许可证

本项目开源，可用于教育和研究目的。

---

**幻尔科技** - 赋能机器人教育创新
