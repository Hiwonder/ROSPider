# ROSSpider

[English](README.md) | 中文

<p align="center">
  一个面向六足蜘蛛机器人的 ROS 工作区，覆盖运动控制、视觉感知、SLAM 建图、导航、语音交互和多机协同。
</p>

## 产品概述

### 关于 ROSSpider

ROSpider 是一个面向真实硬件落地的开源六足机器人 ROS 工作区。它不是单独的步态 demo，也不是只做视觉识别的小项目，而是把底层舵机控制、全身运动学、传感器驱动、AI 视觉玩法、SLAM 建图、自主导航，甚至多机编队，整合进了一套完整开发栈里。

它解决的痛点其实很现实：足式机器人很酷，但真想让机器人“又能稳走、又能看懂环境、还能跑 ROS 应用”，往往需要自己拼很多零散包。ROSpider 把这些环节提前打通了，让你能更快从硬件 bringup 走到视觉跟踪、建图、路径规划和交互实验。

如果你想做教学机器人、想搭一个基于 Jetson 的足式平台，或者单纯想搞明白“运动控制”和“环境感知”在 ROS 里是怎么串起来的，那这个仓库比普通示例工程更像一套可以直接干活的机器人系统。

### 核心：一套能真正跑起来的六足机器人平台

从代码结构看，ROSpider 是一套很完整的六足蜘蛛机器人系统，包含 **18 路腿部总线舵机**，外加一个 **相机云台关节**。`rospider_controller` 对外提供了腿端控制、机体姿态控制、内置姿态、动作组播放、步态控制，以及基于速度的话题接口，既适合直接做动作实验，也方便往上接导航和自主行为。

**六足运动控制**：`rospider_controller` 负责逆运动学、关节控制、内置姿态、动作组、步态生成和原始里程计发布。从代码里能看出来，它支持 tripod、ripple 等步态模式，支持机体平移/旋转变换、头部偏航控制，以及 `cmd_vel` 风格的运动接口。

**传感器整体 bringup**：默认启动栈会一起拉起机器人模型、手柄控制、RGB 灯、IMU、相机、激光雷达、OLED 显示和主控制节点。这说明它不是只面向仿真的代码仓，而是围绕真实机器人硬件设计的。

**偏 Jetson 的部署方式**：多个包直接使用了 `Jetson.GPIO`，而且启动脚本会自动加载 **ROS Melodic** 和 `~/rospider` 工作区环境。也就是说，这个仓库天然更适合跑在 Jetson 系教育或研究机器人上。

### 软件栈：从视觉玩法一路到自主导航

ROSpider 的重点不只是“会走路”，它的软件层很完整：

**视觉应用**：`rospider_app` 和 `rospider_tutorial` 里包含颜色跟踪、AprilTag 跟踪、手势识别、巡线、人脸相关 demo、姿态相关 demo、KCF 跟踪、AR 叠加、颜色识别和基于 MediaPipe 的交互玩法。

**自平衡和反应式行为**：应用层里有基于 IMU + PID 的自平衡节点，也有巡线、目标跟踪这种“看到什么就怎么动”的闭环行为，感知和运动控制是直接打通的。

**SLAM 与导航**：`rospider_slam` 支持 **GMapping、Karto、Hector、Cartographer** 等多种建图方案；`rospider_navigation` 则把地图加载、AMCL 定位、move_base 和多点导航串了起来。

**语音交互**：`xf_mic_asr_offline` 提供离线语音识别和语音控制能力，代码里已经包括语音控制移动、颜色跟踪和导航等功能入口。

**多机协同**：`rospider_multi` 提供了编队和协同导航相关启动文件，适合做多机器人教学、实验或编队演示。

### 学习与扩展：这不是示例文件夹，而是一整套工作区

这个仓库很适合拿来学习，也很适合继续二开。

**教程覆盖面广**：`rospider_tutorial` 下面有大量脚本和 launch，覆盖板载 IO、舵机控制、基础步态、逆运动学、OpenCV 玩法、AR demo、深度学习 demo、创意交互等内容。

**ROS 包拆分清楚**：bringup、controller、SDK、peripherals、interfaces、navigation、slam、app、tutorial、third_party 都是独立包，后续你想换传感器、加新节点、或者把某一层单独复用到别的机器人上，都比较顺手。

**工程化程度比较高**：仓库自带环境初始化脚本，里面已经配置了机器人命名空间、主机名称、雷达类型、相机类型和网络参数。这种东西往往是“真机部署”时最容易踩坑的部分，这里已经提前铺好了。

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
- 搭载激光雷达、RGB 相机、IMU 和总线舵机的 ROSSpider 硬件平台

### 安装

1. 先把仓库克隆成一个 catkin 工作区：

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

4. 如果你用的是官方系统镜像，也可以直接加载预设环境：

```bash
source ~/.hiwonderrc
```

这个脚本会自动设置 `ROBOT_NAME`、`MASTER_NAME`、`LIDAR_TYPE`、`CAMERA_TYPE`、`ROS_HOSTNAME`、`ROS_MASTER_URI`，并加载 ROS Melodic 和当前工作区环境。

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
ROSPider/
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
    ├── rospider_interfaces/     # 应用层用到的 ROS 接口定义
    ├── rospider_tutorial/       # 示例脚本和教程 launch
    ├── lab_config/              # LAB 颜色阈值配置工具
    ├── dataset_capture/         # 数据采集/图像采集工具
    ├── vision_utils/            # 通用视觉工具函数
    ├── xf_mic_asr_offline/      # 离线语音识别和语音控制
    └── third_party/             # 随仓库附带的第三方 ROS 依赖
```

## 社区与支持

- **GitHub Issues**: 用来反馈 Bug、集成问题和功能建议
- **邮件支持**: support@hiwonder.com
- **教程入口**: 可以优先从 `rospider_tutorial` 里的示例开始上手

## 许可证

这个仓库目前没有顶层 `LICENSE` 文件，而且多个包的 `package.xml` 里许可证字段还是占位写法。如果你后面打算分发代码，或者要商用，建议先和维护者确认具体授权方式。

---

**幻尔科技** - 赋能机器人教育创新
