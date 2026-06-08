# robot_description — AGT 比赛机器人 URDF 模型与可视化

> **定位：** URDF 模型与可视化包，提供 AGT 比赛机器人的完整运动学描述、3D 网格文件和 RViz2 可视化配置。
> **核心依赖：** `robot_state_publisher`（TF 发布）、`joint_state_publisher_gui`（关节调试面板）、`rviz2`（3D 可视化）
> **下游消费者：** `robot_gazebo`（仿真模型加载）、`robot_startup`（TF 树发布）、`at_nav2`（坐标系参考与导航）

---

## 1. 包概述

本包是 AGT 轮式机器人的物理模型定义包，核心职责：

- 提供完整的 **URDF 模型**（`urdf/robot_description.urdf`），包含所有 link、joint 的几何、惯量和碰撞属性
- 通过 `robot_state_publisher` 解析 URDF 并发布 **TF 变换树**（`tf2_msgs/TFMessage`）
- 提供 `joint_state_publisher_gui` 调试界面，用于离线拖动关节验证模型
- 提供预置的 **RViz2 配置文件**，开箱即用查看机器人模型

本包本身不发布任何关节状态话题（`/joint_states`），仅负责 TF 发布。关节状态由 `robot_startup` 包从实际编码器数据生成，或由 `robot_gazebo` 在仿真中提供。

---

## 2. 包结构

```
robot_description/
├── CMakeLists.txt                          # 构建配置（安装 URDF/meshes/launch/rviz2）
├── package.xml                             # 包依赖声明
├── README.md                               # 本文档
├── urdf/
│   └── robot_description.urdf              # 机器人 URDF 模型（~874 行）
├── meshes/
│   ├── base_link.STL                       # 底盘主体
│   ├── laser_frame.STL                     # LiDAR 支架
│   ├── arm0_Link.STL .. arm4_Link.STL      # 5 自由度机械臂（5 段）
│   ├── LFs_Link.STL / LFd_Link.STL         # 左前轮（转向节 + 车轮）
│   ├── LRs_Link.STL / LRd_Link.STL         # 左后轮
│   ├── RFs_Link.STL / RFd_Link.STL         # 右前轮
│   └── RRs_Link.STL / RRd_Link.STL         # 右后轮
├── rviz2/
│   └── rviz.rviz                           # RViz2 可视化配置
└── launch/
    └── robot_description.launch.py         # 主启动文件
```

---

## 3. 依赖

### 3.1 apt 安装（ROS2 Humble）

```bash
sudo apt install ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-rviz2 \
                 ros-humble-gazebo-ros
```

> `gazebo-ros` 依赖是为了下游 `robot_gazebo` 仿真包使用，本包独立运行时不需要 Gazebo。

### 3.2 package.xml 声明的依赖

| 依赖 | 类型 | 说明 |
|------|------|------|
| `robot_state_publisher` | `<depend>` | 解析 URDF 并发布 TF |
| `joint_state_publisher_gui` | `<depend>` | 离线关节调试 GUI |
| `rviz2` | `<depend>` | 3D 可视化 |
| `gazebo_ros` | `<depend>` | 供下游仿真包使用 |
| `ament_cmake` | `<buildtool_depend>` | 构建系统 |

---

## 4. URDF 模型说明

### 4.1 机器人概览

该机器人是一个 **四轮独立转向 + 独立驱动** 的底盘平台，搭载一个 **5 自由度机械臂** 和一个 **LiDAR 传感器**。

- **驱动方式：** 四个车轮均可独立转向（continuous joint）和独立旋转（continuous joint），支持全方位移动
- **机械臂：** arm0（腰部回转）连续旋转，arm1~arm4 为 revolute joint，带角度限位
- **传感器：** 固定式单线 LiDAR，安装在底盘上方

### 4.2 完整 link 与 joint 列表

#### 底盘层级

| Link | Joint | Joint 类型 | Parent -> Child | 说明 |
|------|-------|------------|-----------------|------|
| `base_footprint` | — | — | — | 机器人在地面的投影原点（虚拟） |
| `base_link` | `base_joint` | fixed | base_footprint -> base_link | 底盘主体坐标系（z 偏移 +0.033m） |
| `laser` | `laser_joint` | fixed | base_link -> laser | LiDAR 传感器支架（z 偏移 +0.112m） |

#### 机械臂层级（5-DOF）

| Link | Joint | Joint 类型 | Parent -> Child | 限位 |
|------|-------|------------|-----------------|------|
| `arm0_Link` | `arm0` | continuous | base_link -> arm0_Link | 无限制（腰部回转） |
| `arm1_Link` | `arm1` | revolute | arm0_Link -> arm1_Link | -210deg ~ 0deg |
| `arm2_Link` | `arm2` | revolute | arm1_Link -> arm2_Link | 0deg ~ 270deg |
| `arm3_Link` | `arm3` | revolute | arm2_Link -> arm3_Link | 0deg ~ 100deg |
| `arm4_Link` | `arm4` | revolute | arm3_Link -> arm4_Link | -90deg ~ 90deg |

#### 四轮独立悬架层级

| Link | Joint | Joint 类型 | Parent -> Child | 说明 |
|------|-------|------------|-----------------|------|
| `LFs_Link` | `LFs` | continuous | base_link -> LFs_Link | 左前转向节 |
| `LFd_Link` | `LFd` | continuous | LFs_Link -> LFd_Link | 左前车轮 |
| `LRs_Link` | `LRs` | continuous | base_link -> LRs_Link | 左后转向节 |
| `LRd_Link` | `LRd` | continuous | LRs_Link -> LRd_Link | 左后车轮 |
| `RFs_Link` | `RFs` | continuous | base_link -> RFs_Link | 右前转向节 |
| `RFd_Link` | `RFd` | continuous | RFs_Link -> RFd_Link | 右前车轮 |
| `RRs_Link` | `RRs` | continuous | base_link -> RRs_Link | 右后转向节 |
| `RRd_Link` | `RRd` | continuous | RRs_Link -> RRd_Link | 右后车轮 |

> **命名约定：** `L/R` = 左/右，`F/R` = 前/后，`s`（steering）= 转向节，`d`（drive）= 车轮驱动。

### 4.3 关键坐标系参考

| 坐标系 | 说明 | 用途 |
|--------|------|------|
| `map` | 世界固定坐标系（由 SLAM 提供） | 全局定位、导航目标 |
| `odom` | 里程计漂移坐标系（由 odometry 节点提供） | 局部运动估计 |
| `base_footprint` | 机器人在 `odom`/`map` 平面的投影 | 导航定位基准 |
| `base_link` | 底盘主体刚体坐标系 | 机器人中心参考 |
| `laser` | LiDAR 传感器坐标系 | LaserScan 数据 frame_id |

> **TF 树流向（导航场景）：** `map -> odom -> base_footprint -> base_link -> laser`（以及各轮、各臂段）

---

## 5. 参数说明

### 5.1 robot_state_publisher 关键参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `robot_description` | string | URDF 文件全文 | 从 `urdf/robot_description.urdf` 读取并传入 |
| `use_sim_time` | bool | false | 仿真时间。若下游使用 Gazebo，需设为 `true` |
| `publish_frequency` | double | 50.0 | TF 发布频率（Hz），默认值通常足够 |

### 5.2 joint_state_publisher_gui 参数

此节点在与真实硬件连接时应关闭（由 `robot_startup` 提供实际的 `/joint_states`），仅在离线调试时使用。

---

## 6. 运行方式

### 6.1 独立运行（离线可视化）

编译后，直接启动本包的 launch 文件即可在 RViz2 中查看机器人模型：

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_description robot_description.launch.py
```

启动后：
- RViz2 窗口将显示机器人的 3D 模型
- `joint_state_publisher_gui` 窗口可用于拖动关节滑块，验证 URDF 运动学
- TF 树从 `base_footprint` 展开至所有子 link

### 6.2 作为子组件运行

本包通常不作为独立节点运行，而是嵌入到以下上层包的 launch 文件中：

#### 方式 A：物理机器人 — `robot_startup`

`robot_startup` 包在启动时会：
1. 调用 `robot_description.launch.py` 启动 `robot_state_publisher`（不启动 rviz2 和 gui）
2. 从编码器读取各轮转向/驱动角度，发布 `/joint_states`
3. `robot_state_publisher` 根据 `/joint_states` 动态发布 TF

#### 方式 B：仿真 — `robot_gazebo`

`robot_gazebo` 包在启动时会：
1. 调用 `robot_description.launch.py` 加载 URDF 到 Gazebo
2. Gazebo 的 joint state plugin 自动发布 `/joint_states`
3. RViz2 通过 TF 同步显示仿真中的机器人位姿

---

## 7. 与其他包的协作

```
┌──────────────────────────────────────────────────────────┐
│                   robot_description                       │
│  ┌─────────────────────┐  ┌──────────────────────────┐  │
│  │ URDF 模型            │  │ robot_state_publisher    │  │
│  │ (link/joint/惯量)    │  │ (TF 发布)                │  │
│  └─────────┬───────────┘  └────────────┬─────────────┘  │
│            │                           │                 │
│            ▼                           ▼                 │
│  ┌─────────────────────┐  ┌──────────────────────────┐  │
│  │ meshes/ (STL)       │  │ tf2_msgs/TFMessage       │  │
│  │ rviz2/ (配置)       │  │ (-> /tf, /tf_static)     │  │
│  └─────────────────────┘  └──────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
          │                             │
          ▼                             ▼
┌──────────────────┐          ┌──────────────────────┐
│   robot_gazebo    │          │   robot_startup      │
│ (仿真模型加载)     │          │ (物理机器人 TF 发布)   │
│ URDF -> SDF 转换  │          │ 编码器 -> /joint_states│
└──────────────────┘          └──────────┬───────────┘
          │                              │
          ▼                              ▼
┌──────────────────────────────────────────────────────────┐
│                       at_nav2                             │
│  ┌─────────────────┐  ┌──────────────┐  ┌─────────────┐ │
│  │ costmap (local, │  │ planner      │  │ controller  │ │
│  │ global)         │  │ server       │  │ server      │ │
│  └─────────────────┘  └──────────────┘  └─────────────┘ │
│              robot_base_frame: base_link                   │
│              global_frame: map                             │
│              odom_frame: odom                              │
└──────────────────────────────────────────────────────────┘
```

**数据流说明：**
1. `robot_description` 提供静态 URDF，`robot_state_publisher` 发布 TF 变换
2. `robot_startup`（物理）或 `robot_gazebo`（仿真）提供 `/joint_states` 驱动动态 TF
3. `at_nav2` 消费 TF 树，以 `base_link` 为 robot_base_frame 进行导航

---

## 8. 故障排查

| 问题 | 可能原因 | 检查/修复命令 |
|------|----------|---------------|
| RViz 中看不到模型 | `RobotModel` display 未启用或 TF 未连接 | 在 RViz 中：Displays -> Add -> RobotModel，设置 Description Topic 为 `/robot_description` |
| 模型显示但 TF 不全（只有 base_link） | `/joint_states` 没有发布 | `ros2 topic echo /joint_states` 检查是否有数据；独立运行时应启动 `joint_state_publisher_gui` |
| `joint_state_publisher_gui` 窗口不弹出 | 相关包未安装或 display 环境未设置 | `sudo apt install ros-humble-joint-state-publisher-gui`；检查是否在带 X11 的终端中运行 |
| mesh 文件找不到（RViz 中报错） | 安装路径错误，URDF 中 `package://` 解析失败 | 确认 `colcon build` 后已 source：`source ~/ros2_ws/install/setup.bash`；检查 `package://robot_description/meshes/` 路径是否存在 |
| `laser` 坐标系无数据 | laser_joint 为 fixed，TF 自动发布 | `ros2 run tf2_tools view_frames` 查看完整 TF 树，确认 `base_link -> laser` 连接存在 |
| Gazebo 中机器人无碰撞 | URDF 碰撞模型需被 Gazebo 识别 | 检查 URDF 中每个 `<link>` 是否都包含 `<collision>` 标签（本包已包含） |
| TF 树断裂（部分 link 不显示） | 某个 joint 的 parent/child 引用错误 | `ros2 run tf2_tools view_frames` 生成 frames.pdf，逐级检查 parent->child 关系 |

---

## 9. 参考

- [ROS2 URDF 教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [robot_state_publisher 文档](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html)
- [Nav2 坐标框架约定](https://docs.nav2.org/configuration/packages/configuring-robot-description.html)
- [SW URDF Exporter](http://wiki.ros.org/sw_urdf_exporter)（本 URDF 由 SolidWorks 导出）
