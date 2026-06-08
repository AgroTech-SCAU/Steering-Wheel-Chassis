# navigation_system — AGT 比赛轮式机器人导航系统

> **基于 ROS2 Humble、Nav2 和 Cartographer 的全栈导航系统**，覆盖从传感器驱动、SLAM 建图、路径规划到底层控制的完整机器人导航栈。

本系统面向 AGT 比赛场景，采用 **ROS2 Humble** 作为中间件框架，**Cartographer 2D 纯定位** 提供全局位姿估计，**Nav2（Navfn + DWB）** 负责全局规划与局部运动控制，**LSLIDAR N10P** 单线激光雷达为感知输入。仿真环境基于 **Gazebo 11** 构建，支持建图-导航全流程验证。

---

## 1. 系统架构

```mermaid
graph TD
    subgraph 启动层
        startup[robot_startup<br/>总启动入口]
    end

    subgraph 感知层
        msgs[lslidar_msgs<br/>消息/服务接口定义] -.->|rosidl 编译依赖| driver
        driver[lslidar_driver<br/>N10P UART 驱动] -->|sensor_msgs/LaserScan| scan["/scan"]
    end

    subgraph 仿真层
        gz[robot_gazebo<br/>Gazebo 仿真环境] -->|加载 URDF| desc[robot_description<br/>URDF 模型 + TF]
        desc -->|robot_state_publisher<br/>/tf + /tf_static| tf_tree[TF 变换树]
    end

    subgraph 建图层
        mapping[robot_cartographer_mapping<br/>Cartographer 2D SLAM] -->|/map| map_topic
        mapping -->|生成| pbstream[pbstream 文件]
        mapping -->|导出| pgm_yaml[PGM + YAML 地图]
    end

    subgraph 导航层
        nav2[at_nav2<br/>Nav2 导航栈] -->|Cartographer 纯定位| loc[全局位姿估计]
        nav2 -->|Navfn 全局规划| gp[全局路径]
        nav2 -->|DWB 局部控制| lc[局部速度指令]
        nav2 -->|geometry_msgs/Twist| cmd_vel["/cmd_vel"]
        pbstream -.->|纯定位初始状态| loc
        pgm_yaml -.->|全局代价地图| nav2
    end

    subgraph 控制层
        fsm[competition_fsm<br/>竞赛状态机]
        mission[mission_manager<br/>任务管理器]
        fsm -->|决策指令| mission
        cmd_vel -->|速度指令| fsm
        fsm -->|geometry_msgs/Twist| motor_cmd_vel["/motor_cmd_vel"]
        motor_cmd_vel -->|串口/CAN| chassis[全向轮式底盘]
    end

    startup -->|Lifecycle 启动| driver
    startup -->|Lifecycle 启动| desc
    startup -->|Lifecycle 启动| nav2
    startup -->|Lifecycle 启动| fsm
    startup -->|Lifecycle 启动| mission

    gz -->|/joint_states + /clock| desc
    gz -->|仿真传感器数据| mapping
    gz -->|仿真传感器数据| nav2

    scan --> mapping
    scan --> nav2
```

> 虚线表示编译时依赖或非实时数据流，实线表示运行时话题/服务通信。

---

## 2. 功能包清单

本 monorepo 包含 7 个功能包，按系统层次排列：

| 包名 | 定位 | 核心依赖 | 类型 |
|------|------|----------|------|
| [`lslidar_msgs`](./lslidar_msgs/) | 雷达消息与服务接口定义 | `builtin_interfaces`, `rosidl_default_generators` | 纯接口包 |
| [`lslidar_driver`](./lslidar_driver/) | LSLIDAR N10P UART 驱动节点 | `lslidar_msgs`, `rclcpp`, `sensor_msgs`, `pcl_conversions`, `libpcap` | 驱动包 |
| [`robot_description`](./robot_description/) | URDF 模型 + robot_state_publisher | `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2` | 模型包 |
| [`robot_gazebo`](./robot_gazebo/) | Gazebo 11 仿真环境启动 | `gazebo_ros`, `gazebo_plugins`, `xacro`, `robot_description` | 仿真包 |
| [`robot_cartographer_mapping`](./robot_cartographer_mapping/) | Cartographer 2D SLAM 建图 | `cartographer_ros`, `nav2_map_server` | 建图包 |
| [`at_nav2`](./at_nav2/) | Nav2 导航栈 + Cartographer 纯定位 | `nav2_bringup`, `nav2_planner`, `nav2_controller`, `cartographer_ros` | 导航包 |
| [`robot_startup`](./robot_startup/) | 总启动入口，组合所有节点 | `robot_state_publisher`, `at_nav2`, `mission_manager`, `competition_fsm`, `lslidar_driver`, `robot_description` | 启动包 |

> `competition_fsm` 和 `mission_manager` 为控制层外部依赖包，不在本目录中。

---

## 3. 快速开始

### 3.1 环境准备

在 Ubuntu 22.04 上安装必需的系统依赖和 ROS2 包：

```bash
# 系统依赖
sudo apt install -y build-essential cmake git python3-colcon-common-extensions \
  python3-rosdep gazebo11 libgazebo11-dev libpcl-dev libpcap-dev \
  libyaml-cpp-dev libboost-all-dev

# ROS2 Humble 核心包
sudo apt install -y ros-humble-desktop ros-humble-rosidl-default-generators

# 导航栈
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-map-server ros-humble-nav2-planner ros-humble-nav2-controller

# Cartographer
sudo apt install -y ros-humble-cartographer ros-humble-cartographer-ros

# Gazebo 集成
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-xacro

# TF 与可视化
sudo apt install -y ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-rviz2

# 依赖初始化
sudo rosdep init && rosdep update
```

### 3.2 克隆与构建

```bash
# 创建工作空间
mkdir -p ~/AT_Atlas_nav_ws/src
cd ~/AT_Atlas_nav_ws

# 克隆本仓库
git clone <repo-url> src/Steering-Wheel-Chassis

# 如果导航包在子目录中，创建符号链接
# （如果 Atlas/navigation_system 已在 src 下，可跳过此步）

# 安装所有依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建全部包
colcon build --symlink-install
source install/setup.bash
```

### 3.3 仿真建图

**终端 A** —— 启动 Gazebo 仿真环境：

```bash
source ~/AT_Atlas_nav_ws/install/setup.bash
ros2 launch robot_gazebo gazebo_sim.launch.py
```

**终端 B** —— 启动 Cartographer 建图：

```bash
source ~/AT_Atlas_nav_ws/install/setup.bash
ros2 launch robot_cartographer_mapping robot_cartographer_mapping_gazebo.launch.py
```

用键盘/手柄遥控机器人在场地中移动，覆盖目标区域后保存地图：

```bash
# 保存 pbstream（用于后续纯定位）
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$(pwd)/src/robot_cartographer_mapping/map/my_map.pbstream'}"

# 导出 PGM + YAML（用于 Nav2 全局代价地图）
cd src/robot_cartographer_mapping/map
ros2 run nav2_map_server map_saver_cli -t map -f my_map
```

### 3.4 仿真导航

将建图产物（`my_map.pbstream`、`my_map.pgm`、`my_map.yaml`）放入 `at_nav2/maps/` 目录，并在 launch 文件中指向这些文件后：

**终端 A** —— 启动仿真：

```bash
source ~/AT_Atlas_nav_ws/install/setup.bash
ros2 launch robot_gazebo gazebo_sim.launch.py
```

**终端 B** —— 启动导航：

```bash
source ~/AT_Atlas_nav_ws/install/setup.bash
ros2 launch at_nav2 at_nav_gazebo.launch.py
```

RViz2 会自动打开，在工具栏点击 "2D Pose Estimate" 设置初始位姿，再点击 "Nav2 Goal" 指定导航目标。

### 3.5 真机部署

真机上通过 `robot_startup` 一键启动全栈：

```bash
source ~/AT_Atlas_nav_ws/install/setup.bash
ros2 launch robot_startup robot_start.launch.py
```

该 launch 文件会依次拉取：
1. `lslidar_driver` —— N10P 雷达驱动
2. `robot_description` —— URDF + TF 发布
3. `at_nav2` —— Cartographer 纯定位 + Nav2 导航
4. `competition_fsm` —— 竞赛状态机
5. `mission_manager` —— 任务管理器

---

## 4. 核心数据流

| Topic | 发布者 | 订阅者 | 说明 |
|-------|--------|--------|------|
| `/scan` | `lslidar_driver` | `cartographer_node`, `nav2_costmap_2d` | N10P 单线激光扫描数据（sensor_msgs/LaserScan） |
| `/odom` | 底盘里程计 / Gazebo plugin | `cartographer_node`, `nav2_controller` | 轮式里程计，提供局部运动估计（nav_msgs/Odometry） |
| `/map` | `cartographer_node` | `nav2_costmap_2d`（global costmap） | SLAM 生成的占据栅格地图（nav_msgs/OccupancyGrid） |
| `/cmd_vel` | `nav2_controller`（DWB） | `competition_fsm` | Nav2 输出的速度指令（geometry_msgs/Twist） |
| `/motor_cmd_vel` | `competition_fsm` | 底盘驱动节点 | 经状态机仲裁后的底盘速度指令（geometry_msgs/Twist） |
| `/joint_states` | 底盘编码器 / Gazebo plugin | `robot_state_publisher` | 各关节状态（sensor_msgs/JointState），驱动 TF 动态发布 |
| `map -> odom` TF | `cartographer_node` | `robot_state_publisher` / `nav2_controller` | 全局定位修正变换 |
| `odom -> base_footprint` TF | 里程计节点 | `robot_state_publisher` / `nav2_costmap_2d` | 局部运动估计变换 |
| `base_link -> laser_link` TF | `robot_state_publisher`（URDF static） | `cartographer_node`, `nav2_costmap_2d` | LiDAR 传感器安装位姿（fixed joint） |

> TF 完整链路：`map -> odom -> base_footprint -> base_link -> laser_link`（及各轮、各臂段）

---

## 5. 硬件/软件要求

| 类别 | 项目 | 要求 |
|------|------|------|
| **操作系统** | Ubuntu | 22.04 LTS（Jammy） |
| **ROS2** | Humble Hawksbill | 桌面完整版（desktop） |
| **仿真** | Gazebo | 11（Classic） |
| **编译器** | GCC / Clang | C++17 标准 |
| **脚本语言** | Python | 3.10+（含 typing） |
| **雷达** | LSLIDAR N10P | 单线激光雷达，UART 串口通信 |
| **底盘** | 全向轮式底盘 | 四轮独立转向 + 独立驱动 |
| **构建系统** | colcon | 需 `--symlink-install` 支持 |

---

## 6. 完整目录树

```
navigation_system/
├── README.md                                 # 本文档（总入口）
│
├── lslidar_msgs/                             # 雷达消息/服务接口包
│   ├── msg/                                  #   自定义消息（LslidarInformation, LslidarPacket）
│   ├── srv/                                  #   自定义服务（IpAndPort, MotorControl, TimeMode 等 11 项）
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── README.md
│
├── lslidar_driver/                           # 雷达驱动包（N10P UART）
│   ├── config/                               #   雷达参数 YAML 配置
│   ├── include/lslidar_driver/               #   头文件（点云类型定义、驱动核心）
│   ├── launch/                               #   启动文件
│   ├── rviz/                                 #   RViz2 调试配置
│   ├── src/                                  #   驱动源码
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── README.md
│
├── robot_description/                        # URDF 模型与 TF
│   ├── urdf/                                 #   robot_description.urdf（~874 行）
│   ├── meshes/                               #   STL 网格（底盘、车轮、机械臂、LiDAR 支架）
│   ├── launch/                               #   robot_description.launch.py
│   ├── rviz2/                                #   模型可视化配置
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── README.md
│
├── robot_gazebo/                             # Gazebo 仿真环境
│   ├── launch/                               #   gazebo_sim.launch.py（主启动文件）
│   ├── urdf/                                 #   robot_sim.xacro（含 Gazebo 插件，725 行）
│   ├── worlds/                               #   competition.world（竞赛场地）
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── README.md
│
├── robot_cartographer_mapping/               # Cartographer 2D SLAM 建图
│   ├── config/                               #   robot_2d_gazebo.lua（SLAM 参数）
│   ├── launch/                               #   robot_cartographer_mapping_gazebo.launch.py
│   ├── map/                                  #   默认地图文件（pbstream, pgm, yaml）
│   ├── rviz/                                 #   建图可视化配置
│   ├── src/                                  #   C++ 源码（预留扩展）
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── README.md
│
├── at_nav2/                                  # Nav2 导航栈 + Cartographer 纯定位
│   ├── config/                               #   Nav2 参数 + BT XML + Cartographer 定位 LUA
│   ├── launch/                               #   导航启动文件（Gazebo / 真机）
│   ├── maps/                                 #   导航用地图（pbstream, pgm, yaml）
│   ├── rviz2/                                #   导航可视化配置
│   ├── CMakeLists.txt
│   └── package.xml
│
└── robot_startup/                            # 总启动入口
    ├── launch/                               #   robot_start.launch.py / robot_start_gazebo.launch.py
    ├── config/                               #   启动配置
    ├── include/robot_startup/                #   头文件
    ├── src/                                  #   启动辅助源码
    ├── CMakeLists.txt
    └── package.xml
```

---

## 7. 维护者

**AGT Dev Team** — 竞赛机器人导航系统开发与维护。

如有问题或建议，请提交 Issue 或通过仓库联系。

---

## 8. 相关链接

| 资源 | 链接 |
|------|------|
| Navigation2 文档 | https://docs.nav2.org |
| Nav2 GitHub | https://github.com/ros-navigation/navigation2 |
| Cartographer ROS 文档 | https://google-cartographer-ros.readthedocs.io |
| Cartographer GitHub | https://github.com/cartographer-project/cartographer_ros |
| LSLIDAR 官方 | https://www.lslidar.com |
| ROS2 Humble 文档 | https://docs.ros.org/en/humble |
| Gazebo 11 文档 | https://classic.gazebosim.org/tutorials |
| ROS2 TF2 教程 | https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2 |
