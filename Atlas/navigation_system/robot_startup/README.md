# robot_startup

> **定位：系统总启动入口**，一键启动所有导航子节点。
> 核心依赖：`robot_description`、`lslidar_driver`、`at_nav2`、`mission_manager`、`competition_fsm`。
> 上游：无（本包为顶层编排器，不被任何其他包依赖）。

## 1. 包概述

`robot_startup` 是 AGT 竞赛机器人导航系统的**顶层启动编排器**。它本身不包含任何算法或业务逻辑，只负责按正确的顺序和时序将各个子系统组装起来，提供开箱即用的"一键启动"能力。

与之配合的还有一个 Gazebo 仿真启动文件（`robot_start_gazebo.launch.py`），用于在仿真环境中调试导航栈，启动逻辑与实车启动同理。

## 2. 包结构

```
robot_startup/
├── CMakeLists.txt                          # 构建配置（安装 launch 与 config 目录）
├── package.xml                             # 包元数据
├── launch/
│   ├── robot_start.launch.py               # 实车一键启动
│   └── robot_start_gazebo.launch.py        # Gazebo 仿真一键启动
└── config/                                 # （预留）参数配置文件目录
```

## 3. 启动编排与顺序

`robot_start.launch.py` 中节点的启动遵循严格的顺序设计。核心原则是：**保证数据生产者先于消费者就绪**。

```
robot_start.launch.py
├── [即时] robot_state_publisher     # 发布 /tf_static，为 TF 树提供 URDF 关节信息
├── [即时] lslidar_driver            # 激光雷达驱动 → /scan
├── [即时] mission_manager           # 任务管理节点（接收上层指令，调度导航目标）
├── [即时] competition_fsm           # FSM 仲裁节点（接管 /cmd_vel，输出 /motor_cmd_vel）
└── [延迟 3s] at_nav2                # Cartographer 纯定位 + Nav2 导航栈
```

### 3.1 为什么 at_nav2 需要延迟 3 秒？

`at_nav2` 内部同时启动了 Cartographer（纯定位模式）和 Nav2（behavior tree / controller / planner）。两者在初始化阶段都需要以下话题已经存在：

- `/scan` —— 由 `lslidar_driver` 提供，雷达上电并开始发布数据通常需要 1~2 秒
- `/odom` —— 由底盘驱动提供（不在本 launch 中启动，需预先运行）
- `/tf` / `/tf_static` —— 由 `robot_state_publisher` 提供

如果 `at_nav2` 启动过早，Cartographer 会因 "无 scan 数据" 或 "缺少 tf 变换" 而报错退出。3 秒的延迟给硬件驱动和基础 TF 足够的时间完成初始化，确保导航栈在一个稳定的数据流上启动。

### 3.2 TimerAction 机制

延迟通过 ROS2 Launch 的 `TimerAction` 实现：

```python
from launch.actions import TimerAction

ld.add_action(TimerAction(period=3.0, actions=[at_nav_launch]))
```

`period=3.0` 表示从 LaunchDescription 开始执行起，延迟 3.0 秒后才触发 `at_nav_launch`。如果实际硬件初始化更慢（例如雷达上电 > 3 秒），可以酌情调大该值。

### 3.3 Gazebo 仿真启动

`robot_start_gazebo.launch.py` 的启动编排有所不同：

```
robot_start_gazebo.launch.py
├── [即时] robot_state_publisher     # TF
├── [即时] robot_gazebo              # Gazebo 仿真环境（含底盘 + 雷达仿真）
└── [延迟 5s] at_nav2                # at_nav_gazebo.launch.py（Cartographer + Nav2）
```

仿真环境中 Gazebo 加载模型和传感器插件需要更长时间，因此延迟设为 5 秒。与实车启动不同，仿真文件不启动 `mission_manager` 和 `competition_fsm`，而是依赖仿真环境内嵌的控制器。

## 4. FSM 仲裁说明

`competition_fsm` 在 cmd_vel 指令到达底盘之前插入了一层仲裁逻辑，形成如下控制链路：

```
controller_server (Nav2)
       │
       ▼ /cmd_vel (geometry_msgs/Twist)
competition_fsm
       │
       ▼ /motor_cmd_vel (geometry_msgs/Twist)
底盘驱动 (motor control)
```

**关键点：底盘驱动订阅的是 `/motor_cmd_vel`，而非 `/cmd_vel`。** 这意味着：

- 直接向 `/cmd_vel` 发送速度指令**不会**驱动机器人移动
- Nav2 的 `controller_server` 输出 `/cmd_vel`，由 `competition_fsm` 根据当前 FSM 状态决定是否放行、修改或拦截
- 如果 FSM 未启动或异常退出，底盘将无法接收任何速度指令

调试时可以用以下命令验证两路话题：

```bash
ros2 topic echo /cmd_vel          # Nav2 输出
ros2 topic echo /motor_cmd_vel    # FSM 仲裁后输出（底盘实际执行）
```

## 5. 运行方式

### 5.1 一键启动（实车）

确保底盘驱动已运行（提供 `/odom`），然后：

```bash
ros2 launch robot_startup robot_start.launch.py
```

### 5.2 一键启动（Gazebo 仿真）

```bash
ros2 launch robot_startup robot_start_gazebo.launch.py
```

### 5.3 分步调试

当需要定位问题或单独调试某个子系统时，可以逐个启动：

```bash
# 1. TF（最先启动）
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat <urdf_path>)"

# 2. 激光雷达
ros2 launch lslidar_driver lsn10p_launch.py

# 3. 任务管理
ros2 run mission_manager mission_manager

# 4. FSM
ros2 run competition_fsm competition_fsm_node

# 5. 导航栈（等 /scan 和 /odom 稳定后）
ros2 launch at_nav2 at_nav.launch.py
```

## 6. 数据流全景

```
                            ┌──────────────────┐
                            │ robot_description │
                            │   (URDF 模型)     │
                            └────────┬─────────┘
                                     │ /robot_description
                                     ▼
                        ┌────────────────────────┐
                        │  robot_state_publisher  │
                        │       /tf_static        │
                        └────────────────────────┘

  ┌────────────────┐
  │ lslidar_driver │──── /scan ────┐
  └────────────────┘               │
                                   ▼
  ┌────────────────┐     ┌─────────────────────┐
  │  底盘驱动(外部) │──── │      at_nav2         │
  │     /odom      │     │  Cartographer + Nav2 │
  └────────────────┘     └──────────┬──────────┘
                                    │ /cmd_vel
                                    ▼
                          ┌────────────────┐
                          │ competition_fsm │
                          └───────┬────────┘
                                  │ /motor_cmd_vel
                                  ▼
                          ┌────────────────┐
                          │    底盘驱动     │
                          └────────────────┘

  ┌────────────────┐
  │ mission_manager │ ──── 调度导航目标 / 任务状态
  └────────────────┘
```

## 7. 协作关系图

```
robot_startup (顶层编排)
    │
    ├── robot_state_publisher    [TF 基础]
    ├── lslidar_driver           [传感器层]
    ├── mission_manager          [任务编排层]
    ├── competition_fsm          [安全仲裁层]
    └── at_nav2                  [导航核心层]
            ├── Cartographer     [定位]
            └── Nav2             [规划 + 控制]
```

五层划分：

| 层次 | 节点 | 职责 |
|------|------|------|
| TF 基础 | `robot_state_publisher` | 发布静态 TF，维护 URDF 关节树 |
| 传感器层 | `lslidar_driver` | 激光雷达数据采集，发布 `/scan` |
| 导航核心层 | `at_nav2` | Cartographer 纯定位 + Nav2 规划与控制 |
| 任务编排层 | `mission_manager` | 接收任务指令，调度导航目标 |
| 安全仲裁层 | `competition_fsm` | FSM 状态机，接管 cmd_vel 保障安全 |

## 8. 故障排查

### 8.1 某个节点未能启动

```bash
# 查看 launch 过程中各节点的状态
ros2 node list

# 查看具体节点的日志
ros2 node info /<node_name>

# 检查依赖包是否已安装
ros2 pkg list | grep -E "robot_startup|lslidar|at_nav2|mission_manager|competition_fsm"
```

### 8.2 导航栈启动即崩溃

最常见的原因：`/scan` 或 `/odom` 在 `at_nav2` 启动时尚未就绪。

```bash
# 检查话题是否存在
ros2 topic list | grep -E "scan|odom"

# 检查话题发布频率
ros2 topic hz /scan
ros2 topic hz /odom
```

如果 `/scan` 需要超过 3 秒才能稳定，在 `robot_start.launch.py` 中将 `TimerAction(period=3.0, ...)` 调大到 5.0 或更多。

### 8.3 机器人不移动但 Nav2 输出 cmd_vel 正常

```bash
# 确认 FSM 节点是否存活
ros2 node list | grep competition_fsm

# 对比两路 cmd_vel
ros2 topic echo /cmd_vel          # Nav2 输出
ros2 topic echo /motor_cmd_vel    # FSM 输出
```

若 `/cmd_vel` 有数据而 `/motor_cmd_vel` 无数据，检查 `competition_fsm` 的状态机当前是否在允许运动的模式下，或查看其日志输出。

### 8.4 FSM 未安装

```bash
# 检查是否已编译安装
ros2 pkg executables competition_fsm

# 若未找到，进入工作空间重新编译
cd ~/ros2_ws
colcon build --packages-select competition_fsm --symlink-install
source install/setup.bash
```

### 8.5 Gazebo 仿真中机器人无雷达数据

```bash
# 检查 Gazebo 是否正确加载了雷达插件
ros2 topic info /scan

# 检查 Gazebo 进程
ps aux | grep gzserver
```

仿真环境建议先单独启动 `robot_gazebo` 包确认传感器和底盘模型工作正常，再通过 `robot_startup` 一键启动全栈。
