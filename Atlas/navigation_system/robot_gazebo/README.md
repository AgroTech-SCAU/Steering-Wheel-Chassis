# robot_gazebo

> Gazebo 仿真环境启动包 — 负责加载世界、解析机器人 xacro 模型、启动 state publisher 并将机器人实体 spawn 到仿真中。
> 核心依赖：gazebo_ros, gazebo_plugins, xacro, robot_description, robot_state_publisher
> 下游消费者：robot_cartographer_mapping（建图）、at_nav2（导航仿真测试）

## 包概述

本包是 AGT 竞赛机器人仿真栈的**入口层**，职责单一：拉齐 Gazebo 运行时所需的所有资源并启动仿真。

一个 `ros2 launch` 命令即可完成以下步骤：

1. 启动 Gazebo 服务端（`gzserver`），加载 `competition.world` 世界文件
2. 可选启动 Gazebo 可视化客户端（`gzclient`，GUI）
3. 用 `xacro` 解析 `robot_sim.xacro`，通过 `robot_state_publisher` 发布 TF
4. 调用 `spawn_entity.py` 将机器人模型生成到 Gazebo 中

本包不包含机器人本体定义——URDF 模型由 `robot_description` 包提供，这里仅通过 xacro 引用其 mesh 与配置，并在 `<gazebo>` 标签中挂载底盘差速驱动、IMU、激光雷达等 Gazebo 插件。

## 包结构

```
robot_gazebo/
├── CMakeLists.txt                 # ament_cmake, 安装 urdf/launch/worlds 到 share
├── package.xml                    # 依赖声明（gazebo_ros, xacro, robot_description 等）
├── launch/
│   └── gazebo_sim.launch.py       # 主 launch 文件
├── urdf/
│   └── robot_sim.xacro            # 仿真用 xacro（含 Gazebo 插件配置，725 行）
└── worlds/
    └── competition.world          # AGT 竞赛场地世界文件（ODE 物理引擎）
```

## 依赖

本包需要以下 ROS2 及系统包：

```bash
# 系统依赖（Gazebo 11）
sudo apt install -y gazebo11 libgazebo11-dev

# ROS2 依赖
sudo apt install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs          # gazebo_ros, gazebo_plugins
sudo apt install -y ros-${ROS_DISTRO}-robot-state-publisher     # robot_state_publisher
sudo apt install -y ros-${ROS_DISTRO}-joint-state-publisher      # joint_state_publisher
sudo apt install -y ros-${ROS_DISTRO}-xacro                     # xacro

# 同级包（需在同一工作空间内编译）
# robot_description — 提供 URDF mesh 与 robot_description topic
```

> `robot_description`、`robot_cartographer_mapping`、`at_nav2` 等包均在同一 `Atlas/navigation_system/` 目录下的 monorepo 中，通过 colcon 统一编译即可。

## Launch 参数

| 参数名 | 默认值 | 说明 |
| --- | --- | --- |
| `use_sim_time` | `true` | 使用 Gazebo 仿真时钟 `/clock`，所有节点必须设为 `true` |
| `world_path` | `<pkg>/worlds/competition.world` | 世界文件路径，可切换自定义场地 |
| `x_pos` | `1.57` | 机器人 spawn X 坐标（米） |
| `y_pos` | `1.4` | 机器人 spawn Y 坐标（米） |
| `z_pos` | `0.1` | 机器人 spawn Z 坐标（米），抬高避免穿模 |
| `gui` | `true` | 是否启动 Gazebo 可视化客户端（headless 时设为 `false`） |

> 默认 spawn 位置 `(1.57, 1.4, 0.1)` 对应 `competition.world` 中预设的竞赛场地起始点。

## 运行方式

### 标准启动（带 GUI）

```bash
ros2 launch robot_gazebo gazebo_sim.launch.py
```

带自定义 spawn 位姿：

```bash
ros2 launch robot_gazebo gazebo_sim.launch.py \
  x_pos:=0.5 y_pos:=0.5 z_pos:=0.05
```

### Headless 模式（无 GUI，用于服务器 / CI）

```bash
ros2 launch robot_gazebo gazebo_sim.launch.py gui:=false
```

### 与建图联合使用

终端 A —— 启动仿真：

```bash
ros2 launch robot_gazebo gazebo_sim.launch.py
```

终端 B —— 启动 Cartographer 建图：

```bash
ros2 launch robot_cartographer_mapping cartographer_mapping.launch.py
```

> 同理，`at_nav2` 导航 launch 也应在仿真启动后在另一个终端运行。

## 启动流程

```
ros2 launch robot_gazebo gazebo_sim.launch.py
  │
  ├─ [1] gzserver.launch.py ── 加载 competition.world ──> /gazebo 节点 + 物理引擎
  │       │
  │       └─ [环境] GAZEBO_RESOURCE_PATH, GAZEBO_MODEL_PATH, GAZEBO_PLUGIN_PATH
  │
  ├─ [2] gzclient 进程 ────────────────> Gazebo GUI 窗口（gui:=false 时跳过）
  │       │
  │       └─ [环境] GDK_BACKEND=x11, __GL_SYNC_TO_VBLANK=0
  │
  ├─ [3] robot_state_publisher ───────> 解析 xacro → /robot_description + TF 树
  │       │                              -----------------
  │       │                              订阅 /joint_states
  │       │                              发布 /tf
  │
  └─ [4] spawn_entity.py ────────────> 从 /robot_description topic 读取 URDF
                                         并 spawn 到 Gazebo 中 (x_pos, y_pos, z_pos)
```

## NVIDIA + Wayland 兼容性

本 launch 文件内置了两项针对 **NVIDIA 显卡 + Wayland 显示服务器** 场景的修复：

| 环境变量 | 值 | 原因 |
| --- | --- | --- |
| `GDK_BACKEND` | `x11` | 强制 GTK/GDK 走 XWayland，绕过 Wayland EGL 与 NVIDIA 驱动的 GLX 冲突 |
| `__GL_SYNC_TO_VBLANK` | `0` | 禁用垂直同步，避免 OGRE 渲染引擎（Gazebo GUI）在 NVIDIA 闭源驱动下触发 `boost::shared_ptr` 断言崩溃 |

> 如果使用 Intel / AMD 显卡且 Wayland 原生运行，可移除这两个变量。它们在非 NVIDIA 环境下通常不产生副作用。

同时，launch 文件在调用 `gzserver.launch.py` 之前显式设置了以下 Gazebo 资源路径，确保即使在非标准安装环境下也能找到系统模型（sun、ground_plane）、shader 和插件：

- `GAZEBO_RESOURCE_PATH` → `/usr/share/gazebo-11`
- `GAZEBO_MODEL_PATH` → `/usr/share/gazebo-11/models`
- `GAZEBO_PLUGIN_PATH` → `/usr/lib/<arch>-linux-gnu/gazebo-11/plugins`

## 协作关系

```
/gazebo
  │
  ├── /clock ───────────────────────> 所有 use_sim_time:=true 的节点（含 robot_state_publisher）
  │
  ├── /joint_states ───────────────> robot_state_publisher → /tf
  │
  └── /robot_description ──────────> spawn_entity.py（一次性读取，spawn 后不再使用）
```

- **`/clock`**：Gazebo 仿真时间，所有感知、规划、控制节点必须 set `use_sim_time:=true`
- **`/joint_states`**：Gazebo 插件发布各关节状态，`robot_state_publisher` 订阅后计算并广播 TF
- **`/robot_description`**：URDF 字符串 topic，`spawn_entity.py` 从中读取模型并生成 SDF 实体

## 故障排查

| 现象 | 可能原因 | 解决 |
| --- | --- | --- |
| Gazebo 闪退，日志含 `GLX` 或 `EGL` 错误 | NVIDIA 驱动 + Wayland 不兼容 | 确认 `GDK_BACKEND=x11` 已设置；或切换到 X11 会话 |
| 机器人未出现在仿真中 | `spawn_entity.py` 找不到 `/robot_description` topic | 检查 `robot_state_publisher` 是否正常启动；`ros2 topic echo /robot_description` 确认有数据 |
| 世界加载失败，提示找不到 `sun` / `ground_plane` | `GAZEBO_MODEL_PATH` 未正确设置 | 确认 `/usr/share/gazebo-11/models` 存在；检查 launch 中的 `os.environ.setdefault` 调用 |
| xacro 解析报错 "package not found" | 工作空间未 source 或 `robot_description` 未编译 | `source install/setup.bash` 后重试；`colcon build --packages-up-to robot_gazebo` |
| GUI 启动但窗口黑屏 / 渲染异常 | NVIDIA OGRE 渲染器 vsync 问题 | 确认 `__GL_SYNC_TO_VBLANK=0` 已设置 |
| `gzclient` 进程启动失败，提示找不到 `gzclient` | Gazebo 未安装或未加入 PATH | `which gzclient`；`sudo apt install gazebo11` |
| `use_sim_time` 相关错误 | 下游节点未设置 `use_sim_time:=true` | 在各自的 launch 或参数文件中确保 `use_sim_time` 为 `true` |

> 调试建议：出现问题时，先用 `gui:=false` 启动 headless 模式排除 GUI 相关干扰，再单独排查 Gazebo 服务端问题。
>
> ```bash
> ros2 launch robot_gazebo gazebo_sim.launch.py gui:=false
> ```
