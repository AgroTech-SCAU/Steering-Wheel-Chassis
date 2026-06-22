# at_nav2 - 导航系统核心配置包

> **定位：** Nav2 导航栈的一站式配置 / 启动 / 地图管理包；核心功能三件套：Cartographer 纯定位 + Nav2 导航栈 + 地图管理。
> **核心依赖：** `nav2_bringup`, `nav2_planner`, `nav2_controller`, `nav2_costmap_2d`, `cartographer_ros`, `rviz2`

---

## 1. 包概述

`at_nav2` 是本导航子系统的核心包，负责：

- 通过 Cartographer ROS 节点加载预建 pbstream 地图，提供纯定位（map -> odom TF）
- 加载静态占据栅格地图（pgm + yaml），供 global_costmap 使用
- 启动完整的 Nav2 导航栈（behavior tree、planner、controller、smoother、costmap、waypoint follower、velocity smoother）
- 提供真机与仿真两套 launch 文件，参数自动切换
- 提供一套经过调优的 Nav2 参数（全向底盘、DWB 本地规划器、Navfn 全局规划器）

---

## 2. 架构选择：Cartographer 纯定位 vs AMCL

| 维度 | Cartographer 纯定位 | AMCL |
|------|---------------------|------|
| 匹配方式 | 子图匹配（scan-to-submap） | 粒子滤波（Monte Carlo） |
| 精度 | 高，可达到厘米级 | 受限于粒子数量和随机采样 |
| 对初始位姿的依赖 | 低，全局搜索 | 高，需在初始位姿附近 |
| 鲁棒性（环境变化） | 较强，可适应部分动态场景 | 一般，依赖静态地图假设 |
| 计算开销 | 中等（需维护子图） | 低 |
| 是否需要 pbstream | 是（预先建图） | 否（只需 pgm+yaml） |
| 支持纯定位模式 | 是（`pure_localization_trimmer`） | 原生支持 |

### 本项目选择 Cartographer 的原因

1. **精度要求**：比赛/演示场景对定位精度要求高，Cartographer 子图匹配在结构化环境中优于 AMCL
2. **初始位姿容忍度**：启动时无需精确给定初始位姿，Cartographer 可在全局范围内匹配
3. **环境一致性**：比赛地图固定，Cartographer 的子图匹配优势明显

### 对 Nav2 启动架构的影响

由于选择 Cartographer 而非 AMCL，本包**不启动 Nav2 自带的 AMCL 节点**；具体表现为：

- **不使用** `nav2_bringup/launch/localization_launch.py`（该文件启动 AMCL + map_server）
- 改为**独立启动** `map_server` 节点 + `lifecycle_manager_map`，保证静态地图加载
- Cartographer 节点独立启动，通过 pbstream 提供 `map -> odom` TF 变换
- `navigation_launch.py` 只启动导航栈（planner、controller、behavior tree、smoother），不负责定位

---

## 3. 包结构

```
at_nav2/
├── CMakeLists.txt                        # 构建配置，安装 launch/config/maps/rviz2 到 share
├── package.xml                           # 包依赖声明
├── README.md                             # 本文档
├── config/
│   ├── at_nav2_params.yaml               # Nav2 全参数配置（核心文件，~290 行）
│   ├── bt_navigator.xml                  # Behavior Tree XML（简化版：ComputePathToPose → FollowPath）
│   ├── cartographer_localization.lua     # 真机 Cartographer 纯定位配置
│   └── cartographer_localization_gazebo.lua  # 仿真 Cartographer 纯定位配置
├── launch/
│   ├── at_nav.launch.py                  # 真机启动文件
│   └── at_nav_gazebo.launch.py           # 仿真启动文件
├── maps/
│   ├── map.yaml                          # 真机地图描述（分辨率 0.02m，含竞赛区域/禁行区定义）
│   ├── map.pgm                           # 真机占据栅格地图
│   ├── gazebo_map.yaml                   # 仿真地图描述（分辨率 0.05m）
│   ├── gazebo_map.pgm                    # 仿真占据栅格地图
│   └── gazebo_map.pbstream               # 仿真 Cartographer pbstream 地图
└── rviz2/
    └── nav2_gazebo.rviz                  # RViz2 可视化配置
```

---

## 4. 启动流程图

```
at_nav.launch.py (真机) / at_nav_gazebo.launch.py (仿真)
├── cartographer_node          # Cartographer 纯定位（加载 .pbstream 地图）
│   ├── 发布 map -> odom TF    # 核心：提供全局定位
│   └── 订阅 /scan, /odom
├── map_server                 # 静态地图服务器
│   └── 加载 .pgm + .yaml → 发布 /map topic
├── lifecycle_manager_map      # map_server 生命周期管理器（autostart=True）
├── navigation_launch.py       # Nav2 导航栈（nav2_bringup 提供）
│   ├── bt_navigator           # 行为树导航器
│   ├── planner_server         # 全局规划器（Navfn）
│   ├── controller_server      # 本地控制器（DWB）
│   ├── smoother_server        # 路径平滑器
│   ├── behavior_server        # 恢复行为（Spin/BackUp/DriveOnHeading/Wait）
│   ├── velocity_smoother      # 速度平滑器
│   ├── waypoint_follower      # 航点跟随器
│   ├── global_costmap         # 全局代价地图
│   └── local_costmap          # 局部代价地图
└── rviz2                      # RViz2 可视化（预加载 .rviz 配置）
```

### 关键架构说明

- **不启动 AMCL**：Cartographer 负责 `map -> odom` 的 TF 发布，AMCL 不启动以避免冲突
- **控制器输出路径**：`controller_server` 默认发布 `/cmd_vel`，下游 FSM（`competition_fsm`）仲裁后转发至 `/motor_cmd_vel`，再由底盘驱动执行

---

## 5. Nav2 参数详解

所有参数集中在 `config/at_nav2_params.yaml`；以下按子系统分类说明。

### 5.1 bt_navigator — 行为树导航器

| 参数 | 值 | 说明 |
|------|-----|------|
| `default_bt_xml_filename` | `bt_navigator.xml` | 行为树 XML 文件路径（相对 config 目录） |
| `global_frame` | `map` | 全局参考坐标系 |
| `robot_base_frame` | `base_link` | 机器人基底坐标系 |
| `odom_topic` | `/odom` | 里程计话题 |
| `bt_loop_duration` | 10 ms | 行为树循环周期 |
| `default_server_timeout` | 20 ms | 默认 Action 服务超时 |
| `wait_for_service_timeout` | 100 ms | 启动时等待服务就绪的超时 |

行为树 XML 内容（`bt_navigator.xml`）为最简流程：

```xml
<Sequence name="root">
    <ComputePathToPose goal="${goal}" path="${path}" planner_id="GridBased"/>
    <FollowPath path="${path}" controller_id="FollowPath"/>
</Sequence>
```

### 5.2 controller_server (DWB) — 本地控制器

DWB (Dynamic Window Approach) 是 Nav2 的本地规划器，负责生成满足运动学约束的短距离速度指令。

**基础参数：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `controller_frequency` | 10.0 Hz | 控制循环频率，取决于底盘响应能力 |
| `failure_tolerance` | 3.0 s | 允许控制失败的最大容忍时间 |
| `transform_tolerance` | 0.2 s | TF 变换容忍时间 |
| `sim_time` | 1.7 s | 轨迹仿真时间窗口，越大评估越远但计算量越大 |
| `linear_granularity` | 0.05 m | 线性轨迹插值间隔 |
| `angular_granularity` | 0.025 rad | 角度轨迹插值间隔 |
| `stateful` | True | 保持内部状态，减少指令振荡 |

**速度约束：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `min_vel_x` | -1.5 m/s | 最小前进速度（负值允许后退） |
| `max_vel_x` | 1.5 m/s | 最大前进速度 |
| `min_vel_y` | -1.5 m/s | 最小侧向速度（全向底盘） |
| `max_vel_y` | 1.5 m/s | 最大侧向速度（全向底盘） |
| `max_vel_theta` | 1.0 rad/s | 最大角速度 |
| `min_speed_xy` | 0.0 m/s | 最小平面合成速度 |
| `max_speed_xy` | 2.0 m/s | 最大平面合成速度 |
| `min_speed_theta` | 0.0 rad/s | 最小角速度 |

**加速度约束：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `acc_lim_x` | 2.5 m/s^2 | x 方向加速度限制 |
| `acc_lim_y` | 2.5 m/s^2 | y 方向加速度限制（全向底盘） |
| `acc_lim_theta` | 3.2 rad/s^2 | 角加速度限制 |
| `decel_lim_x` | -2.5 m/s^2 | x 方向减速度限制 |
| `decel_lim_y` | -2.5 m/s^2 | y 方向减速度限制 |
| `decel_lim_theta` | -3.2 rad/s^2 | 角减速度限制 |

**采样参数：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `vx_samples` | 20 | x 速度采样点数 |
| `vy_samples` | 20 | y 速度采样点数 |
| `vtheta_samples` | 20 | 角速度采样点数 |

**目标容忍度：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `xy_goal_tolerance` | 0.10 m | 到达目标的位置容差 |
| `trans_stopped_velocity` | 0.1 m/s | 判定为停止的平移速度阈值 |

**Critics（评价器）配置：**

| 评价器 | Scale | 作用 |
|--------|-------|------|
| `RotateToGoal` | 32.0 | 奖励朝向目标方向的角速度；`slowing_factor=5.0` 控制接近目标时减速程度，`lookahead_time=-1.0` 禁用前瞻 |
| `BaseObstacle` | 0.02 | 惩罚与障碍物碰撞的轨迹 |
| `PathAlign` | 32.0 | 奖励对齐全局路径的轨迹；`forward_point_distance=0.1` 定义前瞻距离 |
| `GoalAlign` | 24.0 | 奖励对齐到目标点方向的轨迹；`forward_point_distance=0.1` |
| `PathDist` | 32.0 | 惩罚偏离全局路径的轨迹，值越大越靠近路径 |
| `GoalDist` | 24.0 | 奖励接近目标点的轨迹 |
| `Oscillation` | （默认） | 惩罚让机器人在原地振荡的轨迹，内置评价器无需额外 scale 配置 |

**Progress Checker（进度检测器）：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `required_movement_radius` | 0.1 m | 判定"正在移动"的最小位移 |
| `movement_time_allowance` | 10.0 s | 无移动超时，触发恢复行为 |

### 5.3 planner_server (Navfn) — 全局规划器

| 参数 | 值 | 说明 |
|------|-----|------|
| `expected_planner_frequency` | 5.0 Hz | 期望的全局规划频率 |
| `planner_plugins` | `["GridBased"]` | 规划器插件列表 |
| 插件 | `nav2_navfn_planner/NavfnPlanner` | 基于 Navfn 的全局规划器 |
| `tolerance` | 0.5 m | 路径规划目标容差 |
| `use_astar` | False | 使用 Dijkstra 算法（False），非 A* |
| `allow_unknown` | True | 允许穿越未知区域（竞赛场景推荐开启） |

### 5.4 smoother_server — 路径平滑器

| 参数 | 值 | 说明 |
|------|-----|------|
| 插件 | `nav2_smoother::SimpleSmoother` | 简单迭代平滑器 |
| `tolerance` | 1.0e-10 | 收敛容差 |
| `max_its` | 1000 | 最大迭代次数 |
| `do_refinement` | True | 启用细化平滑 |

### 5.5 global_costmap — 全局代价地图

| 参数 | 值 | 说明 |
|------|-----|------|
| `update_frequency` | 1.0 Hz | 地图更新频率 |
| `publish_frequency` | 1.0 Hz | 地图发布频率 |
| `global_frame` | `map` | 全局坐标系，与静态地图一致 |
| `robot_base_frame` | `base_link` | 机器人基底坐标系 |
| `rolling_window` | False | 固定窗口模式，随机器人移动但保持地图大小不变 |
| `resolution` | 0.05 m | 栅格分辨率 |
| `robot_radius` | 0.25 m | 机器人外接圆半径 |
| `track_unknown_space` | True | 追踪未知空间 |

**插件层配置：**

| 层 | 插件 | 关键参数 |
|----|------|---------|
| `static_layer` | `nav2_costmap_2d::StaticLayer` | 从 map_server 订阅静态地图，`map_subscribe_transient_local=True` |
| `obstacle_layer` | `nav2_costmap_2d::ObstacleLayer` | 数据源 `/scan`（LaserScan），`obstacle_max_range=2.5m`，`raytrace_max_range=3.0m`，2D 雷达 `max_obstacle_height=0` |
| `inflation_layer` | `nav2_costmap_2d::InflationLayer` | `inflation_radius=0.5m`，`cost_scaling_factor=3.0` |

### 5.6 local_costmap — 局部代价地图

| 参数 | 值 | 说明 |
|------|-----|------|
| `update_frequency` | 5.0 Hz | 局部地图更新频率，高于全局 |
| `publish_frequency` | 2.0 Hz | 局部地图发布频率 |
| `global_frame` | `odom` | 里程计坐标系（随机器人移动） |
| `rolling_window` | True | 以机器人为中心的滚动窗口 |
| `width` / `height` | 4 m | 局部地图尺寸 |
| `resolution` | 0.05 m | 栅格分辨率，与全局一致 |
| `robot_radius` | 0.25 m | 机器人外接圆半径 |

**插件层配置：**

| 层 | 插件 | 关键参数 |
|----|------|---------|
| `obstacle_layer` | `nav2_costmap_2d::ObstacleLayer` | `/scan` 数据源，`obstacle_max_range=6.0m`，`raytrace_max_range=8.0m`，`obstacle_min_range=0.25m`（LiDAR 盲区） |
| `inflation_layer` | `nav2_costmap_2d::InflationLayer` | `inflation_radius=0.5m`，`cost_scaling_factor=3.0` |

### 5.7 behavior_server — 恢复行为

| 参数 | 值 | 说明 |
|------|-----|------|
| `cycle_frequency` | 10.0 Hz | 行为检查循环频率 |
| `local_costmap_topic` | `local_costmap/costmap_raw` | 局部代价地图话题 |
| `global_costmap_topic` | `global_costmap/costmap_raw` | 全局代价地图话题 |
| `simulate_ahead_time` | 2.0 s | 恢复行为仿真前瞻时间 |

**四种恢复行为：**

| 行为 | 插件 | 触发条件 | 参数 |
|------|------|---------|------|
| Spin | `nav2_behaviors/Spin` | 路径阻塞，原地旋转寻找可行方向 | `max_rotational_vel=1.0 rad/s`，`min_rotational_vel=0.2 rad/s`，`rotational_acc_lim=1.5 rad/s^2` |
| BackUp | `nav2_behaviors/BackUp` | 可能卡住时倒退 | 同上，反向移动 |
| DriveOnHeading | `nav2_behaviors/DriveOnHeading` | 按固定航向直线行驶一段距离 | 同上 |
| Wait | `nav2_behaviors/Wait` | 等待代价地图清除后再尝试 | — |

### 5.8 velocity_smoother — 速度平滑器

| 参数 | 值 | 说明 |
|------|-----|------|
| `smoothing_frequency` | 20.0 Hz | 平滑输出频率 |
| `scale_velocities` | False | 不按比例缩放速度 |
| `feedback` | `OPEN_LOOP` | 开环模式，不依赖里程计反馈进行闭环校正 |
| `max_velocity` | `[1.5, 1.5, 1.0]` | 最大速度 `[vx, vy, vth]`（全向底盘） |
| `min_velocity` | `[-1.5, -1.5, -1.0]` | 最小/反向速度 `[vx, vy, vth]` |
| `max_accel` | `[2.5, 2.5, 1.5]` | 最大加速度 `[ax, ay, ath]` |
| `max_decel` | `[-2.5, -2.5, -1.0]` | 最大减速度 `[ax, ay, ath]` |
| `odom_topic` | `/odom` | 里程计话题（用于闭环模式时的反馈） |
| `odom_duration` | 0.1 s | 里程计数据有效期 |
| `velocity_timeout` | 1.0 s | 速度命令超时，超时后输出零速度 |

### 5.9 waypoint_follower — 航点跟随器

| 参数 | 值 | 说明 |
|------|-----|------|
| `loop_rate` | 20 Hz | 主循环频率 |
| `stop_on_failure` | False | 航点失败不停止整体任务 |
| `waypoint_task_executor_plugin` | `wait_at_waypoint` | 到达航点后的执行器 |
| `waypoint_pause_duration` | 1 s | 到达每个航点后暂停时间 |

### 5.10 map_server — 地图服务器参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `yaml_filename` | 由 launch 文件传入 | 占位符，launch 文件启动时覆盖为真机或仿真地图路径 |

---

## 6. 运行方式

### 6.1 真机运行

```bash
ros2 launch at_nav2 at_nav.launch.py
```

启动内容：
- Cartographer 纯定位（加载 `maps/map.pbstream`，配置 `cartographer_localization.lua`）
- map_server（加载 `maps/map.yaml`，分辨率 0.02m）
- Nav2 导航栈
- RViz2（rviz2 配置文件为空，需手动配置或使用 `nav2_gazebo.rviz`）

### 6.2 仿真运行

```bash
ros2 launch at_nav2 at_nav_gazebo.launch.py
```

启动内容：
- Cartographer 纯定位（加载 `maps/gazebo_map.pbstream`，配置 `cartographer_localization_gazebo.lua`）
- map_server（加载 `maps/gazebo_map.yaml`，分辨率 0.05m）
- Nav2 导航栈
- RViz2（加载 `rviz2/nav2_gazebo.rviz`）

### 6.3 真机 vs 仿真参数差异

| 参数项 | 真机 (`at_nav.launch.py`) | 仿真 (`at_nav_gazebo.launch.py`) |
|--------|--------------------------|-----------------------------------|
| `use_sim_time` | False | True |
| 地图 yaml | `maps/map.yaml`（0.02m 分辨率） | `maps/gazebo_map.yaml`（0.05m 分辨率） |
| pbstream 文件 | `maps/map.pbstream`（需手动准备） | `maps/gazebo_map.pbstream`（已内置） |
| Cartographer lua | `cartographer_localization.lua` | `cartographer_localization_gazebo.lua` |
| RViz2 配置 | `rviz2/at_nav2.rviz`（可能不存在） | `rviz2/nav2_gazebo.rviz` |

> **注意：** 真机 pbstream 文件 `maps/map.pbstream` 需要提前通过 `robot_cartographer_mapping` 包建图获得；目前仓库中真机 pbstream 未提交（文件较大），需运行前手动放置。

---

## 7. 关键 Topic 数据流

```
                          ┌─────────────────────────────────┐
                          │          lslidar_driver          │
                          └──────────────┬──────────────────┘
                                         │ /scan (sensor_msgs/LaserScan)
                    ┌────────────────────┼────────────────────┐
                    ▼                    ▼                    ▼
          ┌─────────────────┐  ┌───────────────┐  ┌──────────────────┐
          │ cartographer_node│  │ global_costmap│  │  local_costmap   │
          │   (纯定位)       │  │ (obstacle_lay)│  │  (obstacle_lay)  │
          └────────┬────────┘  └───────┬───────┘  └────────┬─────────┘
                   │                   │                    │
                   │ map → odom TF     │                    │
                   │ (广播)            │                    │
                   ▼                   ▼                    ▼
          ┌────────────────────────────────────────────────────────┐
          │                    Nav2 导航栈                          │
          │  bt_navigator → planner_server → controller_server     │
          │                      │                   │             │
          │                  /plan (Path)      /cmd_vel (Twist)    │
          └──────────────────────────────────────┬─────────────────┘
                                                 │ /cmd_vel
                                                 ▼
                                        ┌────────────────┐
                                        │ competition_fsm │  ← FSM 仲裁
                                        └───────┬────────┘
                                                │ /motor_cmd_vel
                                                ▼
                                        ┌────────────────┐
                                        │  底盘驱动       │
                                        │ (motor_driver)  │
                                        └────────────────┘
```

**数据流说明：**

1. LiDAR 的 `/scan` 同时被 Cartographer、global_costmap、local_costmap 消费
2. Cartographer 利用 `/scan` + `/odom` 进行子图匹配，发布 `map -> odom` TF
3. global_costmap 融合静态地图 + `/scan` 障碍物 → 供 planner_server 使用
4. local_costmap 融合 `/scan` 障碍物 → 供 controller_server (DWB) 使用
5. controller_server 输出 `/cmd_vel` → `competition_fsm` 仲裁 → `/motor_cmd_vel` → 底盘执行

---

## 8. 与其他包的协作关系

### 上游（输入依赖）

| 上游包 | 提供 | 说明 |
|--------|------|------|
| `lslidar_driver` | `/scan` | 激光雷达扫描数据，Nav2 障碍物感知核心输入 |
| `robot_description` | TF（`base_link` 等） | 机器人 URDF 模型，提供坐标系变换 |
| `robot_cartographer_mapping` | `.pbstream` 地图文件 | 预建地图，Cartographer 纯定位必需 |
| 底盘驱动 | `/odom` | 里程计话题，供 Cartographer 和 velocity_smoother 使用 |

### 下游（输出消费者）

| 下游包 | 消费 | 说明 |
|--------|------|------|
| `competition_fsm` | `/cmd_vel` | 导航输出速度指令，FSM 仲裁后转发 |
| `rviz2` | 各 costmap、plan、marker 话题 | 可视化导航状态 |

---

## 9. Behavior Tree 说明

`bt_navigator.xml` 定义了本项目的导航行为树，采用最简结构：

```
MainTree (Sequence)
├── ComputePathToPose     # 全局规划：计算从当前位置到目标的路径
└── FollowPath            # 路径跟踪：DWB 控制器沿路径行驶
```

这是一个不包含恢复行为的简化行为树；恢复行为（Spin/BackUp/DriveOnHeading/Wait）由 `behavior_server` 以插件形式注册，当 Nav2 检测到机器人卡住时自动调用。如需在行为树中显式编排恢复逻辑，可修改 XML 加入 Recovery 子树。

---

## 10. 故障排查

| 现象 | 可能原因 | 排查方法 |
|------|---------|---------|
| RViz2 中机器人不显示 | TF 树不完整 | `ros2 run tf2_tools view_frames` 检查 TF 树，确认 `map -> odom -> base_link` 链路完整 |
| 机器人定位漂移/错误 | Cartographer 匹配失败或 pbstream 与实际环境不符 | `ros2 topic echo /tf` 检查 `map->odom` 变换是否稳定，观察 Cartographer 终端日志 |
| Nav2 不规划路径 | global_costmap 无地图或 inflation 过大 | `ros2 topic echo /global_costmap/costmap` 检查地图数据，RViz2 中添加 costmap 图层 |
| 规划了路径但机器人不走 | controller_server 未激活或 lifecycle 未完成 | `ros2 lifecycle list /controller_server` 查看状态，确保已到 active 状态 |
| DWB 报 "no valid trajectory" | 局部代价地图中障碍物过密或采样不足 | 检查 local_costmap 膨胀半径，增加 `vx_samples`/`vy_samples`，确认障碍物距离合理 |
| 机器人反复 Spin 恢复行为 | 路径被障碍物完全阻塞或局部代价地图更新不及时 | RViz2 中查看障碍物位置，确认 `/scan` 频率正常，检查 `obstacle_max_range` |
| `map -> odom` TF 缺失 | Cartographer 未启动或 pbstream 加载失败 | 检查 Cartographer 节点进程和终端日志，确认 pbstream 文件路径和内容有效 |
| `bt_navigator` 提示缺少 action 服务器 | 导航节点 lifecycle 未完全启动 | 等待约 10 秒让所有节点 lifecycle 完成，或 `ros2 lifecycle list` 逐个检查 |

---

## 11. 参数调优建议

### 按场景调整

| 场景 | 调优参数 | 建议值范围 |
|------|---------|-----------|
| 狭窄通道 | `inflation_radius`（降低） | 0.1 ~ 0.3 m |
| 高速行驶 | `max_vel_x`（提高），`acc_lim_x`（提高） | 视底盘能力定 |
| 定位不稳定 | Cartographer `max_range`（降低），`min_score`（提高） | range 2.0~3.0，score 0.6~0.7 |
| 规划过慢 | `expected_planner_frequency`（降低），`use_astar=True` | freq 2.0~5.0 |
| 避障太激进 | `BaseObstacle.scale`（提高），`PathDist.scale`（降低） | BaseObstacle 0.03~0.05 |

### 全向底盘 vs 差速底盘

如需切换至差速底盘，修改以下参数：

```yaml
# controller_server
min_vel_y: 0.0
max_vel_y: 0.0
vy_samples: 1   # 只需 1 个采样点（0 速度）
acc_lim_y: 0.0
decel_lim_y: 0.0

# velocity_smoother
max_velocity: [1.5, 0.0, 1.0]
min_velocity: [-1.5, 0.0, -1.0]
max_accel: [2.5, 0.0, 1.5]
max_decel: [-2.5, 0.0, -1.0]
```
