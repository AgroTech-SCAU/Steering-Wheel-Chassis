# at_nav2 - 导航系统核心配置包

> **定位：** Nav2 导航栈的一站式配置 / 启动 / 地图管理包；核心功能三件套：Cartographer 纯定位 + Nav2 导航栈 + 地图管理。
> **核心依赖：** `nav2_bringup`, `nav2_planner`, `nav2_controller`, `nav2_costmap_2d`, `cartographer_ros`

---

## 1. 包概述

`at_nav2` 是本导航子系统的核心包，负责：

- 通过 Cartographer ROS 节点加载预建 pbstream 地图，提供纯定位（map -> odom TF）
- 加载静态占据栅格地图（pgm + yaml），供 global_costmap 使用
- 启动完整的 Nav2 导航栈（behavior tree、planner、controller、smoother、costmap、waypoint follower、velocity smoother）
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
│   ├── at_nav2_params.yaml               # Nav2 全参数配置（核心文件）
│   ├── bt_navigator.xml                  # Behavior Tree XML（ComputePathToPose → FollowPath）
│   └── cartographer_localization.lua     # Cartographer 纯定位配置
├── launch/
│   └── at_nav.launch.py                  # 导航启动文件
├── maps/
│   ├── ruikang.yaml                      # 地图描述（分辨率 0.02m）
│   ├── ruikang.pgm                       # 占据栅格地图
│   └── ruikang.pbstream                  # Cartographer pbstream 地图
└── rviz2/                                # RViz2 可视化配置（预留）
```

---

## 4. 启动流程图

```
at_nav.launch.py
├── cartographer_node          # Cartographer 纯定位（加载 .pbstream 地图）
│   ├── 发布 map -> odom TF    # 核心：提供全局定位
│   └── 订阅 /scan, /odom
├── map_server                 # 静态地图服务器
│   └── 加载 .pgm + .yaml → 发布 /map topic
├── lifecycle_manager_map      # map_server 生命周期管理器（autostart=True）
└── navigation_launch.py       # Nav2 导航栈（nav2_bringup 提供）
    ├── bt_navigator           # 行为树导航器
    ├── planner_server         # 全局规划器（Navfn）
    ├── controller_server      # 本地控制器（DWB）
    ├── smoother_server        # 路径平滑器
    ├── behavior_server        # 恢复行为（Spin/BackUp/DriveOnHeading/Wait）
    ├── velocity_smoother      # 速度平滑器
    ├── waypoint_follower      # 航点跟随器
    ├── global_costmap         # 全局代价地图
    └── local_costmap          # 局部代价地图
```

### 关键架构说明

- **不启动 AMCL**：Cartographer 负责 `map -> odom` 的 TF 发布，AMCL 不启动以避免冲突
- **控制器输出路径**：Nav2 默认发布 `/cmd_vel`，本包启动时 remap 到 `/atlas/navigation/cmd_vel`；随后由 `atlas_mission_manager` 做安全门控并转发到 `/motor_cmd_vel`，再由 `mcu_comm_bridge` 下发到底盘。不要让 Nav2 直接发布到 `/motor_cmd_vel`。

---

## 5. Atlas 任务栈相关 launch 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `cmd_vel_output` | `/atlas/navigation/cmd_vel` | Nav2 `/cmd_vel` remap 目标，供任务状态机安全门控。 |
| `params_file` | `config/at_nav2_params.yaml` | Nav2 参数文件。 |
| `map` | `maps/ruikang.yaml` | map_server 使用的占据栅格地图。 |
| `pbstream` | `maps/ruikang.pbstream` | Cartographer 纯定位使用的 pbstream。 |

## 6. Nav2 参数详解

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

行为树 XML 内容（`bt_navigator.xml`）：

```xml
<Sequence name="root">
    <ComputePathToPose goal="${goal}" path="${path}" planner_id="GridBased"/>
    <FollowPath path="${path}" controller_id="FollowPath"/>
</Sequence>
```

### 5.2 controller_server (DWB) — 本地控制器

**基础参数：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `controller_frequency` | 10.0 Hz | 控制循环频率 |
| `failure_tolerance` | 3.0 s | 允许控制失败的最大容忍时间 |
| `transform_tolerance` | 0.2 s | TF 变换容忍时间 |
| `sim_time` | 1.7 s | 轨迹仿真时间窗口 |
| `linear_granularity` | 0.05 m | 线性轨迹插值间隔 |
| `angular_granularity` | 0.025 rad | 角度轨迹插值间隔 |
| `stateful` | True | 保持内部状态 |

**速度约束（全向底盘）：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `min_vel_x` / `max_vel_x` | -1.5 / 1.5 m/s | 前进速度范围 |
| `min_vel_y` / `max_vel_y` | -1.5 / 1.5 m/s | 侧向速度范围 |
| `max_vel_theta` | 1.0 rad/s | 最大角速度 |
| `max_speed_xy` | 2.0 m/s | 最大平面合成速度 |

**加速度约束：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `acc_lim_x` / `acc_lim_y` | 2.5 m/s^2 | 线加速度限制 |
| `acc_lim_theta` | 3.2 rad/s^2 | 角加速度限制 |
| `decel_lim_x` / `decel_lim_y` | -2.5 m/s^2 | 线减速度限制 |
| `decel_lim_theta` | -3.2 rad/s^2 | 角减速度限制 |

**Critics（评价器）配置：**

| 评价器 | Scale | 作用 |
|--------|-------|------|
| `RotateToGoal` | 32.0 | 奖励朝向目标方向的角速度 |
| `BaseObstacle` | 0.02 | 惩罚与障碍物碰撞的轨迹 |
| `PathAlign` | 32.0 | 奖励对齐全局路径的轨迹 |
| `GoalAlign` | 24.0 | 奖励对齐到目标点方向的轨迹 |
| `PathDist` | 32.0 | 惩罚偏离全局路径的轨迹 |
| `GoalDist` | 24.0 | 奖励接近目标点的轨迹 |
| `Oscillation` | （默认） | 惩罚振荡轨迹 |

**Progress Checker：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `required_movement_radius` | 0.1 m | 判定移动的最小位移 |
| `movement_time_allowance` | 10.0 s | 无移动超时 |

### 5.3 planner_server (Navfn)

| 参数 | 值 | 说明 |
|------|-----|------|
| `expected_planner_frequency` | 5.0 Hz | 期望规划频率 |
| `planner_plugins` | `["GridBased"]` | Navfn 规划器 |
| `tolerance` | 0.5 m | 目标容差 |
| `use_astar` | False | Dijkstra 算法 |
| `allow_unknown` | True | 允许穿越未知区域 |

### 5.4 smoother_server

| 参数 | 值 | 说明 |
|------|-----|------|
| 插件 | `nav2_smoother::SimpleSmoother` | 简单迭代平滑器 |
| `tolerance` | 1.0e-10 | 收敛容差 |
| `max_its` | 1000 | 最大迭代次数 |

### 5.5 global_costmap

| 参数 | 值 | 说明 |
|------|-----|------|
| `global_frame` | `map` | 全局坐标系 |
| `robot_base_frame` | `base_link` | 机器人基底坐标系 |
| `rolling_window` | False | 固定窗口模式 |
| `resolution` | 0.05 m | 栅格分辨率 |
| `robot_radius` | 0.25 m | 机器人外接圆半径 |

**插件层：**

| 层 | 插件 | 关键参数 |
|----|------|---------|
| `static_layer` | `nav2_costmap_2d::StaticLayer` | 从 map_server 订阅静态地图 |
| `obstacle_layer` | `nav2_costmap_2d::ObstacleLayer` | 数据源 `/scan`，`obstacle_max_range=2.5m` |
| `inflation_layer` | `nav2_costmap_2d::InflationLayer` | `inflation_radius=0.5m`，`cost_scaling_factor=3.0` |

### 5.6 local_costmap

| 参数 | 值 | 说明 |
|------|-----|------|
| `global_frame` | `odom` | 里程计坐标系 |
| `rolling_window` | True | 以机器人为中心的滚动窗口 |
| `width` / `height` | 4 m | 局部地图尺寸 |
| `resolution` | 0.05 m | 栅格分辨率 |

**插件层：**

| 层 | 插件 | 关键参数 |
|----|------|---------|
| `obstacle_layer` | `nav2_costmap_2d::ObstacleLayer` | `/scan` 数据源，`obstacle_max_range=6.0m`，`raytrace_max_range=8.0m` |
| `inflation_layer` | `nav2_costmap_2d::InflationLayer` | `inflation_radius=0.5m`，`cost_scaling_factor=3.0` |

### 5.7 behavior_server — 恢复行为

| 行为 | 触发条件 | 参数 |
|------|---------|------|
| Spin | 路径阻塞 | `max_rotational_vel=1.0 rad/s` |
| BackUp | 卡住倒退 | 同上，反向移动 |
| DriveOnHeading | 按航向直线行驶 | 同上 |
| Wait | 等待代价地图清除 | — |

### 5.8 velocity_smoother

| 参数 | 值 | 说明 |
|------|-----|------|
| `smoothing_frequency` | 20.0 Hz | 平滑输出频率 |
| `feedback` | `OPEN_LOOP` | 开环模式 |
| `max_velocity` | `[1.5, 1.5, 1.0]` | 全向底盘速度上限 |
| `min_velocity` | `[-1.5, -1.5, -1.0]` | 反向速度上限 |
| `max_accel` | `[2.5, 2.5, 1.5]` | 加速度限制 |
| `max_decel` | `[-2.5, -2.5, -1.0]` | 减速度限制 |

### 5.9 waypoint_follower

| 参数 | 值 | 说明 |
|------|-----|------|
| `loop_rate` | 20 Hz | 主循环频率 |
| `stop_on_failure` | False | 航点失败不停止 |
| `waypoint_pause_duration` | 1 s | 航点暂停时间 |

---

## 6. 运行方式

```bash
ros2 launch at_nav2 at_nav.launch.py
```

启动内容：
- Cartographer 纯定位（加载 `maps/ruikang.pbstream`，配置 `cartographer_localization.lua`）
- map_server（加载 `maps/ruikang.yaml`）
- Nav2 导航栈（planner / controller / behavior tree / smoother / costmap / velocity_smoother）
- `use_sim_time` 默认为 False（真机模式）

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
                                        │ mcu_comm_bridge │  ← MCU 桥接
                                        └───────┬────────┘
                                                │ /motor_cmd_vel
                                                ▼
                                        ┌────────────────┐
                                        │  底盘驱动       │
                                        └────────────────┘
```

---

## 8. 与其他包的协作关系

### 上游（输入依赖）

| 上游包 | 提供 | 说明 |
|--------|------|------|
| `lslidar_driver` | `/scan` | 激光雷达扫描数据 |
| `robot_description` | TF（`base_link` 等） | 机器人 URDF 模型 |
| `robot_cartographer_mapping` | `.pbstream` 地图文件 | 预建地图 |
| 底盘驱动 / `mcu_comm_bridge` | `/odom` | 里程计话题 |

### 下游（输出消费者）

| 下游包 | 消费 | 说明 |
|--------|------|------|
| `mcu_comm_bridge` | `/cmd_vel` | 桥接后转发至底盘 |
| `send_navigation_target` | `navigate_to_pose` Action | 发送导航目标 |
| `rviz2` | costmap、plan、marker 话题 | 可视化导航状态 |

---

## 9. 故障排查

| 现象 | 可能原因 | 排查方法 |
|------|---------|---------|
| RViz2 中机器人不显示 | TF 树不完整 | `ros2 run tf2_tools view_frames` 检查 TF 树 |
| 机器人定位漂移 | Cartographer 匹配失败 | 检查 pbstream 与实际环境是否一致 |
| Nav2 不规划路径 | global_costmap 无地图 | `ros2 topic echo /global_costmap/costmap` 检查 |
| 规划了路径但机器人不走 | controller_server 未激活 | `ros2 lifecycle list /controller_server` |
| DWB 报 "no valid trajectory" | 障碍物过密或采样不足 | 检查 local_costmap 膨胀半径 |
| `map -> odom` TF 缺失 | Cartographer 未启动或 pbstream 加载失败 | 检查 Cartographer 终端日志 |
| `bt_navigator` 提示缺少 action 服务器 | lifecycle 未完成 | 等待约 10 秒，或逐个检查 lifecycle 状态 |

---

## 10. 参数调优建议

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
vy_samples: 1
acc_lim_y: 0.0
decel_lim_y: 0.0

# velocity_smoother
max_velocity: [1.5, 0.0, 1.0]
min_velocity: [-1.5, 0.0, -1.0]
max_accel: [2.5, 0.0, 1.5]
max_decel: [-2.5, 0.0, -1.0]
```
