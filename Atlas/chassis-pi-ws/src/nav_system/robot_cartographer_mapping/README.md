# robot_cartographer_mapping

基于 Cartographer 的 2D SLAM 建图功能包，用于在真机环境中生成 `map.pbstream` 和 `map.pgm` / `map.yaml` 地图文件。

## 前置条件

- ROS2 Humble
- 已安装 `cartographer_ros`
- 已安装 `nav2_map_server`
- 已构建本工作空间
- LSLIDAR N10P 雷达已连接

```bash
sudo apt install ros-humble-cartographer-ros ros-humble-nav2-map-server
```

## 快速开始

### 1. 构建 & 加载环境

```bash
cd ~/AT_Atlas_nav_ws
colcon build --symlink-install --packages-select robot_cartographer_mapping
source install/setup.bash
```

### 2. 启动 Cartographer 建图

建图 launch 文件会同时拉起雷达驱动、MCU 桥接和 TF 发布：

```bash
ros2 launch robot_cartographer_mapping robot_cartographer_mapping.launch.py
```

启动内容：
- `lslidar_driver` — 激光雷达驱动（`/scan`）
- `mcu_comm_bridge` — MCU 通信桥接（`/odom`）
- `robot_description` — robot_state_publisher（TF）
- `cartographer_node` — Cartographer SLAM 节点
- `cartographer_occupancy_grid_node` — 占据栅格地图发布（`/map`）

### 3. 控制机器人移动完成建图

使用键盘/手柄遥控机器人在环境中移动，直到地图覆盖所有目标区域。

### 4. 保存地图

建图完成后，**先确认 `/write_state` 服务存在**：

```bash
ros2 service list | grep write_state
```

保存 **pbstream** 文件（Cartographer 内部格式，用于后续纯定位）：

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$(pwd)/<地图名字>.pbstream'}"
```

然后生成 **PGM + YAML** 格式（Nav2 导航使用）：

```bash
ros2 run nav2_map_server map_saver_cli -t map -f <地图名字>
```

生成的文件：
- `<地图名字>.pgm` — 占据栅格地图图像
- `<地图名字>.yaml` — 地图元数据（分辨率、原点、阈值）

## 包结构

```
robot_cartographer_mapping/
├── config/
│   └── robot_2d.lua                        # Cartographer 2D SLAM 配置
├── launch/
│   └── robot_cartographer_mapping.launch.py # 建图启动文件（含雷达/MCU/TF）
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Launch 参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `use_sim_time` | `false` | 是否使用仿真时间 |
| `resolution` | `0.05` | 占据栅格地图分辨率（m） |
| `publish_period_sec` | `1.0` | 地图发布周期（s） |
| `configuration_directory` | `config/` | Cartographer 配置目录 |
| `configuration_basename` | `robot_2d.lua` | Cartographer 配置文件 |

## 与导航的衔接

| 文件 | 用途 | 下游消费者 |
|------|------|-----------|
| `<name>.pbstream` | Cartographer 纯定位初始状态 | `at_nav2` — `cartographer_localization.lua` |
| `<name>.pgm` + `<name>.yaml` | Nav2 全局代价地图 | `at_nav2` — `map_server` |

建图完成后，将 `pbstream` 和 `pgm`/`yaml` 复制到 `at_nav2/maps/` 目录，并在 `at_nav.launch.py` 中指定即可切换至导航模式。

## 故障排查

| 现象 | 检查 |
|------|------|
| 无地图显示 | `ros2 topic echo /map` 确认 Cartographer 正常发布子图 |
| `/write_state` 不存在 | Cartographer 节点未启动，检查 `ros2 node list \| grep cartographer` |
| 保存失败 | 确认目标路径可写，Cartographer lifecycle 处于 active 状态 |
| 无雷达数据 | 检查 `lslidar_driver` 是否正常启动，`ros2 topic hz /scan` |
| 无里程计 | 检查 `mcu_comm_bridge` 是否正常，`ros2 topic echo /odom` |
