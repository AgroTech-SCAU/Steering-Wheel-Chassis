# robot_cartographer_mapping

> **定位**：基于 Cartographer 的 2D SLAM 建图功能包，生成 pbstream/PGM/YAML 地图文件供导航使用 | **核心依赖**：`cartographer_ros`、`nav2_map_server` | **上游**：`robot_gazebo`（提供仿真传感器数据）| **下游**：`at_nav2`（使用建图结果进行 Cartographer 纯定位 + Nav2 导航）

---

基于 Cartographer 的 2D SLAM 建图功能包，用于在 Gazebo 仿真环境中生成 `map.pbstream` 和 `map.pgm` / `map.yaml` 地图文件。

## 前置条件

- ROS2 Humble
- 已安装 `cartographer_ros`
- 已安装 `nav2_map_server`
- 已构建本工作空间

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

### 2. 启动仿真环境

终端 A：

```bash
ros2 launch robot_gazebo gazebo_sim.launch.py
```

### 3. 启动 Cartographer 建图

终端 B：

```bash
ros2 launch robot_cartographer_mapping robot_cartographer_mapping_gazebo.launch.py
```

RViz2 会自动打开，显示 Cartographer 的建图进度。

### 4. 控制机器人移动完成建图

使用键盘/手柄遥控机器人在环境中移动，直到地图覆盖所有目标区域。

### 5. 保存地图

建图完成后，**先确认 `/write_state` 服务存在**：

```bash
ros2 service list | grep write_state
```

保存 **pbstream** 文件（Cartographer 内部格式，用于后续纯定位）：

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$(pwd)/src/robot_cartographer_mapping/map/<地图名字>.pbstream'}"
```

然后在 `map/` 目录下生成 **PGM + YAML** 格式（Nav2 导航使用）：

```bash
cd src/robot_cartographer_mapping/map
ros2 run nav2_map_server map_saver_cli -t map -f <地图名字>
```

生成的文件：
- `<地图名字>.pgm` — 占据栅格地图图像
- `<地图名字>.yaml` — 地图元数据（分辨率、原点、阈值）

## 包结构

```
robot_cartographer_mapping/
├── config/
│   └── robot_2d_gazebo.lua              # Cartographer 2D SLAM 配置
├── launch/
│   └── robot_cartographer_mapping_gazebo.launch.py  # Gazebo 建图启动文件
├── map/
│   └── gazebo_map.{pbstream,pgm,yaml}   # 默认地图文件
├── rviz/
│   └── rviz2_cartographer.rviz          # RViz2 可视化配置
├── src/                                 # C++ 源文件（预留）
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 与导航的衔接

| 文件 | 用途 | 下游消费者 |
|------|------|-----------|
| `map/<name>.pbstream` | Cartographer 纯定位初始状态 | `at_nav2` — `cartographer_localization.lua` |
| `map/<name>.pgm` + `map/<name>.yaml` | Nav2 全局代价地图 | `at_nav2` — `map_server` |

建图完成后，将 `pbstream` 和 `pgm`/`yaml` 复制到 `at_nav2/maps/` 目录，并在 `at_nav.launch.py` 中指定即可切换至导航模式。

## 故障排查

| 现象 | 检查 |
|------|------|
| 无地图显示 | `ros2 topic echo /map` 确认 Cartographer 正常发布子图 |
| `/write_state` 不存在 | Cartographer 节点未启动，检查 `ros2 node list \| grep cartographer` |
| 保存失败 | 确认目标路径可写，Cartographer lifecycle 处于 active 状态 |
