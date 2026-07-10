# send_navigation_target

> **定位：** 导航目标发送节点 — 通过 ROS2 服务接口将航点标识（P1-P15）转换为 Nav2 导航目标
> **核心依赖：** `rclcpp`, `rclcpp_action`, `nav2_msgs`, `geometry_msgs`
> **上游：** 总控系统（服务调用方）
> **下游：** `at_nav2`（NavigateToPose Action Server）

---

## 1. 包概述

`send_navigation_target` 是一个 C++ ROS2 节点，提供 `navigate_to_target` 服务，作为总控系统与 Nav2 导航栈之间的桥梁；总控只需指定目标点标识（如 "P1"），节点自动查表获取完整位姿（含朝向），通过 Nav2 的 `NavigateToPose` Action 发送导航目标，并同步等待导航结果

## 2. 工作流程

```
总控系统 ──(navigate_to_target 服务)──→ send_navigation_target
                                            │
                                    查表: P1→(0.67, 0.05), …, P15→(0.02, -0.01)
                                            │
                              ┌─ NavigateToPose Action ─→ Nav2 (at_nav2)
                              │
                              └─ 同步等待（最长 300 秒）
                                            │
                                            ▼
                              返回: success + message
```

## 3. 线程模型

节点使用 `MultiThreadedExecutor` + 双回调组架构：

| 线程组 | 类型 | 职责 |
|--------|------|------|
| 默认互斥组 | `MutuallyExclusive` | 处理 `navigate_to_target` 服务回调（阻塞等待） |
| Action 回调组 | `Reentrant` | 处理 `NavigateToPose` Action 的 goal_response / feedback / result 回调 |

通过 `condition_variable` 桥接两个线程组，使服务线程可以同步等待 Action 完成

## 4. 包结构

```
send_navigation_target/
├── CMakeLists.txt                          # 构建配置
├── package.xml                             # 包元数据
├── README.md                               # 本文档
├── srv/
│   └── NavigateToTarget.srv                # 自定义服务定义
├── include/send_navigation_target/         # 头文件（预留）
└── src/
    └── send_navigation_target.cpp          # 节点实现
```

## 5. 服务接口

### NavigateToTarget.srv

**请求：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `waypoint_id` | `string` | 目标点标识：P1 ~ P15 |

**响应：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 是否成功到达目标点 |
| `message` | `string` | 附加信息（成功/失败原因） |

## 6. 预设航点

数据来源：`Atlas/navigation_system/目标.txt`（RViz 点击导航点记录的日志）

| 标识 | x (m) | y (m) | qz | qw |
|------|-------|-------|----|----|
| P1 | 0.798 | -0.112 | 0.017 | 1.000 |
| P2 | 1.339 | -0.020 | 0.002 | 1.000 |
| P3 | 1.904 | -0.069 | -0.026 | 1.000 |
| P4 | 2.385 | -0.094 | 0.013 | 1.000 |
| P5 | 2.174 | -1.129 | 0.021 | 1.000 |
| P6 | 1.665 | -0.929 | 0.019 | 1.000 |
| P7 | 1.115 | -0.931 | 0.014 | 1.000 |
| P8 | 0.498 | -0.876 | -0.001 | 1.000 |
| P9 | 0.119 | -0.876 | 0.001 | 1.000 |
| P10 | 0.063 | -1.755 | -0.010 | 1.000 |
| P11 | 0.763 | -1.744 | 0.036 | 0.999 |
| P12 | 1.178 | -1.847 | 0.000 | 1.000 |
| P13 | 1.661 | -1.775 | 0.068 | 0.998 |
| P14 | -0.002 | -1.843 | 0.001 | 1.000 |
| P15 | -0.009 | 0.211 | -0.009 | 1.000 |

> 修改航点：编辑 `send_navigation_target.cpp` 中的 `waypoint_map_` 字典

## 7. 运行方式

### 编译

```bash
cd ~/AT_Atlas_nav_ws
colcon build --symlink-install --packages-select send_navigation_target
source install/setup.bash
```

### 独立运行

```bash
ros2 run send_navigation_target send_navigation_target
```

### 通过 robot_startup 启动（推荐）

```bash
ros2 launch robot_startup robot_autonomous_transport.launch.py
```

### 调用服务

```bash
# 前往航点 P1
ros2 service call /navigate_to_target send_navigation_target/srv/NavigateToTarget \
  "{waypoint_id: 'P1'}"

# 前往航点 P8
ros2 service call /navigate_to_target send_navigation_target/srv/NavigateToTarget \
  "{waypoint_id: 'P8'}"
```

## 8. 故障排查

| 现象 | 原因 | 解决 |
|------|------|------|
| 节点启动后立即退出 | `navigate_to_pose` Action Server 未就绪（10s 超时） | 确保 `at_nav2` 已完全启动 |
| 服务调用返回 "未知的目标点标识" | waypoint_id 不在 P1-P15 中 | 检查输入标识（大小写敏感，如 P1 不是 p1） |
| 导航超时 | 300s 内未到达目标 | 检查导航栈状态、路径是否可达 |
| 编译报错找不到 `nav2_msgs` | nav2_msgs 未安装 | `sudo apt install ros-humble-nav2-msgs` |
