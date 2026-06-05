# Spec: navigation_system README 文档编写

## 目标

为 `Atlas/navigation_system/` 下的 7 个功能包各生成/补充 README.md，并编写一个总 README.md，形成完整的项目文档体系。

## 范围

- 新建 README 的包（5 个）：`lslidar_msgs`、`robot_description`、`robot_gazebo`、`at_nav2`、`robot_startup`
- 补充摘要头的包（2 个）：`lslidar_driver`、`robot_cartographer_mapping`
- 总 README（1 个）：`Atlas/navigation_system/README.md`

## 设计决策

- **语言**：纯中文，与项目已有文档一致
- **深度**：详细版（150-300 行），含架构说明、节点/话题/服务列表、参数详解、故障排查
- **已有 README 处理**：保留原有内容，在文件开头插入统一风格的摘要头
- **架构图**：使用 Mermaid 流程图展示包间运行时关系

## README 模版结构（每个包）

```markdown
# <包名>

> **摘要头**（统一格式）：一句话定位 + 核心依赖 + 上下游关系

## 包概述
- 功能定位、适用场景

## 包结构
- 目录树

## 依赖
- ROS2 包 + 系统依赖 + 安装命令

## 节点 / 话题 / 服务 / 动作
- 发布、订阅的 topic，提供的 service/action 列表

## 参数说明
- 关键配置参数及含义

## 运行方式
- launch 命令 + 命令行示例

## 与其他包的协作
- 数据流向图（文字描述）

## 真机 vs 仿真
- （仅 at_nav2）真机/仿真参数切换要点

## 故障排查
- 常见问题和解决方法
```

## 总 README 结构

```markdown
# AGT Navigation System

## 项目概述
- 系统定位、技术栈

## 系统架构
- Mermaid 架构图 + 文字说明

## 功能包清单
- 表格：包名、定位、关键依赖

## 快速开始
1. 环境准备
2. 构建工作空间
3. 仿真启动 + 建图
4. 导航启动
5. 真机部署

## 数据流说明
- 关键 topic 流转

## 硬件/软件要求
- Ubuntu 22.04、ROS2 Humble、Gazebo 11 等

## 目录结构
- 完整目录树

## 维护者
```

## 各包重点关注内容

### lslidar_msgs
- 2 个消息类型（`LslidarInformation`, `LslidarPacket`）和 9 个服务定义（`IpAndPort`, `TimeMode`, `MotorControl`, `MotorSpeed`, `PowerControl`, `RfdRemoval`, `TailRemoval`, `AngleDistortionCorrection`, `FrameRate`, `InvalidData`, `StandbyMode`）的完整列表
- 每个消息/服务的字段含义、取值范围、适用雷达系列

### lslidar_driver
- 统一摘要头说明本项目使用的是 N10P 单线雷达 + UART 配置
- 保留厂商原有详细文档

### robot_description
- URDF link/joint 树结构
- 传感器坐标系（laser_link 等）
- mesh 文件清单
- robot_state_publisher + joint_state_publisher_gui 启动方式

### robot_gazebo
- Gazebo 仿真世界（competition.world）
- xacro 模型编辑
- spawn 参数（初始位姿 x/y/z）
- headless 模式（gui:=false）
- NVIDIA + Wayland 兼容性说明

### robot_cartographer_mapping
- 统一摘要头说明其在系统中的定位（建图→导航衔接）
- 保留已有完整 README

### at_nav2
- **核心包**，重点内容：
  - Cartographer 纯定位 vs AMCL 的架构选择说明
  - Nav2 参数详解：controller_server (DWB)、planner_server (Navfn)、costmap (global/local)、behavior_server、velocity_smoother、waypoint_follower
  - [HW_CONFIG] 标注参数的含义和调优建议
  - 真机 vs 仿真 use_sim_time 切换
  - launch 文件：`at_nav.launch.py`（真机）vs `at_nav_gazebo.launch.py`（仿真）

### robot_startup
- 总启动编排：robot_state_publisher → lslidar_driver → mission_manager → competition_fsm → at_nav2（延迟 3s）
- 节点启动顺序的原因（/scan 和 /odom 就绪依赖）
- FSM 仲裁说明（controller_server /cmd_vel → FSM → /motor_cmd_vel）
- 依赖的外部包（mission_manager, competition_fsm）

### 总 README
- Mermaid 架构图展示全部 7 个包的运行时关系和数据流
- 从零开始的快速启动指南（一行命令链）
- 完整目录树
- 维护者信息

## 输出文件清单

| 路径 | 操作 |
|------|------|
| `Atlas/navigation_system/README.md` | 新建 |
| `Atlas/navigation_system/lslidar_msgs/README.md` | 新建 |
| `Atlas/navigation_system/lslidar_driver/README.md` | 插入摘要头 |
| `Atlas/navigation_system/robot_description/README.md` | 新建 |
| `Atlas/navigation_system/robot_gazebo/README.md` | 新建 |
| `Atlas/navigation_system/robot_cartographer_mapping/README.md` | 插入摘要头 |
| `Atlas/navigation_system/at_nav2/README.md` | 新建 |
| `Atlas/navigation_system/robot_startup/README.md` | 新建 |

## 验收标准

- [ ] 每个包都有 README.md，内容符合上述模版结构
- [ ] 总 README 含 Mermaid 架构图
- [ ] 所有 launch 命令和参数名称与源码一致
- [ ] lslidar_driver 和 robot_cartographer_mapping 原有内容完整保留
- [ ] 纯中文，技术名词保留英文原词
- [ ] 提交到 git
