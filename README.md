<div align="center">

# Steering-Wheel-Chassis

</div>

> AgroTech 协会中型轮式机器人代码仓库  

> 本仓库用于统一管理协会内部中型轮式机器人平台的嵌入式电控代码、Pi 端底盘通信桥、ROS2 导航系统与后续视觉 / 任务编排代码  

> 机械模型已拆分到独立仓库维护，当前仓库按“一车一目录”组织，每个一级目录代表一辆独立机器人平台，便于按车型迭代、按平台维护、按任务复用

---

## 1. 仓库定位

- 面向 AgroTech 协会中型轮式机器人平台的统一代码仓库
- 当前主要包含底盘 MCU 控制代码、Pi 端通信桥、ROS2 导航定位系统、雷达驱动、仿真与启动文件
- 适合用于舵轮底盘、五自由度机械臂、导航定位、任务状态机等整车软件链路的协同开发
- 机械模型统一维护在 [`AgroTech-SCAU/Steering-Wheel-Chassis-Model-Collection`](https://github.com/AgroTech-SCAU/Steering-Wheel-Chassis-Model-Collection)

---

## 2. 当前包含的机器人

| 机器人 | 状态 | 简介 |
| --- | --- | --- |
| `SteerWheel Mk.1` | 开发中 / 原型验证 | 初代原型机，用于验证舵轮底盘、机械臂集成、电控架构与控制算法 |
| `Atlas` | 开发中 / 导航与底盘联调 | 新一代舵轮底盘平台，当前包含 MCU 底盘控制代码、Pi 端通信桥和 ROS2 导航系统 |

另有一台尚未命名的轮式机器人正在规划中，同样采用舵轮底盘与五轴五自由度机械臂，后续会在仓库中以独立文件夹形式加入

---

## 3. 目录结构

```text
.
├─ Atlas/
│  ├─ chassis_control_code/   # MCU 端底盘控制代码，烧录到 STM32 控制板
│  ├─ chassis-pi-ws/          # 树莓派 5 ROS2 工作区，负责 MCU 通信桥接
│  ├─ navigation_system/      # ROS2 导航系统，包含雷达驱动、建图、定位、Nav2 与启动文件
│  └─ README.md
├─ SteerWheel Mk.1/
│  ├─ chassis_control_code/   # STM32H7 底盘 / 机械臂嵌入式控制代码
│  └─ README.md
└─ README.md
```

---

## 4. `SteerWheel Mk.1` 简介

`SteerWheel Mk.1` 是本仓库较早期的原型平台，主要承担以下任务：

- 验证四舵轮底盘的机械结构与运动学控制
- 验证底盘与五自由度机械臂的一体化集成
- 验证自研 STM32H723 控制板的软件分层与设备驱动架构
- 为后续 `Atlas` 及其他轮式机器人积累电控和软件复用资产

其目录中当前主要包含：

- 基于 STM32H723 的底盘与机械臂控制固件
- 机械臂 `URDF`、`mesh`、`launch` 等开发相关资源

机器人机械模型请前往独立仓库查看：

- [`AgroTech-SCAU/Steering-Wheel-Chassis-Model-Collection`](https://github.com/AgroTech-SCAU/Steering-Wheel-Chassis-Model-Collection)

更详细的说明请见 [SteerWheel Mk.1/README.md](./SteerWheel%20Mk.1/README.md)

---

## 5. `Atlas` 简介

`Atlas` 是当前重点推进的新一代中型轮式机器人平台，采用舵轮底盘作为移动底盘，使用 STM32 控制板完成底层实时控制，使用树莓派 5 运行 ROS2 Humble 导航系统

当前 `Atlas` 已经包含三部分：

- `chassis_control_code`：MCU 端底盘控制代码，负责底盘运动控制、IMU / odom 数据组织、Pi 通信协议收发与状态机权限判断
- `chassis-pi-ws`：Pi 端 ROS2 通信桥，负责读取 MCU 的 IMU / ODOM 数据并发布 `/odom`、`/imu` 和 TF，同时订阅 `/motor_cmd_vel` 并下发 `PI_CONTROL`
- `navigation_system`：ROS2 导航系统，包含 LSLIDAR 驱动、URDF、Gazebo、Cartographer 建图 / 纯定位、Nav2 与总启动文件

`Atlas` 的运行目标是形成如下闭环：

```text
MCU 底盘控制程序
  ↓ MCU_IMU / MCU_ODOM
chassis-pi-ws
  ↓ /odom /imu /tf
navigation_system
  ↓ /cmd_vel
competition_fsm
  ↓ /motor_cmd_vel
chassis-pi-ws
  ↓ PI_CONTROL
MCU 底盘控制程序
```

更详细的说明请见 [Atlas/README.md](./Atlas/README.md)

---

## 6. 开发路线

### 6.1 已有内容

- 舵轮底盘 MCU 控制代码
- Pi 端 ROS2 通信桥
- IMU / ODOM 二进制协议解析
- `/odom`、`/imu` 与 `odom -> base_footprint` TF 发布
- `/motor_cmd_vel` 到 MCU `PI_CONTROL` 的速度下发链路
- ROS2 Humble 导航系统
- Cartographer 2D 建图与纯定位
- Nav2 全局规划与局部控制
- LSLIDAR N10P 雷达驱动
- Gazebo 仿真启动文件

### 6.2 计划加入

- 更完整的 `competition_fsm` 与 `mission_manager` 说明
- `mcu_comm_bridge` 与 `robot_startup` 的统一启动入口
- `/mcu/status` 状态话题或诊断信息
- 真实场地地图与定位配置
- 视觉感知节点
- 机械臂上层控制接口
- 系统级联调记录与故障排查文档

---

## 7. 面向协作者的建议

- 新车型请直接以一级目录形式加入，保持“一车一目录”
- 机械、电控、ROS、仿真和任务代码建议按职责拆分维护，避免单个包承担过多功能
- MCU 端、Pi 端和导航端的接口应优先通过协议文档、README 和 launch 参数保持一致
- 涉及底盘安全的接口应区分周期性 topic、一次性 service 和长期 action，避免控制语义混乱
- 合并新功能前建议至少验证 `/odom`、`/imu`、`/scan`、`/motor_cmd_vel`、TF 树和 MCU AutoPi 执行链路

---

## 8. TODO

- [ ] 完善 `Atlas/README.md` 中真实地图、串口设备、启动参数和联调流程
- [ ] 将 `chassis-pi-ws` 的 `mcu_comm_bridge` 纳入 `navigation_system/robot_startup` 的真机总启动链路
- [ ] 补充或确认 `competition_fsm`、`mission_manager` 的源码位置、接口和启动方式
- [ ] 补充 `/mcu/status` 话题或诊断接口，方便状态机读取 MCU ready / fault / estop 状态
- [ ] 修正或补齐 `at_nav2` 真机定位所需的 `map.pbstream`、`map.yaml` 和 RViz 配置
- [ ] 给 URDF 补充 `imu_link` 及其相对 `base_link` 的固定变换
- [ ] 完善 MCU 固件编译、烧录、协议版本和调试说明
- [ ] 补充真实场地导航联调记录、常见问题和验收 checklist

---

## 9. 说明

本仓库仍处于持续开发阶段，部分车型资料、启动链路、接口文档和任务代码仍可能随项目推进调整
