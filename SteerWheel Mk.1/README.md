<div align="center">

# SteerWheel Mk.1

</div>

> AgroTech 协会中型轮式机器人初代原型机

> `SteerWheel Mk.1` 用于验证舵轮底盘、五自由度机械臂、一体化电控架构和基础遥控作业流程，是后续 `Atlas` 及同类轮式机器人平台的技术试验床与资产来源

---

## 1. 项目目标

- 验证四舵轮底盘的机械方案与控制算法
- 验证中型轮式平台与五轴五自由度机械臂的整机集成
- 建立可复用的嵌入式软件分层架构
- 为后续 ROS 视觉、导航和任务执行系统预留接口与模型基础

---

## 2. 当前状态

- 机械模型：较完整，包含整车、主动万向脚轮、机械臂、PCB、场地等模型
- 嵌入式控制：已具备底盘、机械臂、IMU、遥控、RGB 指示、日志等基础能力
- ROS 上层：仓库内已有机械臂 URDF/mesh/launch 资源，但尚未形成完整 ROS 工作流
- 项目阶段：原型机验证阶段，适合作为开发底座，不建议视为最终量产方案

---

## 3. 核心特性

- 四模块舵轮底盘
- 底盘转向与驱动双总线分离控制
- 五轴五自由度机械臂
- STM32H723 主控板
- BMI088 IMU 姿态采集
- iBus 遥控链路
- WS2812 RGB 状态指示
- 机械臂 URDF/mesh 资源与基础仿真文件

---

## 4. 系统总览

### 4.1. 机械系统

本车型目录下的 `model/` 包含完整的 SolidWorks 机械资料，能看出原型机由以下几部分构成：

- 底盘主体
  - `整车.SLDASM`
  - `装配体.SLDASM`
  - `底盘玻纤板.SLDPRT`
  - `顶部玻纤板.SLDPRT`
  - `后壳.SLDPRT`
  - `挡板.SLDPRT`
  - 电池垫、黄铜柱、PCB 安装件等结构件
- 主动万向脚轮 / 舵轮模块
  - `主动万向脚轮/主动万向脚轮.SLDASM`
  - `主动万向脚轮/转向机构.SLDASM`
  - 轮组玻纤板、轴承、同步带、滑环、连接件、打印件等零件
- 机械臂
  - `机械臂/机械臂.SLDASM`
  - `机械臂底座/` 中的底座、轴承压环、固定套等
  - `机械臂A/B/C/D/E` 等连杆零件
- 板级与外设相关模型
  - `CtrBoard-H7_V1.0.SLDPRT`
  - `PCB.SLDPRT`
  - `PCB整体.SLDASM`
  - `八路灰度传感器`、`BGS301N 光电传感器` 等
- 场地/任务环境模型
  - `场地/`
  - `场地道具/`

### 4.2. 电控系统

从 `chassis_control_code/` 与 `robot.ioc` 可以确认，原型机电控核心为一块基于 `STM32H723VGT6` 的自研控制板，固件已接入如下外设：

- `FDCAN1`
  - 转向电机总线
- `FDCAN2`
  - 驱动电机总线
- `SPI2`
  - BMI088 IMU
- `SPI6`
  - WS2812 RGB 灯数据输出
- `UART5`
  - iBus 遥控接收
- `UART7`
  - 机械臂总线舵机通信
- `USART1`
  - 调试/日志串口
- `TIM6`
  - 500 Hz 控制节拍

CubeMX 工程显示主控运行在 `500 MHz`，底盘双 CAN 均配置为 `1 Mbps`

---

## 5. 硬件推断与实现说明

以下内容来自现有模型命名与代码实现，可作为当前原型机的实际设计参考：

### 5.1. 底盘驱动与转向

`src/service/assemble/assemble_chassis.c` 中可以看到：

- 转向电机接口使用 `dm_motor`
- 驱动电机接口使用 `dji_motor`
- 底盘模型参数为
  - 长：`0.26572986916 m`
  - 宽：`0.26572986916 m`
  - 轮半径：`0.057965 m`
  - 最大轮线速度：`2.0 m/s`

而在机械模型中可以看到：

- 转向侧包含 `DM-G6220`
- 驱动侧包含 `RoboMaster M3508`
- 轮组含同步带、轴承、滑环与若干打印件/碳板/玻纤板

因此可以将当前原型理解为：

- 四个舵轮模块
- 每个模块 1 个转向电机 + 1 个驱动电机
- 转向与驱动分别通过两路 CAN 独立管理

### 5.2. 机械臂

现有代码和模型表明本机机械臂为五轴五自由度方案：

- 软件运动学接口位于 `src/domain/serial_arm/`
- 服务层接口位于 `src/service/arm.c`
- 装配时通过 `assemble_arm.c` 配置 `ARM_DOF`
- URDF 与 STL 资源位于
  - `chassis_control_code/arm_description/`
  - `model/URDF_arm/ARM/`

`assemble_arm.c` 中的默认零位关节角为：

```text
Joint 1:   0.0 deg
Joint 2:  94.5 deg
Joint 3: 135.0 deg
Joint 4: -54.0 deg
Joint 5: -13.5 deg
```

机械臂控制层支持：

- 整体关节控制
- 单关节控制
- 回舵机零位
- 回 MDH 零位
- 位姿 IK 控制
- 仅位置控制
- 仅姿态控制
- 当前关节/位姿缓存读取

### 5.3. 传感器与交互

- IMU：`BMI088`
- 遥控：`FS-iA10B / iBus`
- 状态灯：`WS2812`
- 另有灰度/光电相关模型仅用于人工智能与机器人大赛，不作为协会通用平台的标配传感器

---

## 6. 软件架构

`chassis_control_code/` 采用比较清晰的分层组织：

```text
chassis_control_code/
├─ Core/                  # STM32Cube 生成入口与底层初始化
├─ src/
│  ├─ app/                # 应用层，如遥控逻辑、系统入口
│  ├─ service/            # 服务层，负责底盘/机械臂/IMU/RGB 等系统装配
│  ├─ device/             # 设备抽象层，电机、舵机、IMU、RGB、遥控接收器
│  ├─ domain/             # 领域层，舵轮底盘与串联机械臂运动学
│  ├─ infra/              # 基础设施，日志、PID、矩阵、协议解析、HFSM 等
│  └─ platform/           # STM32 HAL 适配层
├─ arm_description/       # 机械臂 ROS 描述资源
├─ robot.ioc              # STM32CubeMX 工程
└─ robot.code-workspace
```

推荐的理解顺序：

1. `Core/Src/main.c`
2. `src/app/entry.h`
3. `src/service/assemble/*.c`
4. `src/service/chassis.c` 与 `src/service/arm.c`
5. `src/domain/`
6. `src/device/` 与 `src/platform/`

---

## 7. 启动流程

固件入口在 `Core/Src/main.c`，初始化外设后调用：

```c
entry_init();
while (1) {
    entry_loop();
}
```

`entry_init()` 的装配顺序为：

1. 延时模块
2. 日志模块
3. RGB 模块
4. IMU 模块
5. 底盘模块
6. 遥控模块
7. `TIM6 500Hz`
8. 机械臂模块

这意味着系统默认就是“先完成所有底层装配，再进入统一事件循环”的结构

---

## 8. 控制循环

`TIM6` 被配置成 `500 Hz` 周期任务，`entry_loop()` 中的主要逻辑为：

- 每个 2 ms：
  - 更新 IMU
  - 执行底盘控制 `chassis.process()`
  - 每 5 个周期执行一次遥控处理 `remote_process()`
- 每 1 s：
  - 根据底盘就绪状态和遥控在线状态更新 RGB 指示灯
  - 输出心跳日志

RGB 状态约定：

- 红色：底盘未就绪
- 绿色：底盘就绪，但遥控未在线
- 蓝色：底盘就绪且遥控在线

---

## 9. 底盘控制逻辑

底盘服务主要位于 `src/service/chassis.c`，具备以下值得关注的设计：

- 四舵轮运动学正逆解
- 转向绝对角最近解选择
- 驱动反向等效解优化
- 转向 S 曲线跟踪速度规划
- “先转向再驱动”门控
- 驱动/转向反馈缺失时的自动重试上电准备
- 驻车刹车姿态控制
- 基于线速度的偏航补偿项

当前实现中还包含一套偏航补偿模型：

```text
k_vx = 0.042
k_vy = 0.028
v_deadband = 0.01
```

这说明原型机已经开始处理舵轮平台在实际运动中的偏航误差问题，而不是只停留在理想运动学层面

---

## 10. 遥控逻辑

遥控应用位于 `src/app/remote.c`，现有行为可以概括为：

- `SWA`
  - 高位：底盘控制模式
  - 低位：机械臂控制模式
- `SWB`
  - 作为快/中/慢挡速度配置
- `SWC`
  - 在底盘模式下用于制动或切换“先转向再驱动”
  - 在机械臂模式下用于不同控制子模式
- `SWD`
  - 高位时不执行控制
- `VRA / VRB`
  - 作为控制使能/保护条件

### 10.1. 底盘模式

底盘模式下支持：

- `vx / vy / wz` 全向速度控制
- 三挡速度上限切换
- 驻车制动
- 航向保持 `yaw hold`

速度上限当前配置为：

| 挡位 | `vx` | `vy` | `wz` |
| --- | --- | --- | --- |
| 快挡 | `2.0 m/s` | `2.0 m/s` | `8.0 rad/s` |
| 中挡 | `1.0 m/s` | `1.0 m/s` | `4.0 rad/s` |
| 慢挡 | `0.5 m/s` | `0.5 m/s` | `2.0 rad/s` |

### 10.2 机械臂模式

机械臂模式下支持：

- 单独控制底座偏航关节
- 控制末端前伸/回缩
- 控制末端升降
- 控制姿态 yaw
- 一键回机械臂零位

这是一个偏“原型验证型”的手动操作方案，适合调试与任务流程试验

---

## 11. 机械臂描述与仿真资源

仓库内已经放入两套相关资源：

- `model/URDF_arm/ARM/`
- `chassis_control_code/arm_description/`

包含：

- `urdf/ARM.urdf`
- `meshes/*.STL`
- `launch/display.launch`
- `launch/gazebo.launch`
- `config/joint_names_ARM.yaml`

这对后续接入 ROS 可视化、MoveIt、Gazebo 或其他上层系统非常有帮助

---

## 12. 目录说明

### 12.1. 机械资料

- [model](./model)
  - 整车、底盘、舵轮、机械臂、板级与场地模型

### 12.2. 嵌入式代码

- [chassis_control_code](./chassis_control_code)
  - STM32H723 固件工程
- [robot.ioc](./chassis_control_code/robot.ioc)
  - STM32CubeMX 配置
- [Core/Src/main.c](./chassis_control_code/Core/Src/main.c)
  - 固件入口

### 12.3. 关键源码位置

- [src/app/entry.h](./chassis_control_code/src/app/entry.h)
- [src/app/remote.c](./chassis_control_code/src/app/remote.c)
- [src/service/chassis.c](./chassis_control_code/src/service/chassis.c)
- [src/service/arm.c](./chassis_control_code/src/service/arm.c)
- [src/service/assemble/](./chassis_control_code/src/service/assemble)
- [src/domain/](./chassis_control_code/src/domain)

---

## 13. 开发环境

当前工程适合以下工具链：

- `STM32CubeMX`
- `STM32CubeIDE`
- `arm-none-eabi-gcc`
- `SolidWorks`
- ROS 1 环境（用于已有 URDF/launch 资源的显示与验证）

仓库中可直接看到：

- `robot.ioc`
- `STM32H723VGTX_FLASH_USER.ld`
- `robot.code-workspace`

---

## 14. 编译与使用建议

### 14.1. 固件

1. 使用 `STM32CubeMX` / `STM32CubeIDE` 打开 `chassis_control_code/robot.ioc`
2. 检查本地 STM32H7 固件包版本是否兼容
3. 生成或刷新工程
4. 编译并下载到 `STM32H723VGT6`
5. 按实际接线确认 CAN、IMU、遥控接收器、总线舵机、RGB 灯工作正常

### 14.2. 机械臂描述

如需单独查看机械臂描述资源，可优先从以下目录入手：

- `chassis_control_code/arm_description/`
- `model/URDF_arm/ARM/`

---

## 15. 已知特点与局限

- 原型属性明显，机械与控制参数仍可能频繁调整
- 当前 README 以仓库已有代码与模型为准，不等同于完整 BOM 或接线手册
- 机械臂上层任务规划、视觉闭环与 ROS 总体系统尚未完成集成
- 遥控逻辑更偏调试/验证用途，正式任务系统仍需要进一步抽象
- 舵机串口读回目前在 `assemble_arm.c` 中尚未真正实现接收逻辑，后续可继续完善闭环能力

---

## 16. 后续开发方向

- 完善机械臂抓取器与末端执行器设计
- 增加完整电气连接图、BOM 与调参手册
- 补充底盘/机械臂联合测试流程
- 将 URDF、运动学与实际控制参数做更系统的一致性校准
- 接入 ROS 视觉、导航、任务规划与仿真链路
- 把 `Atlas` 与后续平台抽象到可复用的共性框架中

---

## 17. 仓库关系

`SteerWheel Mk.1` 是本仓库的第一代验证平台；后续的 `Atlas` 与另一台尚未命名的轮式机器人，会在这个原型的基础上继续演化：

- 底盘：继续采用舵轮方案
- 机械臂：继续采用五轴五自由度方案
- 软件：逐步向更清晰的模块复用与 ROS 上层融合演进

如果你要从本仓库开始接手开发，建议先熟悉 `SteerWheel Mk.1`，再迁移理解到后续车型
