# MCU 控制底座说明与 TODO

## 1. 项目定位

本项目当前 MCU 端不再绑定某一场具体比赛流程，而是作为整车控制底座使用

MCU 端主要负责：

- 系统模式状态机
- 控制权仲裁
- 底盘闭环执行
- 机械臂底层执行
- 遥控接管
- PC 上位机通信
- 树莓派通信
- 故障锁存与安全收口
- 状态显示与低频日志

树莓派端主要负责：

- 自动任务流程
- 导航
- 视觉识别
- 地图更新
- 自动任务决策
- 向 MCU 下发底盘、yaw、机械臂命令

PC 端主要负责：

- 调试命令
- 主臂关节角输入
- 急停指令
- 串口日志查看

FS-iA10B 主要负责：

- 人工遥控接管
- 手动底盘控制
- 手动机械臂控制

------

## 2. 当前主架构

```text
entry
  ↓
remote / pc_comms / pi_comms / chassis / odom / arm
  ↓
app_runtime
  ↓
app_fsm
  ↓
app_control
  ↓
chassis / arm
```

各模块职责：

```text
entry.h
  系统初始化顺序
  主循环调度表

remote
  FS-iA10B 遥控输入解析

pc_comms
  USART1 PC 通信
  接收 PC 调试命令
  接收 PC 主臂关节角
  保留 USART1 log 发送能力

pi_comms
  USART10 树莓派通信
  发送 IMU / odom / MCU_STATUS
  接收底盘 / yaw / 机械臂 / 急停 / 任务结果命令

app_runtime
  控制权仲裁
  模式切换
  安全检查
  控制结果处理

app_fsm
  MCU 系统级状态机

app_control
  将 Remote / PC / Pi 命令统一转换为 chassis / arm 执行请求

app_status
  LED 状态
  heartbeat
  低频调试日志

chassis
  底盘闭环
  速度控制
  brake
  yaw hold

odom
  IMU + 底盘里程计融合

arm
  机械臂关节控制
  反馈刷新
  动作执行
```

------

## 3. 主循环调度

当前 `entry_loop()` 应保持清晰调度表风格：

```text
500Hz:
  chassis.process()
  app_runtime_process()

250Hz:
  odom.process()

100Hz:
  remote_process()
  pc_comms_process()
  pi_comms_process()

50Hz:
  arm.refresh_current_state()

background:
  app_status_process()
```

原则：

- `entry.h` 只负责调度
- 不在 `entry.h` 中堆业务逻辑
- 控制权仲裁统一放在 `app_runtime`
- 具体执行映射统一放在 `app_control`

------

## 4. MCU 系统状态机

当前 `app_fsm` 是基于 HFSM 框架实现的系统级状态机

当前实现更接近平铺 FSM：

```text
Idle
Manual
AutoPi
Fault
EStop
Finished
```

语义上分为三类：

```text
Operational:
  Idle
  Manual
  AutoPi
  Finished

Fault:
  故障锁存状态
  可通过 CLEAR_FAULT 恢复

EStop:
  急停锁死状态
  普通 START / STOP / CLEAR_FAULT 不可恢复
```

控制权优先级：

```text
EStop > Fault > Manual > AutoPi > Idle
```

------

## 5. 控制模式说明

### 5.1 Idle

```text
所有运动命令无效
底盘 brake
PC 主臂命令忽略
Pi 命令忽略
```

### 5.2 ManualChassisPcArm

```text
FS-iA10B 控制底盘
PC 主臂关节角控制机械臂
Pi 命令无效
PC 主臂数据过期后停止更新机械臂目标
```

### 5.3 ManualArmFs

```text
FS-iA10B 控制机械臂
底盘 brake
PC 主臂命令无效
Pi 命令无效
```

### 5.4 AutoPi

```text
Pi 控制底盘 / yaw / 机械臂
FS 手动输出无效
PC 主臂输出无效
Pi 掉线或命令过期触发安全策略
所有 Pi 指令必须限幅和合法性检查
```

### 5.5 Fault

```text
所有运动命令无效
底盘 brake
机械臂 stop 或保持安全状态
只能 CLEAR_FAULT 恢复到 Idle
不能 START 直接恢复
```

### 5.6 EStop

```text
所有运动命令无效
底盘 brake
机械臂 stop
不能被普通 START / CLEAR_FAULT 恢复
```

------

## 6. 当前 PC 协议

PC 通过 USART1 与 MCU 通信

当前 ASCII 联调协议示例：

```text
PING
CMD:START
CMD:STOP
CMD:CLEAR_FAULT
CMD:BRAKE
CMD:ARM_ENABLE
CMD:ARM_STOP
CMD:ESTOP
JOINT:q0,q1,q2,q3,q4
```

说明：

- `CMD:ESTOP` 触发 MCU 进入 EStop
- `CMD:ARM_ENABLE` 不能在 Fault / EStop / AutoPi 下执行
- `CMD:ARM_STOP` 作为安全命令，可用于停止机械臂
- `JOINT` 只在 `ManualChassisPcArm` 下生效
- PC 主臂关节角必须通过 finite 检查和机械臂关节限位检查

------

## 7. 当前 Pi 协议

Pi 通过 USART10 与 MCU 通信

Pi -> MCU 当前 ASCII 联调协议示例：

```text
PING
ESTOP
MISSION:DONE
MISSION:FAIL,code
CHASSIS:vx,vy,wz
YAW:HOLD_ENABLE
YAW:HOLD_DISABLE
YAW:TARGET,value
YAW:RATE,value
ARM:STOP
ARM:ENABLE
ARM:SEQ,id
ARM:JOINT,q0,q1,q2,q3,q4,speed
```

MCU -> Pi 当前 ASCII 状态帧示例：

```text
IMU_ODOM:stamp_ms,roll,pitch,yaw,gyro_z,odom_x,odom_y,odom_yaw
MCU_STATUS:stamp_ms,state,manual_mode,chassis_ready,arm_ready,odom_ready,remote_online,pc_online,pi_online,fault_latched,fault_source,fault_level,fault_code
```

协议语义：

- `ESTOP` 触发 MCU 进入 EStop
- `MISSION:DONE` 仅在 AutoPi 下生效，MCU 停止运动并进入 Finished
- `MISSION:FAIL,code` 仅在 AutoPi 下生效，MCU 进入 Fault
- `CHASSIS:vx,vy,wz` 仅在 AutoPi 下生效
- `YAW:HOLD_ENABLE / HOLD_DISABLE / TARGET` 走 yaw hold
- `YAW:RATE` 表示 Pi 直接给 yaw rate
- `YAW:RATE` 新鲜时覆盖 `CHASSIS` 中的 `wz`
- `ARM:JOINT` 需要通过关节限位和速度检查
- `ARM:SEQ` 当前仅完成协议入口，尚未实现本地动作序列执行器

------

## 8. 已完成的关键修复

- MCU 主流程已从具体比赛任务状态机转为系统级状态机
- `Navigate / Pollinate / ReturnHome` 已退出 MCU 主链路
- `entry.h` 已整理为清晰调度表
- `remote` 已变成纯遥控输入解析服务
- `pc_comms` 已支持 PC 命令和主臂关节角输入
- `pi_comms` 已支持树莓派底盘 / yaw / 机械臂命令
- `Pi ESTOP` 已接入
- `Pi MISSION:DONE / FAIL` 已接入
- `MCU_STATUS` 已包含状态机和故障信息
- `Fault` 状态不能被普通 START / SWITCH_TO_AUTO_PI 打破
- `EStop` 状态不能被普通 START / CLEAR_FAULT 恢复
- `app_fsm_raise_fault()` 已采用高优先级 fault 事件处理
- PC `ARM_ENABLE` 已限制不能在 Fault / EStop / AutoPi 下执行
- `app_control` 已引入 `AppControlResult`
- PC / Pi 外部机械臂关节角已加入限位检查
- `YAW:RATE` 覆盖 `wz` 的语义已明确
- `ARM:SEQ` 未实现时已显式 warning / unsupported
- LED 优先级已调整为 `EStop > Fault > Manual > AutoPi > NotReady > Ready`
- Heartbeat log 已包含 fault source / level / code

------

# 9. TODO

## P0 近期必须处理

### TODO 1：修复残留中文注释乱码

检查全项目是否仍有类似乱码：

```text
瀹?瀹?涔?
鍙?閲?澹?
绉?鏈?鍑?鏁?
```

统一修复为：

```c
// ! ========================= 宏 定 义 ========================= ! //
// ! ========================= 类 型 定 义 ========================= ! //
// ! ========================= 变 量 声 明 ========================= ! //
// ! ========================= 私 有 函 数 声 明 ========================= ! //
// ! ========================= 接 口 函 数 实 现 ========================= ! //
// ! ========================= 私 有 函 数 实 现 ========================= ! //
```

要求：

- 保持 UTF-8 编码
- 不改动无关逻辑

------

### TODO 2：提升所有 ESTOP 的处理优先级

当前 Pi ESTOP 已优先处理，PC ESTOP 也已接入

建议进一步统一为：

```text
1. Pi ESTOP
2. PC ESTOP
3. 硬件 ESTOP
4. 遥控 ESTOP
5. Pi MISSION event
6. 其他 PC command
7. remote mode
```

目标：

- 任意 ESTOP 都应优先于 MISSION:DONE / FAIL
- 任意 ESTOP 都应优先于 START / STOP / CLEAR_FAULT
- EStop 后立即 `app_control_stop_all()`

------

### TODO 3：补充硬件急停或遥控急停入口

当前急停主要来自 PC 和 Pi

建议新增至少一种现场急停方式：

```text
硬件 GPIO 急停
或
FS-iA10B 组合开关急停
```

建议优先级：

```text
硬件急停 > 遥控急停 > Pi 急停 > PC 急停
```

原因：

- 比赛现场不应只依赖 PC / Pi 急停
- MCU 必须保留本地最高安全入口

------

### TODO 4：Unsupported 命令日志节流或清除

当前 `ARM:SEQ` 未实现时会返回 `APP_CONTROL_RESULT_UNSUPPORTED`

需要检查是否在 freshness 时间窗口内反复 warning

建议：

```text
收到 unsupported 后清除该 Pi arm command
或
对 unsupported warning 做 1000ms 节流
```

目标：

- 不刷屏
- 保留足够调试信息
- 不让树莓派误以为命令执行成功

------

### TODO 5：明确 Manual 下执行失败策略

当前策略大致为：

```text
ManualChassisPcArm:
  底盘执行失败 -> Fault
  PC 主臂失败 -> stop_arm + warning

ManualArmFs:
  机械臂执行失败 -> Fault
```

仍需根据比赛安全策略确认：

```text
PC 主臂执行失败是否需要 Fault
Manual 下 arm not ready 是否直接 Fault
Manual 下 chassis brake 失败是否立即 Fault
```

建议：

```text
底盘执行失败必须 Fault
机械臂遥控执行失败建议 Fault
PC 主臂单次非法命令只 warning，不 Fault
```

------

## P1 功能补齐

### TODO 6：实现本地 arm_sequence 执行器

当前 Pi 协议已有：

```text
ARM:SEQ,id
```

但 MCU 端尚未真正实现动作序列执行

建议新增能力：

```text
arm_sequence_load(sequence_id)
arm_sequence_process()
arm_sequence_cancel()
arm_sequence_is_running()
arm_sequence_get_result()
```

用途：

- 固定机械臂动作由 MCU 本地执行
- Pi 只需要发送 sequence_id
- 减少 Pi 连续下发关节角导致的通信压力
- 提高固定动作可靠性

建议状态：

```text
IDLE
RUNNING
DONE
FAILED
CANCELED
```

------

### TODO 7：Pi 增加任务暂停 / 继续协议

当前 Pi 支持：

```text
MISSION:DONE
MISSION:FAIL,code
```

后续可增加：

```text
MISSION:PAUSE
MISSION:RESUME
MISSION:CANCEL
```

用途：

- AutoPi 中临时暂停自动任务
- 遥控接管后暂停 Pi 自动逻辑
- 故障恢复后由 Pi 决定是否继续任务

------

### TODO 8：Pi 增加自身状态上报

当前 Pi 主要下发控制命令，建议后续增加 Pi 状态：

```text
PI_STATUS:stamp,mission_state,nav_state,vision_state,error_code
```

用途：

- MCU 或 PC 端能看到 Pi 当前自动任务阶段
- 便于现场 debug
- 便于判断 Pi 是否卡在导航、视觉或任务逻辑中

------

### TODO 9：完善 `MCU_STATUS` 协议文档

当前 `MCU_STATUS` 字段较多，建议固定文档：

```text
字段 0: stamp_ms
字段 1: app_state
字段 2: manual_mode
字段 3: chassis_ready
字段 4: arm_ready
字段 5: odom_ready
字段 6: remote_online
字段 7: pc_online
字段 8: pi_online
字段 9: fault_latched
字段 10: fault_source
字段 11: fault_level
字段 12: fault_code
```

要求：

- Pi 端解析时严格按字段顺序
- 后续新增字段要考虑版本号
- 建议加入 `protocol_version`

------

## P2 通信可靠性升级

### TODO 10：PC / Pi ASCII 协议升级为二进制帧

当前 ASCII 协议用于联调，比赛稳定版建议升级为：

```text
frame_header
length
message_type
sequence_id
payload
crc16
```

目标：

- 防止串口丢字节导致误解析
- 支持 payload 长度检查
- 支持 CRC 校验
- 支持消息序号
- 支持 ACK / NACK

------

### TODO 11：USART RX 改为 DMA ring / IDLE 中断

当前接收方式适合联调，后续建议升级为：

```text
DMA circular buffer
IDLE line interrupt
ring buffer parser
```

目标：

- 降低中断频率
- 提高高频通信稳定性
- 避免单字节中断导致 CPU 开销过大

------

### TODO 12：USART TX 改为非阻塞 DMA

当前如果存在 blocking write，后续建议升级为：

```text
TX DMA
发送队列
非阻塞发送
```

目标：

- 避免日志或状态帧阻塞控制周期
- 避免 100Hz 通信影响 500Hz 控制
- 提升 PC / Pi 通信稳定性

------

## P3 架构纯度优化

### TODO 13：降低 `pi_comms` 对 app 层的依赖

当前 `pi_comms` 可能直接依赖：

```text
app_runtime
chassis
arm
odom
remote
pc_comms
```

用于组装 MCU 状态帧

长期更干净的结构是：

```text
app_runtime / app_status 组装 McuStatusFrame
pi_comms 只负责发送 frame
```

建议后续改为：

```c
typedef struct {
    uint32_t stamp_ms;
    uint8_t app_state;
    uint8_t manual_mode;
    uint8_t chassis_ready;
    uint8_t arm_ready;
    uint8_t odom_ready;
    uint8_t remote_online;
    uint8_t pc_online;
    uint8_t pi_online;
    uint8_t fault_latched;
    uint8_t fault_source;
    uint8_t fault_level;
    int32_t fault_code;
} McuStatusFrame;

void pi_comms_send_mcu_status(const McuStatusFrame* frame);
```

目标：

- service 层不反向依赖 app 层
- pi_comms 更像纯通信模块
- 状态组装逻辑更清楚

------

### TODO 14：根据体量拆分 `app_control.c`

当前 `app_control.c` 同时包含：

```text
remote manual control
pc master arm control
pi auto control
yaw control
safety stop
validation
```

当前可以接受，但后续如果继续增长，建议拆分：

```text
app_control.c
app_control_remote.c
app_control_pc.c
app_control_pi.c
app_control_safety.c
```

原则：

- 不改变对外接口
- 只拆内部实现
- 保持 `app_runtime` 仍然只调用统一的 `app_control` 接口

------

### TODO 15：状态复杂后再考虑真正 HFSM 层级化

当前 `app_fsm` 是平铺系统状态机，语义上区分：

```text
Operational
Fault
EStop
```

当前状态数量少，平铺实现足够

如果未来状态增加，例如：

```text
Manual
  ManualChassisPcArm
  ManualArmFs

AutoPi
  AutoPiRunning
  AutoPiPaused
  AutoPiWaitingPi
```

再考虑真正使用 HFSM 层级结构：

```text
Root
  Operational
    Idle
    Manual
      ManualChassisPcArm
      ManualArmFs
    AutoPi
      AutoPiRunning
      AutoPiPaused
    Finished
  Fault
  EStop
```

当前不建议为了形式强行层级化

------

## 10. 当前不建议做的事

- 不建议继续大改 MCU 主架构
- 不建议恢复旧比赛任务流程
- 不建议让 MCU 重新管理完整自动任务
- 不建议让 Pi / PC / Remote 直接调用 chassis 或 arm
- 不建议现在强行把所有文件迁移到多级目录
- 不建议现在强行实现复杂任务脚本解释器
- 不建议让 `Fault / EStop` 被普通事件恢复
- 不建议让 `ARM:SEQ` 静默成功

------

## 11. 下一阶段建议

短期建议：

```text
1. 修复所有残留编码乱码
2. 补硬件或遥控急停
3. 修复 unsupported warning 刷屏问题
4. 根据实际比赛确定 Manual 错误策略
5. 开始和树莓派联调 AutoPi
```

中期建议：

```text
1. 实现 arm_sequence
2. 固定 Pi/MCU 协议文档
3. 增加 Pi 状态上报
4. 完善协议版本号
```

长期建议：

```text
1. 升级二进制协议 + CRC
2. USART TX/RX DMA 化
3. 降低 pi_comms 对 app 层依赖
4. 根据状态复杂度决定是否真正层级化 app_fsm
```

------

## 12. 当前版本评价

当前 MCU 端已经可以作为后续比赛的通用控制底座使用

后续主要工作应集中在：

```text
Pi 自动任务状态机
Pi 导航视觉
Pi/MCU 协议稳定性
机械臂动作序列执行
现场急停安全链路
```

不建议再大改 MCU 主架构
