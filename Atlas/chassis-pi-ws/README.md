# chassis-pi-ws

`chassis-pi-ws` 是运行在树莓派 5 上的 ROS2 工作区，主要负责完成 **ROS2 导航系统与底盘 MCU 之间的通信桥接**

---

## 1. 工作区定位

`chassis-pi-ws` 位于整车系统中的中间层：

```text
导航系统 / 状态机 / 上层任务
        ↓
chassis-pi-ws
        ↓
MCU 底盘控制程序
        ↓
电机 / 舵机 / IMU / 里程计
```

它主要解决两个问题：

1. **MCU 数据上行**
   将 MCU 发送的 IMU、里程计、状态信息解析后发布为 ROS2 话题，供导航系统使用

2. **ROS 控制下行**
   将导航系统或状态机发布的底盘速度指令转换为 MCU 协议帧，下发给 MCU 执行

---

## 2. 核心功能

### 2.1 串口通信

`chassis-pi-ws` 通过串口与 MCU 通信

当前主要支持：

```text
Pi <-> MCU
```

通信内容采用统一二进制帧协议，包含帧头、长度、协议版本、消息 ID、序号、标志位、payload 和 CRC 校验

该工作区负责：

* 打开串口设备
* 接收 MCU 二进制数据流
* 完成流式帧解析
* 校验 CRC
* 识别不同消息 ID
* 打包 Pi 发往 MCU 的控制帧
* 维护发送序号
* 避免多线程写串口时帧内容交叉

---

### 2.2 MCU 数据解析

MCU 会周期性向 Pi 发送底盘相关数据

`chassis-pi-ws` 当前主要解析以下 MCU 上行帧：

```text
MCU_IMU   0x25
MCU_ODOM  0x26
MCU_STATUS
MCU_START_SENSOR_EVENT
MCU_ACK
MCU_FAULT_EVENT
```

其中最关键的是：

```text
MCU_IMU
```

用于提供 IMU 数据，包括：

* 三轴加速度
* 三轴角速度
* roll / pitch / yaw
* 融合后的 yaw

```text
MCU_ODOM
```

用于提供底盘局部里程计数据，包括：

* x
* y
* yaw
* vx
* vy
* wz

其中 `yaw` 同时存在于 IMU 帧和 ODOM 帧中，目的为：

* IMU 帧中的 yaw 用于姿态话题
* ODOM 帧中的 yaw 用于完整构造底盘局部里程计

这样可以避免 Pi 端用“当前 odom 的 x/y + 最近一次 imu 的 yaw”临时拼接里程计，从而减少时间戳错位和语义混乱

---

## 3. 发布给导航系统的话题

`chassis-pi-ws` 会将 MCU 数据转换为导航系统可直接使用的 ROS2 标准话题

### 3.1 `/odom`

```text
话题名：/odom
类型：nav_msgs/msg/Odometry
来源：MCU_ODOM
```

`/odom` 表示底盘局部里程计，是导航系统的核心输入之一

它包含：

* 底盘在 odom 坐标系下的位置
* 底盘在 odom 坐标系下的 yaw
* 底盘在 base_footprint 坐标系下的速度
* 底盘角速度

该话题主要供以下模块使用：

* Nav2
* Cartographer
* 速度平滑器
* 状态机
* 调试与 rosbag 记录

---

### 3.2 `/imu`

```text
话题名：/imu
类型：sensor_msgs/msg/Imu
来源：MCU_IMU
```

`/imu` 表示 MCU 融合后的 IMU 姿态与惯性数据

它包含：

* orientation
* angular_velocity
* linear_acceleration

该话题主要用于：

* 查看车体姿态
* 辅助调试 yaw 漂移
* 后续扩展 robot_localization
* 后续扩展视觉系统或视觉惯性系统
* rosbag 数据记录

当前导航系统可以优先依赖 `/odom`，`/imu` 作为辅助数据保留

---

### 3.3 TF

`chassis-pi-ws` 可以根据 MCU_ODOM 发布：

```text
odom -> base_footprint
```

这与当前导航系统的 TF 链路保持一致：

```text
map -> odom -> base_footprint -> base_link -> laser_link
```

其中：

* `odom -> base_footprint` 由 Pi 端根据 MCU 里程计发布
* `base_footprint -> base_link` 由机器人模型或 robot_state_publisher 发布
* `base_link -> laser_link` 由机器人模型或静态 TF 发布

---

## 4. 订阅导航系统的控制话题

`chassis-pi-ws` 不直接订阅 Nav2 的原始 `/cmd_vel` 作为正式控制输入，而是订阅状态机仲裁后的最终底盘控制话题：

```text
/motor_cmd_vel
```

```text
话题名：/motor_cmd_vel
类型：geometry_msgs/msg/Twist
用途：底盘周期性速度控制
```

在当前导航系统中，典型链路是：

```text
Nav2
  ↓ /cmd_vel
competition_fsm
  ↓ /motor_cmd_vel
chassis-pi-ws
  ↓ PI_CONTROL
MCU
```

这样可以避免 Pi 端绕过状态机，直接执行 Nav2 的原始速度输出

---

## 5. 下发给 MCU 的控制帧

### 5.1 周期性底盘控制

`/motor_cmd_vel` 会被转换为 MCU 协议中的：

```text
PI_CONTROL 0x31
```

对应关系为：

```text
linear.x   -> vx
linear.y   -> vy
angular.z  -> wz
```

其中：

* `vx` 表示底盘 x 方向速度
* `vy` 表示底盘 y 方向速度
* `wz` 表示底盘绕 z 轴角速度

Pi 端不会在收到每一条 `/motor_cmd_vel` 时立即发送一帧，而是缓存最新速度指令，并以固定频率周期性下发给 MCU

这样做的原因是：

* 保持串口发送频率稳定
* 避免 ROS 话题频率波动直接影响 MCU
* 让 MCU 可以通过 fresh timeout 判断控制是否有效
* 符合高频连续控制量使用周期帧发送的设计

---

### 5.2 一次性底盘命令

除周期性速度控制外，底盘还存在一些一次性命令，例如：

* 刹车
* 急停
* yaw hold 开关
* 设置 yaw 目标角

这些命令不适合设计成周期性 topic，而更适合使用 service

因此 `chassis-pi-ws` 的设计原则是：

```text
周期性动作使用 topic
一次性动作使用 service
长期执行且需要反馈进度的动作才使用 action
```

目前底盘部分没有长期动作序列，因此暂时不需要 action

---

## 6. 底盘相关 service

### 6.1 `/mcu/set_brake`

```text
服务名：/mcu/set_brake
类型：std_srvs/srv/SetBool
作用：设置或解除底盘刹车请求
```

当请求为 `true` 时：

* Pi 端发送零速控制
* 设置 brake_request
* 后续周期控制保持刹车状态

当请求为 `false` 时：

* 解除刹车锁存
* 恢复 `/motor_cmd_vel` 控制

---

### 6.2 `/mcu/estop`

```text
服务名：/mcu/estop
类型：mcu_comm_bridge/srv/Estop
作用：向 MCU 发送急停事件
```

急停属于全局高优先级一次性事件

该服务会转换为：

```text
PI_ESTOP
```

MCU 收到急停后，应由 MCU 侧状态机进入急停状态

---

### 6.3 `/mcu/set_yaw_hold`

```text
服务名：/mcu/set_yaw_hold
类型：std_srvs/srv/SetBool
作用：开启或关闭 MCU 侧 yaw hold
```

该服务会转换为：

```text
PI_YAW_ACTION
```

其中：

* `true` 表示开启 yaw hold
* `false` 表示关闭 yaw hold

---

### 6.4 `/mcu/set_yaw_target`

```text
服务名：/mcu/set_yaw_target
类型：mcu_comm_bridge/srv/SetYawTarget
作用：设置 MCU 侧目标 yaw
```

该服务会将目标角转换为 MCU 协议中的定点角度单位，并通过：

```text
PI_YAW_ACTION
```

发送给 MCU

---

## 7. 使用前准备

### 7.1 系统环境

`chassis-pi-ws` 面向树莓派 5 和 ROS2 Humble，推荐环境为：

```text
硬件：Raspberry Pi 5
系统：Ubuntu 22.04
ROS：ROS2 Humble
通信：USB CDC 或 USB-TTL 串口
```

使用 USB 串口：

```text
STM32 USB CDC  -> /dev/ttyACM0
USB-TTL 模块   -> /dev/ttyUSB0
```

---

### 7.2 串口权限

树莓派用户需要加入 `dialout` 用户组，否则可能无法打开串口：

```bash
sudo usermod -aG dialout $USER
```

执行后需要重新登录或重启：

```bash
sudo reboot
```

检查当前用户是否已经属于 `dialout`：

```bash
groups
```

如果输出中包含：

```text
dialout
```

说明串口权限已经生效

---

### 7.3 固定串口设备名

如果 MCU 使用 STM32 USB CDC，设备一般是：

```text
/dev/ttyACM0
```

如果 MCU 通过 USB-TTL 模块连接，设备一般是：

```text
/dev/ttyUSB0
```

为了避免设备编号变化，建议配置 udev 规则

查看设备信息：

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct|serial" | head
```

或者：

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial" | head
```

创建 udev 规则：

```bash
sudo nano /etc/udev/rules.d/99-mcu-uart.rules
```

如果是 STM32 USB CDC，可以先使用：

```text
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="mcu_uart", GROUP="dialout", MODE="0660"
```

重新加载规则：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔 MCU 后检查：

```bash
ls -l /dev/mcu_uart
```

如果能看到 `/dev/mcu_uart`，说明固定设备名已经生效

---

## 8. 编译工作区

进入工作区根目录：

```bash
cd ~/chassis-pi-ws
```

安装依赖：

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

编译：

```bash
colcon build --symlink-install
```

加载环境：

```bash
source install/setup.bash
```

---

## 9. 配置参数

`mcu_comm_bridge` 的参数通常放在：

```text
src/mcu_comm_bridge/config/mcu_comm_bridge.yaml
```

常用参数包括：

```yaml
mcu_comm_bridge_node:
  ros__parameters:
    port: "/dev/mcu_uart"
    baudrate: 921600

    odom_topic: "/odom"
    imu_topic: "/imu"
    cmd_vel_topic: "/motor_cmd_vel"

    odom_frame_id: "odom"
    base_frame_id: "base_footprint"
    imu_frame_id: "imu_link"

    publish_tf: true

    heartbeat_rate_hz: 1.0
    control_rate_hz: 50.0
    cmd_vel_timeout_ms: 200

    max_vx_m_s: 1.5
    max_vy_m_s: 1.5
    max_wz_rad_s: 1.0
```

其中：

* `port` 是 Pi 与 MCU 通信的串口设备
* `baudrate` 必须与 MCU 端保持一致
* `odom_topic` 是发布给导航系统的里程计话题
* `imu_topic` 是发布给导航系统或调试工具的 IMU 话题
* `cmd_vel_topic` 是订阅的底盘控制话题，正式系统中应为 `/motor_cmd_vel`
* `publish_tf` 控制是否发布 `odom -> base_footprint`
* `control_rate_hz` 控制 `PI_CONTROL` 的周期发送频率
* `cmd_vel_timeout_ms` 控制速度命令超时保护时间

---

## 10. 启动节点

使用 launch 文件启动：

```bash
cd ~/chassis-pi-ws
source install/setup.bash
ros2 launch mcu_comm_bridge mcu_comm_bridge.launch.py
```

如果需要临时指定串口：

```bash
ros2 run mcu_comm_bridge mcu_comm_bridge_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=921600
```

如果没有启动 `competition_fsm`，只想单独调试底盘，也可以临时把输入话题改成 `/cmd_vel`：

```bash
ros2 run mcu_comm_bridge mcu_comm_bridge_node --ros-args \
  -p cmd_vel_topic:=/cmd_vel
```

正式接入导航系统时仍然建议使用：

```text
/motor_cmd_vel
```

---

## 11. 运行状态检查

### 11.1 检查节点

```bash
ros2 node list
```

正常情况下应看到类似：

```text
/mcu_comm_bridge_node
```

查看节点信息：

```bash
ros2 node info /mcu_comm_bridge_node
```

---

### 11.2 检查话题

查看话题列表：

```bash
ros2 topic list
```

正常情况下应至少包含：

```text
/odom
/imu
/motor_cmd_vel
```

检查 `/odom` 频率：

```bash
ros2 topic hz /odom
```

期望频率：

```text
约 50Hz
```

检查 `/imu` 频率：

```bash
ros2 topic hz /imu
```

期望频率：

```text
约 100Hz
```

查看 `/odom` 内容：

```bash
ros2 topic echo /odom
```

查看 `/imu` 内容：

```bash
ros2 topic echo /imu
```

---

### 11.3 检查 TF

检查 `odom -> base_footprint`：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

如果 TF 正常，应持续输出位姿变换

也可以查看完整 TF 树：

```bash
ros2 run tf2_tools view_frames
```

生成的 PDF 中应能看到：

```text
odom -> base_footprint -> base_link
```

其中 `base_footprint -> base_link` 通常由 robot_state_publisher 发布

---

### 11.4 检查串口数据

如果节点启动后没有 `/odom` 或 `/imu`，先确认串口是否有数据

使用 minicom：

```bash
sudo apt install minicom
minicom -b 921600 -o -D /dev/mcu_uart
```

由于 MCU 发送的是二进制帧，终端显示乱码是正常现象

重点是确认：

* 串口能打开
* 有连续数据流
* 波特率与 MCU 一致

---

## 12. 底盘控制测试

### 12.1 发布速度指令

正式系统中，速度指令应由 `competition_fsm` 发布到：

```text
/motor_cmd_vel
```

单独测试时可以手动发布：

```bash
ros2 topic pub /motor_cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

如果要测试横向速度：

```bash
ros2 topic pub /motor_cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

如果要测试旋转：

```bash
ros2 topic pub /motor_cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

节点会缓存最新速度，并按 `control_rate_hz` 周期性下发 `PI_CONTROL`

---

### 12.2 速度超时保护

如果 `/motor_cmd_vel` 超过 `cmd_vel_timeout_ms` 没有更新，节点会发送一次零速刹车帧

这样可以避免 ROS 侧控制中断后，MCU 继续执行旧速度

但最终是否进入刹车或停止状态，仍由 MCU 端状态机和控制逻辑决定

---

## 13. Service 使用方法

### 13.1 设置刹车

开启刹车：

```bash
ros2 service call /mcu/set_brake std_srvs/srv/SetBool "{data: true}"
```

解除刹车：

```bash
ros2 service call /mcu/set_brake std_srvs/srv/SetBool "{data: false}"
```

开启刹车后，节点会持续发送零速和 brake_request

解除刹车后，节点恢复使用 `/motor_cmd_vel` 的速度控制

---

### 13.2 急停

调用急停服务：

```bash
ros2 service call /mcu/estop mcu_comm_bridge/srv/Estop "{reason: 1}"
```

`reason` 用于标记急停原因，具体含义由 MCU 端定义

急停是一次性高优先级事件，节点收到服务请求后会立即发送 `PI_ESTOP`

---

### 13.3 yaw hold

开启 yaw hold：

```bash
ros2 service call /mcu/set_yaw_hold std_srvs/srv/SetBool "{data: true}"
```

关闭 yaw hold：

```bash
ros2 service call /mcu/set_yaw_hold std_srvs/srv/SetBool "{data: false}"
```

该服务会发送 `PI_YAW_ACTION`

---

### 13.4 设置 yaw 目标角

设置目标 yaw：

```bash
ros2 service call /mcu/set_yaw_target mcu_comm_bridge/srv/SetYawTarget "{yaw_rad: 1.57}"
```

其中：

```text
yaw_rad
```

单位为 rad

节点会将其转换为 MCU 协议中的 `urad` 定点整数后发送

---

## 14. 推荐联调顺序

建议按下面顺序联调，避免一开始同时排查导航、TF、串口和 MCU 状态机

### 14.1 只验证串口连接

目标：

```text
Pi 能打开串口
MCU 有连续数据输出
节点无串口打开失败日志
```

检查：

```bash
ls -l /dev/mcu_uart
ros2 launch mcu_comm_bridge mcu_comm_bridge.launch.py
```

---

### 14.2 验证 `/imu` 和 `/odom`

目标：

```text
/imu 约 100Hz
/odom 约 50Hz
```

检查：

```bash
ros2 topic hz /imu
ros2 topic hz /odom
```

---

### 14.3 验证 TF

目标：

```text
odom -> base_footprint 正常发布
base_footprint -> base_link 由机器人模型正常发布
```

检查：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

---

### 14.4 验证底盘速度下发

目标：

```text
发布 /motor_cmd_vel 后，MCU 能收到 PI_CONTROL
底盘只在 MCU 允许的自动状态下执行
```

检查：

```bash
ros2 topic pub /motor_cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

### 14.5 验证一次性 service

目标：

```text
刹车、急停、yaw hold、yaw target 能正确下发
```

检查：

```bash
ros2 service list | grep mcu
```

依次调用：

```bash
ros2 service call /mcu/set_brake std_srvs/srv/SetBool "{data: true}"
ros2 service call /mcu/set_brake std_srvs/srv/SetBool "{data: false}"
ros2 service call /mcu/set_yaw_hold std_srvs/srv/SetBool "{data: true}"
ros2 service call /mcu/set_yaw_target mcu_comm_bridge/srv/SetYawTarget "{yaw_rad: 0.0}"
ros2 service call /mcu/estop mcu_comm_bridge/srv/Estop "{reason: 1}"
```

---

### 14.6 接入完整导航系统

在确认底层桥接正常后，再启动完整导航系统

此时数据链路应为：

```text
MCU
  ↓
chassis-pi-ws
  ↓ /odom /imu
Cartographer / Nav2
```

控制链路应为：

```text
Nav2
  ↓ /cmd_vel
competition_fsm
  ↓ /motor_cmd_vel
chassis-pi-ws
  ↓ PI_CONTROL
MCU
```

---

## 15. 常见问题

### 15.1 找不到串口设备

检查：

```bash
ls -l /dev/ttyACM*
ls -l /dev/ttyUSB*
ls -l /dev/mcu_uart
```

如果没有 `/dev/mcu_uart`，说明 udev 规则没有生效或设备没有重新插拔

---

### 15.2 串口权限不足

如果日志中出现 permission denied，检查用户组：

```bash
groups
```

如果没有 `dialout`，执行：

```bash
sudo usermod -aG dialout $USER
sudo reboot
```

---

### 15.3 有串口数据但没有 `/odom` 或 `/imu`

可能原因：

* 波特率不一致
* MCU 仍然发送旧的 `0x20` 合并帧
* MCU 没有发送 `0x25` 和 `0x26`
* CRC 校验失败
* payload 长度和 Pi 端解析定义不一致

优先检查 MCU 当前是否发送：

```text
MCU_IMU   0x25
MCU_ODOM  0x26
```

---

### 15.4 `/odom` 有数据但 TF 不正常

检查参数：

```text
publish_tf
odom_frame_id
base_frame_id
```

推荐设置：

```yaml
publish_tf: true
odom_frame_id: "odom"
base_frame_id: "base_footprint"
```

同时确认 robot_state_publisher 正常发布：

```text
base_footprint -> base_link
```

---

### 15.5 发布 `/motor_cmd_vel` 后底盘不动

可能原因：

* MCU 当前不在 AutoPi 或自动控制状态
* Pi 没有成功发送 `PI_CONTROL`
* MCU 控制缓存 fresh timeout 过短
* 串口发送失败
* `/mcu/set_brake` 仍处于 true
* MCU 底盘控制层未 ready
* MCU 处于 fault 或 estop 状态

需要同时检查：

```bash
ros2 topic echo /motor_cmd_vel
ros2 node info /mcu_comm_bridge_node
```

以及 MCU 侧状态日志

---

### 15.6 急停后无法恢复

急停属于高优先级状态，是否能够恢复由 MCU 状态机决定

Pi 端 `/mcu/estop` 只负责发送急停事件，不负责解除急停

如果需要解除急停，应在 MCU 端设计明确的恢复条件或恢复服务，不建议让普通速度指令自动解除急停

