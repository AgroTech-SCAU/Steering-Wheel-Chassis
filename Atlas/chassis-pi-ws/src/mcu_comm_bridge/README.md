# mcu_comm_bridge

`mcu_comm_bridge` 负责在 ROS 2 与 Atlas MCU 二进制协议之间做桥接

## 功能

- 解析 `MCU_IMU / MCU_ODOM / MCU_ARM_STATE / MCU_STATUS`
- 发布 `/imu`、`/odom`、`/arm/joint_states`、`/arm/fk_position`
- 订阅 `/motor_cmd_vel`，以 `50Hz` 连续发送 `PI_CONTROL`
- 提供刹车、急停、yaw、机械臂目标等 ROS 2 服务
- 对机械臂目标做本地排队与有限重发

## PI_CONTROL 发送语义

`PI_CONTROL` 仍固定为 38 字节，bridge 每个控制周期会按需组合发送：

- 底盘目标
- 机械臂目标
- 刹车标志

允许的组合：

- `chassis-only`
- `arm-only`
- `chassis + arm`

如果同一时刻两者都没有待发送内容，则不发 `PI_CONTROL`

## 机械臂服务

新增服务：

- `/mcu/set_arm_joints`
- `/mcu/set_arm_pose`
- `/mcu/set_arm_position`
- `/mcu/set_arm_orientation`

返回 `success=true` 只表示：

```text
arm command queued for transmission
```

不表示 MCU 已接收或已执行成功

### 服务示例

```bash
ros2 service call /mcu/set_arm_joints mcu_comm_bridge/srv/SetArmJoints \
  "{joints_rad: [0.0, 0.2, -0.3, 0.1, 0.0], speed_rad_s: 1.5}"
```

```bash
ros2 service call /mcu/set_arm_pose mcu_comm_bridge/srv/SetArmPose \
  "{x_m: 0.35, y_m: 0.00, z_m: 0.22, pitch_rad: 0.30, yaw_rad: 0.00, speed_rad_s: 1.2}"
```

```bash
ros2 service call /mcu/set_arm_position mcu_comm_bridge/srv/SetArmPosition \
  "{x_m: 0.30, y_m: 0.05, z_m: 0.18, speed_rad_s: 1.0}"
```

```bash
ros2 service call /mcu/set_arm_orientation mcu_comm_bridge/srv/SetArmOrientation \
  "{pitch_rad: 0.20, yaw_rad: -0.10, speed_rad_s: 0.8}"
```

## 重发策略

- 参数：`arm_command_repeat_count`
- 默认值：`3`
- 同一条机械臂命令的所有重发帧共享同一个 `arm_command_seq`
- 只有串口写成功后才减少剩余重发次数
- 新机械臂服务请求会覆盖尚未发完的旧命令

## 主要参数

```yaml
mcu_comm_bridge_node:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baudrate: 1000000
    cmd_vel_topic: "/motor_cmd_vel"
    control_rate_hz: 50.0
    cmd_vel_timeout_ms: 200
    arm_command_repeat_count: 3
    brake_service: "/mcu/set_brake"
    arm_joints_service: "/mcu/set_arm_joints"
    arm_pose_service: "/mcu/set_arm_pose"
    arm_position_service: "/mcu/set_arm_position"
    arm_orientation_service: "/mcu/set_arm_orientation"
    yaw_hold_service: "/mcu/set_yaw_hold"
    yaw_target_service: "/mcu/set_yaw_target"
    estop_service: "/mcu/estop"
```

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mcu_comm_bridge
colcon test --packages-select mcu_comm_bridge
colcon test-result --verbose
```

## 说明

- `speed_rad_s == 0` 表示使用 MCU 默认速度
- 负速度会在服务层直接拒绝
- `ARM_VALID=0` 的语义是不更新机械臂目标，不是停止
- 停止机械臂仍通过 `PI_ARM_ACTION_STOP`
- 机械臂命令只允许在 MCU 的 `AutoPi` 模式中执行
