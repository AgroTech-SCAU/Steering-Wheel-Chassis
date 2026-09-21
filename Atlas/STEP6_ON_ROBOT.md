# Step 6 集中真机验收

本包基于用户提供的 Step 1/2 完成包续作，包含 Step 3—5 源码和主机回归

当前尚未通过完整 MCU 编译、ROS 2 构建及真机验收，不能标记 Step 6 完成

## 1 编译和部署

MCU 与 Pi 必须配套更新，新 pitch/yaw 表示工具 z 轴方向，不能与旧 RPY 固件混用

先保存现场当前版本，在新目录解压本包，核对本地未提交修改后再替换对应源码

MCU 使用原有 EIDE 工程打开 `chassis_control_code/robot.code-workspace`，保持现有 GCC Cortex-M7 配置，执行完整 Rebuild，再烧录本次生成的固件

本包未包含旧 `build/Debug/robot.hex` 或 `robot.elf`，避免误烧旧版本

Pi 在新的 ROS 2 Humble 终端执行

```bash
cd Atlas/chassis-pi-ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to atlas_competition_bringup
source install/setup.bash
ros2 interface show mcu_comm_bridge/msg/ArmCommandResult
ros2 interface show vison_topic_interfaces/msg/PickTarget
ros2 interface show vison_topic_interfaces/msg/PickResult
```

确认 `PickTarget` 包含 `request_id` 和 `cancel`，`PickResult` 包含目标 XYZ 和方向角

所有使用 `vison_topic_interfaces` 的节点都需重新构建并重启，新增字段不保证与旧生成消息兼容

## 2 启动并记录

先用完整软件栈但关闭任务自动调度、导航和雷达，机械臂模式按原有操作进入 AutoPi

```bash
ros2 launch atlas_competition_bringup competition_stack.launch.py \
  enable_mission:=false enable_navigation:=false enable_lidar:=false
```

另开终端并加载同一工作空间

```bash
ros2 param get /atlas_competition_manipulation_backend view_scan.enabled
ros2 topic echo /mcu/arm_command_result
```

`view_scan.enabled` 应为 true，实际查询结果仍需现场确认

录包覆盖本次整个验收 session

```bash
ros2 bag record -o atlas_step6_acceptance \
  /arm/pose /arm/joint_states /mcu/arm_command_result \
  /pick_target /pick_result /vision_pose_ready /atlas/manipulation/status
```

## 3 逐点矩阵

`validation/step3_step5/step6_targets.yaml` 已列出 8 个 pickup 和 16 个 placement 目标，以及对应出发关节角

每个 pickup 先回该项 `seed_joints_rad`，再测试目标 XYZ，方向保持该观察位工具轴

每个 placement 先回相应 park prepare，再依次到目标上方 0.06 m、目标高度、目标上方 0.06 m，三段 pitch 均为 −π/2、yaw 为 0

这些是静态生成的测试目标，不是实际抓取定位结果，现场真实抓取仍由检测帧同步位姿计算

逐条调用下列服务，将花括号内数值替换为矩阵中的对应值，先等待上一条到位再发下一条

```bash
ros2 service call /mcu/set_arm_joints mcu_comm_bridge/srv/SetArmJoints \
  '{joints_rad: [q0, q1, q2, q3, q4], speed_rad_s: 0.3, suction_valid: false, suction_enable: false}'

ros2 service call /mcu/set_arm_pose mcu_comm_bridge/srv/SetArmPose \
  '{x_m: x, y_m: y, z_m: z, pitch_rad: pitch, yaw_rad: yaw, speed_rad_s: 0.3, suction_valid: false, suction_enable: false}'
```

上面是模板，`q0`、`x` 等必须替换为数值后执行

每条记录 service 返回的 `command_seq`、同 seq 的 MCU `result/arm_status`、到位 XYZ、工具轴和位置/姿态残差

`result=0` 表示 MCU 真正接受，不表示已经到位；必须结合后续新鲜反馈判断

姿态残差只比较工具 z 轴夹角，要求小于 3°，不约束末端自由自旋

反馈四元数为 `(qx,qy,qz,qw)` 时先归一化，再计算工具轴

```text
d_actual = [2(qx*qz+qw*qy), 2(qy*qz-qw*qx), 1-2(qx²+qy²)]
d_target = [cos(pitch)cos(yaw), cos(pitch)sin(yaw), sin(pitch)]
axis_error = acos(clamp(dot(d_actual,d_target),-1,1))
```

不得出现工具轴 180° 翻转，观察位 XYZ 正确但轴向错误时 `/vision_pose_ready` 必须为 false

## 4 失败与完整流程

在同一 session 中覆盖以下情况

- 不可达姿态返回 NO_SOLUTION，而不是只有服务 queued 成功
- 无检测、过期帧、位姿不同步、workspace 拒绝能在 `/pick_result` 看到明确 reason
- 观察位切换后不复用旧 best frame
- 抓取失败进入既有 deferred retry，而不是因错误 slot 进入全局 Recovery
- 取消预备抓取后不再自动发送后续 contact

`cancel` 仅取消未发送的后续抓取阶段，不是物理急停，已被 MCU 接受的动作仍需使用原有急停/模式控制处理

完成逐点验证后结束前一个 launch，启动原有全自主测试入口

```bash
ros2 launch atlas_competition_bringup auto_zone_test.launch.py
```

按照原有启动流程执行一次完整抓放链

观察 → 检测 → 抓取 → 吸附 1.5 s → 姿态保持抬升 → 搬运 → park prepare → 水平放置 → 释放 → 下一货物或 retry

验收前保持 `manual_offset_x_m=-0.055`、plane1/plane2 和各 slot 层高原值，本轮不凭现象重新标定来补偿运动学

请保留构建日志、录包、失败点名称与对应 command_seq，供核对结果
