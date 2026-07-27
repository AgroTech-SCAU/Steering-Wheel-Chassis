# atlas_nav_pseudo_backend 说明

本包是基于 mcu 里程计的伪导航后端

它不使用地图，不避障，不做全局路径规划

它只接收一个任务相对点，转换为 odom 目标点，然后输出底盘机体系速度

## 一，包定位

负责

```text
提供 /atlas/navigation/start
提供 /atlas/navigation/cancel
发布 /atlas/navigation/status
发布 /atlas/navigation/cmd_vel
订阅 /odom
记录任务原点
把任务相对坐标转换为 odom 坐标
根据 odom 误差计算 vx，vy，wz
判断到点和稳定
处理 odom 超时和 waypoint 超时
```

不负责

```text
不发布 /motor_cmd_vel
不直接请求 mcu 刹车
不判断 mcu 是否 AutoPi
不执行视觉和机械臂任务
不处理 DONE 或 FAIL 上报
```

## 二，服务接口

### /atlas/navigation/start

类型

```text
atlas_mission_interfaces/srv/StartNavigation
```

请求字段

| 字段 | 说明 |
|---|---|
| `backend` | 必须为空或 pseudo |
| `waypoint_id` | 当前点位编号 |
| `x_m` | 任务相对 x，单位米 |
| `y_m` | 任务相对 y，单位米 |
| `yaw_rad` | 任务相对偏航角，单位弧度 |
| `reset_origin` | 是否用当前 /odom 作为任务原点 |
| `timeout_s` | 该点超时时间 |

### /atlas/navigation/cancel

类型

```text
atlas_mission_interfaces/srv/CancelNavigation
```

用途

```text
取消当前移动并发布零速
```

## 三，话题接口

### /odom

输入里程计

来源是 `mcu_comm_bridge`

### /atlas/navigation/cmd_vel

输出速度

类型

```text
geometry_msgs/msg/Twist
```

速度含义

```text
linear.x 为底盘前后速度
linear.y 为底盘左右速度
angular.z 为底盘旋转速度
```

### /atlas/navigation/status

输出导航状态

类型

```text
atlas_mission_interfaces/msg/NavigationStatus
```

## 四，坐标规则

任务开始时，如果 `reset_origin=true`，本节点记录当前 odom 为任务原点

任务点位使用任务相对坐标

```text
x_m，y_m，yaw_rad
```

转换方式

```text
目标 odom x = origin_x + cos(origin_yaw) * x_m - sin(origin_yaw) * y_m
目标 odom y = origin_y + sin(origin_yaw) * x_m + cos(origin_yaw) * y_m
目标 odom yaw = origin_yaw + yaw_rad
```

控制器会把 odom 坐标误差转换到机器人机体系

```text
ex_body = cos(yaw) * dx + sin(yaw) * dy
ey_body = -sin(yaw) * dx + cos(yaw) * dy
```

## 五，控制流程

```text
收到 start 请求
  ↓
检查 /odom 是否新鲜
  ↓
必要时记录任务原点
  ↓
计算目标 odom 坐标
  ↓
进入 RUNNING
  ↓
周期计算位置误差和偏航误差
  ↓
输出速度
  ↓
误差进入阈值后输出零速
  ↓
速度稳定持续 settle_duration_s
  ↓
发布 SUCCEEDED
```

失败条件

```text
没有新鲜 /odom
移动超时
内部目标缺失
收到 cancel
```

## 六，配置文件

配置文件

```text
config/pseudo_nav.yaml
```

重点参数

| 参数 | 说明 |
|---|---|
| `control_rate_hz` | 控制循环频率 |
| `odom_timeout_s` | odom 最大允许延迟 |
| `kp_xy` | 平面位置比例系数 |
| `kp_yaw` | 偏航比例系数 |
| `max_linear_speed_m_s` | 最大平移速度 |
| `max_angular_speed_rad_s` | 最大角速度 |
| `max_linear_accel_m_s2` | 最大线加速度 |
| `max_angular_accel_rad_s2` | 最大角加速度 |
| `slowdown_distance_m` | 开始减速的距离 |
| `position_tolerance_m` | 到点位置阈值 |
| `yaw_tolerance_rad` | 到点角度阈值 |
| `settle_duration_s` | 稳定保持时间 |
| `settle_timeout_s` | 到点后等待稳定的最大时间 |

## 七，启动

```bash
ros2 launch atlas_nav_pseudo_backend pseudo_nav.launch.py
```

替换配置

```bash
ros2 launch atlas_nav_pseudo_backend pseudo_nav.launch.py \
  config:=/home/wheeltec/my_config/pseudo_nav.yaml
```

## 八，单独测试

启动 mcu 通信桥和伪导航后端后，可以手动调用

```bash
ros2 service call /atlas/navigation/start atlas_mission_interfaces/srv/StartNavigation \
"{backend: 'pseudo', waypoint_id: 'test_01', x_m: 0.0, y_m: -0.07, yaw_rad: 0.0, reset_origin: true, timeout_s: 20.0}"
```

查看状态

```bash
ros2 topic echo /atlas/navigation/status
```

取消

```bash
ros2 service call /atlas/navigation/cancel atlas_mission_interfaces/srv/CancelNavigation \
"{reason: 'manual test cancel'}"
```

注意

```text
单独测试时伪导航只发布 /atlas/navigation/cmd_vel
不会直接驱动底盘
要让底盘真实运动，需要通过 atlas_mission_manager 的安全门控转发到 /motor_cmd_vel
```
