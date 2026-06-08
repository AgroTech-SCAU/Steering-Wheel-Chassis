# lslidar_msgs

> **纯 ROS2 接口定义包（`rosidl_interface_packages` 成员）**，定义了 LSLIDAR 激光雷达的 2 个自定义消息和 9 个自定义服务。只包含 `.msg` / `.srv` 文件和接口编译逻辑，不包含任何可执行节点。
>
> - **核心依赖**: `builtin_interfaces`, `rosidl_default_generators`
> - **下游消费者**: `lslidar_driver`

## 包概述

`lslidar_msgs` 是 `rosidl_interface_packages` 组成员包，遵循 ROS2 接口包标准结构。它的唯一职责是定义 `lslidar_driver` 与外部系统通信所需的消息和服务契约，编译后生成 C++ / Python 类型支持，供驱动包连接和订阅使用。

本包不发布话题、不提供服务——只定义接口。

## 包结构

```
lslidar_msgs/
├── msg/
│   ├── LslidarInformation.msg   # 雷达设备信息
│   └── LslidarPacket.msg        # 雷达原始数据包
├── srv/
│   ├── AngleDistortionCorrection.srv  # 角度畸变矫正 (1550系列)
│   ├── FrameRate.srv                  # 帧率设置 (1550系列)
│   ├── InvalidData.srv                # 无效数据发送控制 (1550系列)
│   ├── IpAndPort.srv                  # 网络参数配置
│   ├── MotorControl.srv              # 电机启停 (机械式)
│   ├── MotorSpeed.srv                # 转速设置 (机械式905)
│   ├── PowerControl.srv              # 上下电控制 (机械式)
│   ├── RfdRemoval.srv                # 去雨雾尘等级 (机械式)
│   ├── StandbyMode.srv               # 待机模式 (1550系列)
│   ├── TailRemoval.srv               # 去拖尾等级 (机械式)
│   └── TimeMode.srv                  # 授时方式设置
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 依赖

编译本包需要以下依赖（通常由 `lslidar_driver` 的 workspace 间接编译，无需单独处理）：

```bash
sudo apt install ros-$ROS_DISTRO-builtin-interfaces \
                 ros-$ROS_DISTRO-rosidl-default-generators
```

本包无运行时依赖（仅生成接口类型，不启动任何进程）。

## 消息定义

### LslidarInformation.msg

雷达设备信息，由 `lslidar_driver` 发布到 `lslidar_device_info` 话题。

| 字段名 | 类型 | 说明 |
|---|---|---|
| `lidar_ip` | `string` | 雷达 IP 地址 |
| `destination_ip` | `string` | 雷达目的 IP 地址 |
| `lidar_mac_address` | `string` | 雷达 MAC 地址 |
| `msop_port` | `uint16` | 雷达目的数据端口（源端口 2369） |
| `difop_port` | `uint16` | 雷达目的设备端口（源端口 2368） |
| `lidar_serial_number` | `string` | 雷达序列号 |
| `fpga_board_2_program` | `string` | FPGA 2 号板程序版本 |
| `fpga_board_3_program` | `string` | FPGA 3 号板程序版本 |

### LslidarPacket.msg

雷达原始数据包，用于内部数据流转。驱动通过 PCAP 或网络套接字读取原始 UDP 包后，封装为此消息再送入点云解析流水线。

| 字段名 | 类型 | 说明 |
|---|---|---|
| `stamp` | `builtin_interfaces/Time` | 数据包时间戳 |
| `data` | `uint8[1212]` | 原始数据载荷（固定 1212 字节） |

## 服务定义

所有服务遵循统一模式：请求携带操作参数，响应返回 `bool result` 表示操作是否成功。

---

### 一、网络配置

#### IpAndPort.srv — 设置雷达网络参数

| 参数 | 类型 | 说明 |
|---|---|---|
| `lidar_ip` | `string` | 雷达 IP 地址 |
| `destination_ip` | `string` | 雷达目的 IP 地址 |
| `data_port` | `uint16` | 数据端口 |
| `dev_port` | `uint16` | 设备端口 |

**更改后需重启驱动。**

```bash
ros2 service call /cx/network_setup lslidar_msgs/srv/IpAndPort \
  "{lidar_ip: '192.168.1.200', destination_ip: '192.168.1.102', data_port: 2368, dev_port: 2369}"
```

---

#### TimeMode.srv — 设置授时方式

| 参数 | 类型 | 说明 |
|---|---|---|
| `time_mode` | `uint8` | 0: GPS, 1: PTP_L2, 2: NTP, 3: PTP_UDPv4, 4: E2E_L2, 5: E2E_UDPv4 |
| `ntp_ip` | `string` | NTP 服务器 IP（仅 time_mode=2 时有效） |

```bash
# GPS 授时
ros2 service call /cx/time_mode lslidar_msgs/srv/TimeMode "{time_mode: 0, ntp_ip: ''}"

# NTP 授时
ros2 service call /cx/time_mode lslidar_msgs/srv/TimeMode "{time_mode: 2, ntp_ip: '192.168.1.102'}"
```

---

### 二、机械式雷达控制

#### PowerControl.srv — 上下电

| 参数 | 类型 | 说明 |
|---|---|---|
| `power_control` | `uint8` | 0: 下电, 1: 上电 |

```bash
ros2 service call /cx/power_control lslidar_msgs/srv/PowerControl "{power_control: 1}"
```

---

#### MotorControl.srv — 电机启停

| 参数 | 类型 | 说明 |
|---|---|---|
| `motor_control` | `uint8` | 0: 停转, 1: 旋转 |

```bash
ros2 service call /cx/motor_control lslidar_msgs/srv/MotorControl "{motor_control: 1}"
```

---

#### MotorSpeed.srv — 转速设置（机械式 905）

| 参数 | 类型 | 说明 |
|---|---|---|
| `motor_speed` | `uint8` | 5 / 10 / 20（Hz） |

```bash
ros2 service call /cx/motor_speed lslidar_msgs/srv/MotorSpeed "{motor_speed: 20}"
```

---

#### RfdRemoval.srv — 去雨雾尘等级

| 参数 | 类型 | 说明 |
|---|---|---|
| `rfd_removal` | `uint8` | 0-3，数值越大去除越强 |

```bash
ros2 service call /cx/remove_rain_fog_dust lslidar_msgs/srv/RfdRemoval "{rfd_removal: 3}"
```

---

#### TailRemoval.srv — 去拖尾等级

| 参数 | 类型 | 说明 |
|---|---|---|
| `tail_removal` | `uint8` | 新版本 0-10，旧版本 0-4；数值越大去除越强 |

```bash
ros2 service call /cx/tail_remove lslidar_msgs/srv/TailRemoval "{tail_removal: 4}"
```

---

### 三、1550 系列控制

#### AngleDistortionCorrection.srv — 角度畸变矫正

| 参数 | 类型 | 说明 |
|---|---|---|
| `angle_distortion_correction` | `uint8` | 0: 关闭, 1: 开启 |

开启可降低驱动算力消耗。**更改后需重启驱动。**

```bash
ros2 service call /ls/angle_distortion_correction lslidar_msgs/srv/AngleDistortionCorrection \
  "{angle_distortion_correction: 1}"
```

---

#### FrameRate.srv — 帧率设置

| 参数 | 类型 | 说明 |
|---|---|---|
| `frame_rate` | `uint8` | 0: 正常 10Hz, 1: 50% (5Hz), 2: 25% (2.5Hz) |

```bash
ros2 service call /ls/frame_rate lslidar_msgs/srv/FrameRate "{frame_rate: 1}"
```

---

#### InvalidData.srv — 无效数据发送控制

| 参数 | 类型 | 说明 |
|---|---|---|
| `invalid_data` | `uint8` | 0: 发送, 1: 不发送 |

不发送无效数据可降低 CPU 占用，但会导致点云时间不连续。

```bash
ros2 service call /ls/invalid_data lslidar_msgs/srv/InvalidData "{invalid_data: 1}"
```

---

#### StandbyMode.srv — 待机模式

| 参数 | 类型 | 说明 |
|---|---|---|
| `standby_mode` | `uint8` | 0: 正常模式, 1: 待机模式 |

```bash
ros2 service call /ls/standby_mode lslidar_msgs/srv/StandbyMode "{standby_mode: 1}"
```

---

## 运行方式

本包无可执行节点，编译后通过以下命令验证接口是否正确生成：

```bash
cd ~/AT_Atlas_nav_ws
colcon build --symlink-install --packages-select lslidar_msgs
source install/setup.bash

# 验证消息
ros2 interface show lslidar_msgs/msg/LslidarInformation
ros2 interface show lslidar_msgs/msg/LslidarPacket

# 验证服务
ros2 interface show lslidar_msgs/srv/IpAndPort
ros2 interface show lslidar_msgs/srv/TimeMode
```

## 与其他包的协作

```
+------------------+
|   lslidar_msgs   |  -- 定义接口 (.msg / .srv)
+--------+---------+
         |
         | rosidl 编译生成类型支持
         v
+--------+---------+
|  lslidar_driver   |  -- 实现驱动逻辑，发布 / 提供服务
+------------------+
         |
         | 话题: lslidar_point_cloud (PointCloud2)
         |        lslidar_device_info (LslidarInformation)
         |
         v
+--------+---------+
|  navigation 系统  |  -- 消费点云，用于定位、避障
+------------------+
```

`lslidar_msgs` -> `lslidar_driver`: 驱动包通过 CMakeLists.txt 中的 `<depend>lslidar_msgs</depend>` 声明依赖，编译时自动引入接口类型。

## 故障排查

| 现象 | 原因 | 解决 |
|---|---|---|
| `ros2 interface show` 报 "unknown package" | 未 source workspace | `source install/setup.bash` |
| `rosidl_generate_interfaces` 编译失败 | 缺少 `builtin_interfaces` | `sudo apt install ros-$ROS_DISTRO-builtin-interfaces` |
| 服务调用返回失败 | 雷达型号不支持该功能 | 确认型号匹配：机械式使用 `MotorControl` 等；1550 系列使用 `FrameRate` 等 |
| 驱动编译时找不到 `lslidar_msgs` | 未先编译接口包 | 先 `colcon build --packages-select lslidar_msgs`，再编译 `lslidar_driver` |
