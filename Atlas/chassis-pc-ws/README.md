# Dual DM Arm 独立遥操作脚本说明

## 1. 项目目标

本目录中的 `teleop_comments_zh_v3.py` 用于把原双臂工程中的主从映射核心逻辑独立出来

它完成的事情是：

1. 打开主臂 Dynamixel 串口，默认 `/dev/ttyUSB0`
2. 持续读取主臂关节角
3. 读取 ID7 作为末端开关输入
4. 将主臂 `q0 ~ q4` 与末端开关状态打包成 PC 到 MCU 的二进制协议帧
5. 打开 MCU 串口，默认 `/dev/ttyUSB1`
6. 按固定频率持续发送 `PC_MASTER_JOINTS`
7. 按低频持续发送 `PC_HEARTBEAT`

这个版本不依赖 LeRobot，只依赖：

```bash
pyserial
dynamixel-sdk
crcmod
```

适用环境：

```text
Ubuntu 22.04
Python 3.10+
Dynamixel Protocol 2.0
```

---

## 2. 文件说明

```text
teleop_comments_zh_v3.py
```

单文件脚本，包含：

1. 用户配置区
2. Dynamixel 主臂读取
3. ID7 末端开关配置与读取
4. CRC16-CCITT 校验
5. PC 到 MCU 二进制协议打包
6. 串口发送主循环

---

## 3. 硬件连接约定

### 3.1 主臂 Dynamixel 串口

默认端口：

```text
/dev/ttyUSB0
```

默认波特率：

```text
115200
```

默认 ID 映射：

| 机械含义 | Dynamixel ID | 发送字段 |
|---|---:|---|
| 主臂关节 0 | 1 | `q0_urad` |
| 主臂关节 1 | 2 | `q1_urad` |
| 主臂关节 2 | 3 | `q2_urad` |
| 主臂关节 3 | 4 | `q3_urad` |
| 主臂关节 4 | 5 | `q4_urad` |
| 末端开关 / gripper | 7 | `end_switch` |

当前协议只发送 `q0 ~ q4` 五个关节角，不发送 ID6

### 3.2 MCU 串口

默认端口：

```text
/dev/ttyUSB1
```

默认波特率：

```text
115200
```

串口格式：

```text
8 数据位
无校验
1 停止位
```

---

## 4. 安装依赖

建议先创建虚拟环境：

```bash
python3 -m venv .venv
source .venv/bin/activate
```

安装依赖：

```bash
pip install pyserial dynamixel-sdk crcmod
```

将当前用户加入串口权限组：

```bash
sudo usermod -aG dialout $USER
```

执行后需要重新登录终端，或者重启系统

---

## 5. 快速运行

普通运行：

```bash
python3 teleop_comments_zh_v3.py \
  --leader-port /dev/ttyUSB0 \
  --mcu-port /dev/ttyUSB1 \
  --freq 50
```

只读取主臂、不发送 MCU，用于调试：

```bash
python3 teleop_comments_zh_v3.py \
  --leader-port /dev/ttyUSB0 \
  --dry-run \
  --print-rate 10
```

运行后调试输出类似：

```text
q_rad=[+0.0123, -0.0456, +0.0789, +0.1000, -0.2000] gripper_raw=1900 gripper_norm=+0.623 end_switch=1 frames=50 heartbeat=1 errors=0
```

---

## 6. 配置区说明

`teleop_comments_zh_v3.py` 文件开头提供了完整配置区，优先修改这里

### 6.1 串口配置

```python
DEFAULT_LEADER_PORT = "/dev/ttyUSB0"
DEFAULT_MCU_PORT = "/dev/ttyUSB1"
DEFAULT_LEADER_BAUD = 115200
DEFAULT_MCU_BAUD = 115200
```

含义：

| 配置项 | 含义 |
|---|---|
| `DEFAULT_LEADER_PORT` | 主臂 Dynamixel 串口 |
| `DEFAULT_MCU_PORT` | MCU 串口 |
| `DEFAULT_LEADER_BAUD` | 主臂 Dynamixel 波特率 |
| `DEFAULT_MCU_BAUD` | MCU 串口波特率 |

### 6.2 发送频率配置

```python
DEFAULT_MASTER_SEND_FREQ_HZ = 50.0
DEFAULT_HEARTBEAT_FREQ_HZ = 1.0
DEFAULT_PRINT_FREQ_HZ = 1.0
DEFAULT_WRITE_TIMEOUT_S = 0.02
```

含义：

| 配置项 | 含义 | 建议 |
|---|---|---|
| `DEFAULT_MASTER_SEND_FREQ_HZ` | 主臂关节角发送频率 | `30Hz ~ 100Hz` |
| `DEFAULT_HEARTBEAT_FREQ_HZ` | PC 心跳发送频率 | `1Hz` |
| `DEFAULT_PRINT_FREQ_HZ` | 调试打印频率 | 调试时可设为 `10Hz`，正式运行可设为 `0` |
| `DEFAULT_WRITE_TIMEOUT_S` | MCU 串口写超时 | 不建议过大 |

### 6.3 关节 ID 配置

```python
DEFAULT_JOINT_IDS = [1, 2, 3, 4, 5]
GRIPPER_ID = 7
```

含义：

| 配置项 | 含义 |
|---|---|
| `DEFAULT_JOINT_IDS` | q0 ~ q4 对应的 5 个 Dynamixel ID |
| `GRIPPER_ID` | 末端开关使用的 Dynamixel ID |

### 6.4 ID7 末端开关映射配置

```python
GRIPPER_OPEN_POS = 2280
GRIPPER_CLOSED_POS = 1670
END_SWITCH_THRESHOLD = 0.50
```

ID7 的映射保持原双臂工程 gripper 逻辑：

```python
gripper_norm = 1 - (raw - GRIPPER_CLOSED_POS) / (GRIPPER_OPEN_POS - GRIPPER_CLOSED_POS)
```

含义：

| 变量 | 含义 |
|---|---|
| `raw` | ID7 的 `Present_Position` 原始值 |
| `GRIPPER_OPEN_POS` | 打开位置原始 tick |
| `GRIPPER_CLOSED_POS` | 闭合位置原始 tick |
| `gripper_norm` | 归一化结果 |
| `END_SWITCH_THRESHOLD` | 触发阈值 |

判断逻辑：

```python
end_switch = 1 if gripper_norm > END_SWITCH_THRESHOLD else 0
```

约定：

| `end_switch` | 含义 |
|---:|---|
| `0` | 未触发 / 打开 |
| `1` | 触发 / 闭合 |

### 6.5 ID7 力矩配置

```python
ENABLE_GRIPPER_TORQUE_ON_START = True
GRIPPER_CURRENT_LIMIT = 100
GRIPPER_GOAL_POSITION_ON_START = GRIPPER_OPEN_POS
```

启动时会对 ID7 执行：

1. `Torque_Enable = 0`
2. `Operating_Mode = CURRENT_POSITION`
3. `Current_Limit = 100`
4. `Torque_Enable = 1`
5. `Goal_Position = GRIPPER_OPEN_POS`

这样 ID7 会和原双臂工程一样开启力矩，并保持在打开位置附近

### 6.6 关节方向与零位配置

```python
DEFAULT_JOINT_SIGNS = [1, 1, 1, 1, 1]
DEFAULT_JOINT_OFFSETS_RAD = [0, 0, 0, 0, 0]
```

最终发送前的关节角计算为：

```python
q = sign * q_raw + offset
```

如果某个关节方向反了，例如 q1 和 q4 方向相反：

```python
DEFAULT_JOINT_SIGNS = [1, -1, 1, 1, -1]
```

也可以通过命令行临时传参：

```bash
python3 teleop_comments_zh_v3.py --joint-signs 1,-1,1,1,-1
```

如果某个关节需要加零位偏置：

```bash
python3 teleop_comments_zh_v3.py --joint-offsets-rad 0,0.15,0,-0.1,0
```

---

## 7. 命令行参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--leader-port` | `/dev/ttyUSB0` | 主臂 Dynamixel 串口 |
| `--mcu-port` | `/dev/ttyUSB1` | MCU 串口 |
| `--leader-baud` | `115200` | 主臂 Dynamixel 波特率 |
| `--mcu-baud` | `115200` | MCU 串口波特率 |
| `--freq` | `50` | `PC_MASTER_JOINTS` 发送频率 |
| `--heartbeat-rate` | `1` | `PC_HEARTBEAT` 发送频率 |
| `--joint-ids` | `1,2,3,4,5` | q0 ~ q4 对应的 ID |
| `--gripper-id` | `7` | 末端开关 ID |
| `--end-switch-threshold` | `0.50` | 末端开关触发阈值 |
| `--joint-signs` | `1,1,1,1,1` | 关节方向修正 |
| `--joint-offsets-rad` | `0,0,0,0,0` | 关节零位偏置 |
| `--ticks-per-rev` | `4096` | 每圈 tick 数 |
| `--no-wrap-ticks` | 关闭 | 不对 tick 做一圈取模 |
| `--signed-position` | 关闭 | 将 Present_Position 按 int32 解释 |
| `--crc-name` | `crc-ccitt-false` | CRC 类型 |
| `--write-timeout` | `0.02` | MCU 串口写超时 |
| `--print-rate` | `1` | 调试打印频率 |
| `--dry-run` | 关闭 | 只读取和打包，不发送 MCU |

---

## 8. 协议说明

### 8.1 通用帧格式

所有帧统一使用以下格式：

```text
SOF0     1 byte   0xA5
SOF1     1 byte   0x5A
LEN_H    1 byte   body length high byte
LEN_L    1 byte   body length low byte
VER      1 byte   protocol version
MSG_ID   1 byte   message id
SEQ      1 byte   sequence number
FLAGS    1 byte   flags
PAYLOAD  N byte   payload
CRC_H    1 byte   CRC16 high byte
CRC_L    1 byte   CRC16 low byte
```

字段说明：

| 字段 | 长度 | 说明 |
|---|---:|---|
| `SOF0` | 1 | 固定 `0xA5` |
| `SOF1` | 1 | 固定 `0x5A` |
| `LEN_H` | 1 | body 长度高字节 |
| `LEN_L` | 1 | body 长度低字节 |
| `VER` | 1 | 协议版本，当前 `0x01` |
| `MSG_ID` | 1 | 消息 ID |
| `SEQ` | 1 | 帧序号 |
| `FLAGS` | 1 | 标志位 |
| `PAYLOAD` | N | 业务数据 |
| `CRC_H` | 1 | CRC16 高字节 |
| `CRC_L` | 1 | CRC16 低字节 |

### 8.2 字节序

| 内容 | 字节序 |
|---|---|
| `LEN` | 大端 |
| `CRC` | 大端 |
| payload 内多字节整数 | 小端 |

### 8.3 CRC

CRC 使用：

```text
CRC16-CCITT-FALSE
```

参数：

| 参数 | 值 |
|---|---|
| 多项式 | `0x1021` |
| 初值 | `0xFFFF` |
| xorout | `0x0000` |
| refin | `false` |
| refout | `false` |

CRC 覆盖范围：

```text
SOF + LEN + BODY
```

也就是从 `0xA5 0x5A` 开始，到 payload 结束，不包含 CRC 字段本身

### 8.4 SEQ

`SEQ` 为 `uint8_t`

每发送一帧加 1

超过 `255` 后回到 `0`

### 8.5 FLAGS

当前 PC 发送帧不需要 ACK，因此：

```text
FLAGS = 0x00
```

---

## 9. PC_HEARTBEAT

方向：

```text
PC -> MCU
```

消息 ID：

```text
0x10
```

payload：

```text
empty
```

推荐频率：

```text
1Hz
```

完整 body：

```text
VER     1 byte   0x01
MSG_ID  1 byte   0x10
SEQ     1 byte
FLAGS   1 byte   0x00
```

因此：

```text
LEN = 4
```

---

## 10. PC_MASTER_JOINTS

方向：

```text
PC -> MCU
```

消息 ID：

```text
0x11
```

推荐频率：

```text
30Hz ~ 100Hz
```

本脚本默认：

```text
50Hz
```

### 10.1 payload 长度

原 `comms_protocol(5).md` 中 `PC_MASTER_JOINTS` payload 为 24 字节：

```text
stamp_ms + q0_urad + q1_urad + q2_urad + q3_urad + q4_urad
```

当前为了表达 ID7 末端开关状态，增加 1 字节：

```text
end_switch
```

因此当前 payload 长度为：

```text
25 bytes
```

### 10.2 payload 格式

```text
uint32_t stamp_ms
int32_t  q0_urad
int32_t  q1_urad
int32_t  q2_urad
int32_t  q3_urad
int32_t  q4_urad
uint8_t  end_switch
```

payload 偏移表：

| payload 偏移 | 长度 | 类型 | 字段 | 单位 / 说明 |
|---:|---:|---|---|---|
| 0 | 4 | `uint32_t` | `stamp_ms` | `ms` |
| 4 | 4 | `int32_t` | `q0_urad` | `urad` |
| 8 | 4 | `int32_t` | `q1_urad` | `urad` |
| 12 | 4 | `int32_t` | `q2_urad` | `urad` |
| 16 | 4 | `int32_t` | `q3_urad` | `urad` |
| 20 | 4 | `int32_t` | `q4_urad` | `urad` |
| 24 | 1 | `uint8_t` | `end_switch` | `0` 未触发，`1` 触发 |

对应 Python 打包格式：

```python
struct.pack("<IiiiiiB", stamp_ms, q0, q1, q2, q3, q4, end_switch)
```

其中 `<` 表示 payload 内部小端

### 10.3 角度单位

脚本内部读取主臂角度为 rad

发送前转换为 urad：

```python
q_urad = round(q_rad * 1_000_000)
```

MCU 端解析后恢复：

```c
q_rad = q_urad * 1e-6f;
```

### 10.4 body 长度

通用协议中：

```text
LEN = VER + MSG_ID + SEQ + FLAGS + PAYLOAD
```

因此当前 `PC_MASTER_JOINTS`：

```text
LEN = 4 + 25 = 29
```

即：

```text
LEN_H = 0x00
LEN_L = 0x1D
```

### 10.5 示例帧结构

```text
A5 5A
00 1D
01
11
SEQ
00
stamp_ms[4]
q0_urad[4]
q1_urad[4]
q2_urad[4]
q3_urad[4]
q4_urad[4]
end_switch[1]
CRC_H CRC_L
```

---

## 11. MCU 端需要同步修改的地方

由于 `PC_MASTER_JOINTS` payload 从 24 字节变为 25 字节，MCU 端必须同步修改解析逻辑

需要修改：

1. `PC_MASTER_JOINTS` payload 长度检查
2. 主臂关节角解析偏移保持不变
3. 新增 payload 偏移 24 的 `end_switch`
4. 数据缓存结构中增加末端开关状态
5. app 层消费主臂控制时同时读取 `end_switch`

建议 MCU 端结构：

```c
typedef struct {
    uint32_t stamp_ms;
    float q[5];
    uint8_t end_switch;
    bool fresh;
} pc_master_joints_t;
```

解析逻辑：

```c
stamp_ms = read_u32_le(payload + 0);
q[0] = read_i32_le(payload + 4) * 1e-6f;
q[1] = read_i32_le(payload + 8) * 1e-6f;
q[2] = read_i32_le(payload + 12) * 1e-6f;
q[3] = read_i32_le(payload + 16) * 1e-6f;
q[4] = read_i32_le(payload + 20) * 1e-6f;
end_switch = payload[24] ? 1 : 0;
```

---

## 12. 调试建议

### 12.1 检查串口设备

```bash
ls -l /dev/ttyUSB*
```

如果插拔顺序导致 `ttyUSB0` 和 `ttyUSB1` 交换，可以临时用命令行参数指定

### 12.2 检查 ID7 阈值

先执行：

```bash
python3 teleop_comments_zh_v3.py --dry-run --print-rate 10
```

观察：

```text
gripper_raw
gripper_norm
end_switch
```

如果开关太敏感，增大：

```python
END_SWITCH_THRESHOLD = 0.70
```

如果开关不容易触发，减小：

```python
END_SWITCH_THRESHOLD = 0.30
```

也可以命令行临时覆盖：

```bash
python3 teleop_comments_zh_v3.py --end-switch-threshold 0.65
```

### 12.3 检查关节方向

如果某个关节主从方向反了，修改：

```python
DEFAULT_JOINT_SIGNS = [1, -1, 1, 1, -1]
```

或者临时运行：

```bash
python3 teleop_comments_zh_v3.py --joint-signs 1,-1,1,1,-1
```

### 12.4 检查零位偏差

如果某个关节存在固定偏差，修改：

```python
DEFAULT_JOINT_OFFSETS_RAD = [0, 0.15, 0, -0.1, 0]
```

或者临时运行：

```bash
python3 teleop_comments_zh_v3.py --joint-offsets-rad 0,0.15,0,-0.1,0
```

---

## 13. 常见问题

### 13.1 Permission denied

现象：

```text
Permission denied: /dev/ttyUSB0
```

解决：

```bash
sudo usermod -aG dialout $USER
```

重新登录终端

### 13.2 找不到 Dynamixel

可能原因：

1. 串口选错
2. 波特率不一致
3. ID 不一致
4. Dynamixel 供电异常
5. USB 转 TTL 模块接线异常

建议先检查：

```bash
ls -l /dev/ttyUSB*
```

然后确认：

```bash
python3 teleop_comments_zh_v3.py --leader-port /dev/ttyUSB0 --dry-run
```

### 13.3 MCU 没收到帧

检查：

1. `--mcu-port` 是否正确
2. MCU 波特率是否与 `--mcu-baud` 一致
3. MCU 是否按照新 payload 长度 25 字节解析
4. MCU CRC 是否使用 CRC16-CCITT-FALSE
5. MCU 是否按大端解析 `LEN` 和 `CRC`

### 13.4 MCU 端 CRC 错误

重点检查：

1. CRC 覆盖范围必须是 `SOF + LEN + BODY`
2. CRC 不包含 CRC 字段本身
3. CRC 输出按大端放入帧尾
4. payload 内多字节字段是小端
5. `LEN` 是 body 长度，不是整帧长度

---

## 14. 推荐运行流程

第一次调试：

```bash
python3 teleop_comments_zh_v3.py --leader-port /dev/ttyUSB0 --dry-run --print-rate 10
```

确认 q0 ~ q4 和 ID7 正常后，再连接 MCU：

```bash
python3 teleop_comments_zh_v3.py \
  --leader-port /dev/ttyUSB0 \
  --mcu-port /dev/ttyUSB1 \
  --freq 50 \
  --print-rate 1
```

正式运行可以减少打印：

```bash
python3 teleop_comments_zh_v3.py \
  --leader-port /dev/ttyUSB0 \
  --mcu-port /dev/ttyUSB1 \
  --freq 50 \
  --print-rate 0
```

---

## 15. 与原双臂工程的关系

原双臂工程中的核心路径是：

```text
leader.get_action()
follower.send_action(action)
```

本脚本保留了 `leader.get_action()` 中主臂读取和 gripper 映射的关键逻辑

改造后的路径是：

```text
读取 Dynamixel 主臂
读取 ID7 末端开关
打包 PC_MASTER_JOINTS
通过 USB 串口发送给 MCU
```

也就是说，原来的 follower 控制协议已经被替换为当前 MCU 二进制协议

---

## 16. 版本约定

当前脚本协议相对于 `comms_protocol(5).md` 有一处扩展：

```text
PC_MASTER_JOINTS payload 从 24 字节扩展为 25 字节
```

新增字段：

```text
uint8_t end_switch
```

如果后续希望严格保持原协议不变，也可以新增单独的 PC 端末端开关消息

但在当前需求下，直接扩展 `PC_MASTER_JOINTS` 最简单，MCU 端只需要同步修改长度和解析偏移
