# ASRPRO TWEN51 适配与烧录说明

## 1 系统定位

ASRPRO 只负责离线语音识别和离线语音播报；不直接控制底盘；机械臂或 MCU

树莓派通过 USB 串口与 ASRPRO 通信；ROS 节点为

```text
atlas_asrpro_bridge
```

默认串口

```text
/dev/atlas_asrpro
115200 8N1
```

## 2 上电行为

ASRPRO 固件上电后自动运行；不等待外部按键

推荐启动流程

```text
初始化串口
→ 初始化离线识别模型和播报资源
→ 生成本次 boot_id
→ 每 500 ms 发送 HELLO
→ 收到 HELLO_ACK
→ 接收 LISTEN,1
→ 开启识别
```

Pi 端可晚于 ASRPRO 启动；ASRPRO 必须持续发送 HELLO 直到握手完成

## 3 物理和串口参数

```text
波特率 115200
数据位 8
停止位 1
校验位 None
流控 None
编码 UTF-8
行尾 CRLF
```

ASRPRO 必须使用固定 USB 串口设备名；配置方法见 `串口固定绑定教程.md`

## 4 帧格式

```text
@DIRECTION,VERSION,SEQUENCE,COMMAND,ARG1,ARG2*HH\r\n
```

字段

| 字段 | 含义 |
|---|---|
| DIRECTION | `A2P` 表示树莓派发往 ASRPRO；`P2A` 表示 ASRPRO 发往树莓派 |
| VERSION | 当前固定 `1` |
| SEQUENCE | `1` 到 `65535`；各方向独立递增 |
| COMMAND | 大写命令名 |
| ARG | 不允许包含逗号；星号；回车或换行 |
| HH | `DIRECTION` 到最后一个参数的 UTF-8 字节逐字节 XOR；两位大写十六进制 |

示例载荷

```text
P2A,1,12,EVENT,ASR,atlas_start
```

XOR 计算范围不包含 `@`；`*HH` 和行尾

## 5 命令表

### 5.1 ASRPRO 到 Pi

#### HELLO

```text
@P2A,1,1,HELLO,1.0.0,BOOT_20260710_001*HH\r\n
```

参数

```text
firmware_version
boot_id
```

同一上电周期 boot_id 保持不变；模块重启后必须变化

#### EVENT ASR

```text
@P2A,1,20,EVENT,ASR,atlas_start*HH\r\n
```

Pi 返回

```text
@A2P,1,20,EVENT_ACK,ASR*HH\r\n
```

ASRPRO 在未收到 `EVENT_ACK` 时应每 200 ms 重发；最多重发五次；相同 sequence 的事件内容必须保持一致

#### EVENT SPEAK_DONE

```text
@P2A,1,21,EVENT,SPEAK_DONE,transition_complete*HH\r\n
```

Pi 返回

```text
@A2P,1,21,EVENT_ACK,SPEAK_DONE*HH\r\n
```

#### ACK

```text
@P2A,1,10,ACK,SPEAK*HH\r\n
```

ACK 的 sequence 必须与收到的 Pi 命令一致

#### NACK

```text
@P2A,1,10,NACK,SPEAK,UNKNOWN_PHRASE*HH\r\n
```

常用原因

```text
UNKNOWN_COMMAND
UNKNOWN_PHRASE
BUSY
BAD_ARGUMENT
NOT_READY
```

#### PONG

```text
@P2A,1,30,PONG,1780000000*HH\r\n
```

sequence 与 PING 相同

### 5.2 Pi 到 ASRPRO

#### HELLO_ACK

```text
@A2P,1,2,HELLO_ACK,BOOT_20260710_001*HH\r\n
```

ASRPRO 校验 boot_id 后返回

```text
@P2A,1,2,ACK,HELLO_ACK*HH\r\n
```

#### LISTEN

```text
@A2P,1,3,LISTEN,1*HH\r\n
```

参数 `1` 打开识别；`0` 关闭识别

返回

```text
@P2A,1,3,ACK,LISTEN*HH\r\n
```

#### SPEAK

```text
@A2P,1,4,SPEAK,voice_prompt*HH\r\n
```

ASRPRO 收到后先按 sequence 去重；合法且空闲时立即返回 ACK；播报结束后发送 `EVENT,SPEAK_DONE`

#### PING

```text
@A2P,1,5,PING,1780000000*HH\r\n
```

ASRPRO 返回同 sequence 的 PONG；不额外返回 ACK

## 6 phrase_id 资源表

ASRPRO 工程中建立以下固定播报资源

| phrase_id | 建议播报文本 |
|---|---|
| transition_complete | 遥操作区任务已完成 |
| voice_prompt | 请说 Atlas 启动 |
| autonomous_start | 开始执行全自主运输任务 |
| delivery_complete | 货物派送完成 |
| task_complete | 全自主运输任务结束 |
| task_skipped | 当前阶段已跳过 |

状态机传输 phrase_id；不传输任意中文文本；这样能够减少串口编码和动态 TTS 依赖

## 7 intent 资源表

| 识别语句 | 上报 intent |
|---|---|
| Atlas 启动 | atlas_start |
| 阿特拉斯启动 | atlas_start |
| 开始全自主运输 | atlas_start |

ASRPRO 只上报稳定英文 intent；不要把原始识别文本直接作为协议参数

## 8 播报期间的识别门控

ASRPRO 播报时应暂停触发词判定；避免自己的扬声器声音触发识别

推荐逻辑

```text
收到 SPEAK
→ 保存 listen_enabled
→ 临时暂停识别回调
→ 播放固定语音
→ 恢复原 listen_enabled
→ 发送 SPEAK_DONE
```

Pi 端状态机在 `SPEAK_DONE` 到达前不会进入下一播报阶段

## 9 sequence 去重

Pi 命令可能因 ACK 超时而重发；ASRPRO 必须缓存最近至少 32 条下行 sequence

同一 sequence 再次收到时

```text
COMMAND 和参数完全一致
→ 不重复执行动作
→ 重发之前的 ACK 或 NACK
```

```text
COMMAND 或参数不一致
→ 返回 NACK,SEQUENCE_CONFLICT
```

`SPEAK` 去重非常重要；否则串口瞬时丢 ACK 会导致重复播报

## 10 ASRPRO 固件伪代码

```c
void setup(void) {
    serial_init(115200);
    speech_resource_init();
    asr_model_init();
    boot_id_generate();
    listen_enabled = false;
    handshake_ok = false;
}

void loop(void) {
    serial_receive_and_parse();

    if (!handshake_ok && elapsed_ms(last_hello) >= 500) {
        send_hello(firmware_version, boot_id);
    }

    if (handshake_ok && listen_enabled && !speech_busy) {
        const char *intent = asr_poll_intent();
        if (intent != NULL) {
            send_event_with_retry("ASR", intent);
        }
    }

    event_retry_tick();
}
```

命令处理

```c
HELLO_ACK:
    verify boot_id
    handshake_ok = true
    ACK HELLO_ACK

LISTEN:
    listen_enabled = argument == 1
    ACK LISTEN

SPEAK:
    reject if phrase_id unknown or speech_busy
    ACK SPEAK
    pause recognition
    play phrase_id
    restore recognition
    EVENT SPEAK_DONE phrase_id

PING:
    PONG with same sequence
```

## 11 Pi 端 ROS 接口

状态

```bash
ros2 topic echo /atlas/asrpro/status
```

识别结果

```bash
ros2 topic echo /atlas/asrpro/recognized
```

底层事件

```bash
ros2 topic echo /atlas/asrpro/event
```

播报服务

```bash
ros2 service call /atlas/asrpro/speak \
  atlas_mission_interfaces/srv/AsrproSpeak \
  "{request_id: test, phrase_id: transition_complete}"
```

## 12 联调步骤

1. 只给 ASRPRO 和树莓派上电
2. 使用串口工具确认 ASRPRO 周期发送 HELLO
3. 启动 `atlas_asrpro_bridge`
4. 确认收到 HELLO_ACK 和 LISTEN ACK
5. 调用六个 phrase_id；确认每次只有一次播报和一次 SPEAK_DONE
6. 连续说触发词；确认每次上报 `atlas_start`
7. 播报期间说触发词；确认不会误触发
8. 拔掉 USB；确认 Pi 状态变为 offline
9. 重新插入；确认自动重连和新 HELLO 握手
10. 在命令 ACK 发送前模拟丢帧；确认重发不会重复播报

## 13 故障行为

```text
ASRPRO 未连接；状态机 PRECHECK 安全等待
ASRPRO 在中转播报阶段掉线；播报重试后按配置跳过
ASRPRO 在 WAIT_VOICE_START 掉线；状态机继续制动；设备重连后仍可识别
ASRPRO 在运输阶段掉线；不影响已经开始的导航和机械臂任务
```
