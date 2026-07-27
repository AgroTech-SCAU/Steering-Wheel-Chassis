# ASRPRO TWEN51 通信协议

## 一，串口参数

```text
baud: 115200
format: 8N1
line: CRLF
```

## 二，帧格式

```text
@payload*XX\r\n
```

`XX` 为 payload 字节异或校验

## 三，方向

| 方向 | 前缀 |
|---|---|
| PI -> ASRPRO | `A2P` |
| ASRPRO -> PI | `P2A` |

## 四，常用命令

| 命令 | 示例 | 说明 |
|---|---|---|
| `HELLO` | `P2A,1,seq,HELLO,version,boot_id` | ASRPRO 开机握手 |
| `HELLO_ACK` | `A2P,1,seq,HELLO_ACK,boot_id` | PI 确认握手 |
| `LISTEN` | `A2P,1,seq,LISTEN,1` | 开启语音识别 |
| `SPEAK` | `A2P,1,seq,SPEAK,autonomous_start` | 播放固定语音 |
| `EVENT` | `P2A,1,seq,EVENT,ASR,atlas_start` | 识别事件 |
| `EVENT_ACK` | `A2P,1,seq,EVENT_ACK,ASR` | PI 确认事件 |

## 五，固定 phrase_id

| phrase_id | 播报内容 |
|---|---|
| `transition_complete` | 遥操作区任务已完成 |
| `voice_prompt` | 请说阿特拉斯启动 |
| `autonomous_start` | 开始执行全自主运输任务 |
| `delivery_complete` | 货物派送完成 |
| `task_complete` | 全自主运输任务结束 |
| `task_skipped` | 当前阶段已跳过 |

