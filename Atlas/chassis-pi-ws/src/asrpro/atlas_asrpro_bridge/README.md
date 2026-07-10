# atlas_asrpro_bridge

ASRPRO TWEN51 USB 串口到 ROS 2 的适配桥

## 默认参数

```text
port=/dev/atlas_asrpro
baudrate=115200
protocol_version=1
```

## ROS 接口

```text
/atlas/asrpro/status       AsrproStatus
/atlas/asrpro/event        AsrproEvent
/atlas/asrpro/recognized   std_msgs/String
/atlas/asrpro/speak        AsrproSpeak
```

## 启动

```bash
ros2 launch atlas_asrpro_bridge asrpro.launch.py
```

## 协议

```text
@DIRECTION,VERSION,SEQUENCE,COMMAND,ARGUMENTS*HH\r\n
```

实现包括重连；ACK 超时重试；事件确认；sequence 去重协作；心跳和失效命令清理

完整固件要求见 `chassis-pi-ws/docs/ASRPRO_TWEN51适配与烧录说明.md`
