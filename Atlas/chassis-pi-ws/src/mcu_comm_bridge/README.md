# mcu_comm_bridge

`mcu_comm_bridge` bridges the STM32 MCU binary protocol to ROS 2 Humble.

## Highlights

- Opens `/dev/ttyUSB0` with `TIOCEXCL`, so only one node can own the serial port.
- Verifies `1000000 8N1 raw` termios settings at startup and every `verify_termios_period_s`.
- Uses a bounded sliding-window parser with CRC-16-CCITT recovery instead of a byte-by-byte fixed state machine.
- Keeps latest valid IMU and ODOM samples, then republishes them at fixed ROS timer rates.
- Reuses the last valid sample on CRC loss only within the configured max reuse age.

## Key Parameters

```yaml
mcu_comm_bridge_node:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baudrate: 1000000
    imu_publish_rate_hz: 100.0
    odom_publish_rate_hz: 50.0
    imu_max_reuse_age_ms: 100
    odom_max_reuse_age_ms: 200
    reused_message_stamp_mode: "preserve_source"
    verify_termios_period_s: 5.0
    parser_raw_buffer_capacity: 4096
    frame_queue_capacity: 512
    stats_rate_hz: 1.0
```

`reused_message_stamp_mode` supports:

- `preserve_source`: reused publishes keep the timestamp from the last valid sample.
- `publish_now`: reused publishes get a fresh ROS timestamp. This can hide stale measurements and is not the default.

## Troubleshooting

Only one process should hold `/dev/ttyUSB0`:

```bash
sudo fuser -v /dev/ttyUSB0
sudo lsof /dev/ttyUSB0
```

Check whether ModemManager is probing the adapter:

```bash
sudo systemctl status ModemManager
```

If ModemManager interferes, prefer a udev rule that sets:

```text
ID_MM_DEVICE_IGNORE=1
```

## Build And Test

```bash
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select mcu_comm_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

colcon test --packages-select mcu_comm_bridge
colcon test-result --verbose
```
