# mcu_comm_bridge

`mcu_comm_bridge` is the ROS 2 Humble bridge between Atlas Pi and the MCU binary protocol.

## What It Does

- Parses `MCU_IMU`, `MCU_ODOM`, `MCU_ARM_STATE`, and `MCU_STATUS`
- Publishes `/imu`, `/odom`, `/arm/joint_states`, `/arm/pose`, `/arm/pose_position`
- Publishes latched MCU state on `/mcu/status`
- Publishes one-shot auto task edges on `/mcu/auto_task_event`
- Subscribes to `/motor_cmd_vel` and streams `PI_CONTROL` at the configured rate
- Provides brake, estop, yaw, and arm command services

## MCU Auto Task Ownership

The MCU is the only owner of:

- The app state machine
- Auto task start authority
- Auto task reset authority

The Pi bridge must not add APIs that request `Idle -> AutoPi`, and it must not invent:

- `PI_MODE_REQUEST`
- `PI_STATE_REQUEST`
- `/mcu/enter_auto`
- `/mcu/set_mode`

The Pi side only:

1. Receives `MCU_STATUS`
2. Parses `auto_start_latched`
3. Detects `START` and `RESET` edges
4. Clears the previous local auto-task context
5. Publishes one-shot events for upper layers
6. Runs automatic control only while MCU is already in `AutoPi`

## Topics

### `/mcu/status`

Message type: `mcu_comm_bridge/msg/McuStatus`

QoS:

- Reliable
- KeepLast(1)
- TransientLocal

Use it to inspect the latest truth state from MCU:

```bash
ros2 topic echo /mcu/status \
  --qos-reliability reliable \
  --qos-durability transient_local
```

### `/mcu/auto_task_event`

Message type: `mcu_comm_bridge/msg/AutoTaskEvent`

QoS:

- Reliable
- KeepLast(10)
- Volatile

This topic only publishes edge events:

- `EVENT_START`
- `EVENT_RESET`

Use it like this:

```bash
ros2 topic echo /mcu/auto_task_event
```

## `auto_start_latched` Semantics

- `0`: MCU has cleared the previous auto task and allows preparing the next one
- `1`: MCU has latched an auto task start for the current round

Pi-side behavior:

1. `0 -> 1` can produce one `EVENT_START`
2. `1 -> 0` clears local auto-task context and produces one `EVENT_RESET`
3. Repeated `0` does not repeat reset
4. Repeated `1` does not repeat start
5. `auto_start_latched=1` is not enough by itself; `app_state` must also be `AutoPi`

If MCU reports `auto_start_latched=1` while the app state is not yet `AutoPi`, the bridge keeps the event pending and waits for a later `AutoPi` status frame.

## Upper-Layer Task Manager Contract

Upper layers should consume `/mcu/auto_task_event` as the one-shot trigger.

- On `EVENT_START`: start one navigation or mission run
- On `EVENT_RESET`: cancel the active task, stop the action client, and clear BT/Nav2/mission context

Upper layers must not repeatedly start tasks only because `app_state == AutoPi`.

## Main Parameters

```yaml
mcu_comm_bridge_node:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baudrate: 1000000
    cmd_vel_topic: "/motor_cmd_vel"
    control_rate_hz: 50.0
    cmd_vel_timeout_ms: 200
    arm_command_repeat_count: 3
    mcu_status_topic: "/mcu/status"
    auto_task_event_topic: "/mcu/auto_task_event"
```

## Build And Test

```bash
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select mcu_comm_bridge \
  --symlink-install

colcon test \
  --packages-select mcu_comm_bridge

colcon test-result --verbose
```
