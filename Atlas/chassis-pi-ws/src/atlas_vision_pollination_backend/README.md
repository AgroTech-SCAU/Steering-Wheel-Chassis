# atlas_vision_pollination_backend 说明

本包同时提供相机目标服务和视觉授粉后端

旧的独立 vision 示例目录已经删除

视觉识别，语音文本，手眼变换，预识别位姿，预授粉位姿，授粉位姿和回退序列都在本包中完成

## 一，包内节点

### atlas_camera_target_service

提供服务

```text
/vision/detect_camera_target
```

服务类型

```text
atlas_mission_interfaces/srv/DetectCameraTarget
```

负责

```text
打开相机
加载目标检测模型
等待画面稳定
进行图像识别
根据业务规则选择目标
发布调试图像
发布语音文本或直接播报
返回相机光学坐标系下的目标点
```

### atlas_vision_pollination_backend

提供服务

```text
/atlas/manipulation/start
/atlas/manipulation/cancel
```

发布状态

```text
/atlas/manipulation/status
```

负责

```text
读取 pollination_actions.yaml
执行预识别关节动作
调用 /vision/detect_camera_target
复用 hand_eye_ok.py 的手眼与工具点偏移思路
计算目标在 arm_base_link 下的位置
执行预授粉位姿
执行授粉位姿
停留
回到预授粉位姿
回到预识别位姿
通过 /arm/joint_states 和 /arm/pose_position 判断动作是否完成
```

## 二，动作总流程

当总任务状态机调用 `/atlas/manipulation/start` 时，后端执行

```text
读取 prepare_action
  ↓
执行预识别关节位姿
  ↓
等待关节到位
  ↓
调用视觉目标服务
  ↓
如果返回 NO_TARGET，根据 empty_target_policy 决定跳过或失败
  ↓
记录识别时刻的关节角和末端姿态
  ↓
计算 target_base
  ↓
执行预授粉位置，工具点 z=0.097
  ↓
执行授粉位置
  ↓
停留 dwell_pollination_s
  ↓
回到预授粉位置
  ↓
回到预识别关节位姿
  ↓
发布 SUCCEEDED
```

当前默认序列

```text
预识别位姿
→ 视觉识别
→ 预授粉位姿
→ 授粉位姿
→ 停留
→ 预授粉位姿
→ 预识别位姿
```

## 三，手眼和工具点偏移

视觉识别发生在预识别位姿

识别时只计算一次目标在机械臂基座下的坐标

```text
target_base = base_T_tool0_at_detection * tool0_T_camera * target_camera
```

后续预授粉和授粉都使用同一个 `target_base`

```text
末端目标 = target_base - R_base_tool0_at_detection * 工具点偏移
```

原因

```text
机械臂移动后相机坐标系已经变化
不能拿旧相机坐标反复重新计算目标
```

工具点偏移在配置中修改

```yaml
pre_pollination_tool_point_m: [0.05, -0.015, 0.097]
pollination_tool_point_m: [0.05, -0.015, 0.087]
```

## 四，配置文件

### camera_target.yaml

用于配置相机目标服务

包括

```text
服务名
调试图像话题
语音文本话题
相机编号
图像尺寸
模型路径
目标真实宽度
画面稳定等待时间
单次处理最大时间
语音模式和语音设备
```

### pollination.yaml

用于配置视觉授粉后端节点

包括

```text
状态话题
启动服务
取消服务
关节状态话题
视觉服务名
机械臂关节服务
机械臂位置服务
动作配置文件路径
控制循环频率
反馈超时时间
动作到位阈值
默认速度
视觉超时
动作超时
```

### pollination_actions.yaml

用于配置预识别动作和到点任务

包括

```text
prepare_actions
arrival_tasks
工具点偏移
动作序列
停留时间
超时策略
空目标策略
```

## 五，配置预识别动作

示例

```yaml
prepare_actions:
  area_a_left_pre_detect:
    type: "joints"
    joints_deg: [180.0, 90.0, 360.0, 180.0, 180.0]
    speed_rad_s: 1.0
    timeout_s: 8.0
```

字段含义

| 字段 | 说明 |
|---|---|
| `type` | 动作类型，当前支持 noop 和 joints |
| `joints_deg` | 五个关节角，单位度 |
| `joints_rad` | 五个关节角，单位弧度，如果和 joints_deg 同时存在，优先使用 joints_rad |
| `speed_rad_s` | 机械臂运动速度 |
| `timeout_s` | 等待到位超时时间 |

## 六，配置授粉任务

示例

```yaml
arrival_tasks:
  visual_pollination:
    type: "visual_pollination"
    empty_target_policy: "skip"
    vision_service: "/vision/detect_camera_target"
    pre_pollination_tool_point_m: [0.05, -0.015, 0.097]
    pollination_tool_point_m: [0.05, -0.015, 0.087]
    speed_rad_s: 1.0
    timeout_s: 8.0
    dwell_pollination_s: 0.3
```

空目标策略

```text
skip 表示该点没有目标时跳过并返回成功
fail 表示该点没有目标时返回失败
```

## 七，视觉服务返回约定

识别成功

```text
success=true
message=目标类别
target_camera_m 为米单位目标点
```

没有目标但流程正常

```text
success=false
message=NO_TARGET
```

视觉故障

```text
success=false
message=其他错误
```

## 八，启动

启动视觉目标服务和授粉后端

```bash
ros2 launch atlas_vision_pollination_backend vision_pollination.launch.py
```

替换配置

```bash
ros2 launch atlas_vision_pollination_backend vision_pollination.launch.py \
  camera_config:=/home/wheeltec/my_config/camera_target.yaml \
  pollination_config:=/home/wheeltec/my_config/pollination.yaml \
  actions_config:=/home/wheeltec/my_config/pollination_actions.yaml
```

## 九，单独测试视觉服务

```bash
ros2 service call /vision/detect_camera_target atlas_mission_interfaces/srv/DetectCameraTarget \
"{waypoint_id: 'test_point', task_id: 'visual_pollination'}"
```

查看调试图像

```bash
ros2 topic echo /vision/debug_image
```

查看语音文本

```bash
ros2 topic echo /vision/voice_text
```

## 十，单独测试授粉后端

```bash
ros2 service call /atlas/manipulation/start atlas_mission_interfaces/srv/StartManipulation \
"{backend: 'vision_pollination', waypoint_id: 'area_a_02_down', prepare_action: 'area_a_left_pre_detect', arrival_task: 'visual_pollination'}"
```

查看状态

```bash
ros2 topic echo /atlas/manipulation/status
```

取消

```bash
ros2 service call /atlas/manipulation/cancel atlas_mission_interfaces/srv/CancelManipulation \
"{reason: 'manual cancel'}"
```

## 十一，常见问题

视觉服务没有启动

```text
检查 camera_index
检查 model_path
检查 OpenCV 是否能打开相机
检查 ultralytics 是否安装
```

返回 NO_TARGET

```text
检查模型类别名是否为配置逻辑期望的类别
检查目标是否完整进入画面
检查相机曝光和焦距
检查 target_real_width_mm 是否合理
```

机械臂动作没有完成

```text
检查 /arm/joint_states 是否新鲜
检查 /arm/pose_position 是否新鲜
检查目标点是否超出工作空间
检查 mcu 日志中是否有 IK 无解
```

手眼方向不对

```text
先只测试预授粉位姿
不要直接执行授粉点
检查 camera 坐标轴方向
检查 tool0_T_camera
检查工具点偏移符号
检查识别时的预识别位姿是否稳定
```

## 动作配置补充

`pollination_actions.yaml` 已经补齐 `src(141).zip` 中全部预识别动作和旧 MCU 纯关节授粉序列

`pre_detect_nav_XX` 对应旧路线 `nav_index=XX` 的预识别位姿

`visual_pollination` 是当前默认视觉授粉流程

`legacy_pollination_nav_XX` 是从旧 `pollen_route.c` 迁移过来的纯关节序列，默认不启用

如果某个点要复刻旧关节动作，把 `mission_route.yaml` 里该点的 `arrival_task` 改成对应 `legacy_pollination_nav_XX`
