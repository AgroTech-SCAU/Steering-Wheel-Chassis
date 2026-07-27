# handeye_calibration_tool 说明

本包用于树莓派端单目手眼标定

它用于得到视觉授粉后端需要的相机到工具坐标关系

## 一，包定位

负责

```text
打开相机
识别棋盘格或编码棋盘格
读取机械臂关节和末端位姿
采集多组机器人位姿和标定板位姿
求解手眼矩阵
保存结果
```

不负责

```text
不执行自动任务
不移动底盘
不执行授粉动作
不替代视觉授粉后端的运行时手眼变换
```

## 二，输入话题

默认使用

```text
/arm/joint_states
/arm/pose
/arm/pose_position
```

其中 `/arm/pose_position` 已经作为默认末端位置话题

## 三，标定模式

眼在手上

```text
相机固定在机械臂末端
标定板固定在外部环境中
配置 handeye_mode: eye_in_hand
```

眼在手外

```text
相机固定在外部环境中
标定板固定在机械臂末端
配置 handeye_mode: eye_to_hand
```

当前视觉授粉任务主要使用眼在手上

## 四，标定板配置

普通棋盘格填写内角点数量

```text
如果棋盘格是 10 x 7 个黑白方格
内角点是 9 x 6
```

方格边长使用米

```text
25 mm 填 0.025
```

编码棋盘格需要配置字典，方格数，方格边长和标记边长

## 五，采样要求

建议

```text
采集 15 组以上姿态
姿态需要覆盖不同位置和不同角度
每组采样前保持机械臂静止
标定板尽量完整清晰
避免全部样本在同一平面和同一角度附近
```

## 六，输出结果使用方式

输出结果用于视觉授粉后端中的手眼变换

需要把结果转成后端使用的配置或代码常量

视觉授粉后端运行时会根据

```text
识别时刻的机械臂位姿
相机坐标目标点
工具点偏移
```

计算机械臂目标位置

## 七，启动

```bash
ros2 launch handeye_calibration_tool handeye_tool.launch.py
```

替换配置

```bash
ros2 launch handeye_calibration_tool handeye_tool.launch.py \
  config:=/home/wheeltec/my_config/handeye_tool.yaml
```

## 八，配置文件

配置文件

```text
config/handeye_tool.yaml
```

重点参数

| 参数 | 说明 |
|---|---|
| `camera_index` | 相机编号 |
| `camera_width` | 图像宽度 |
| `camera_height` | 图像高度 |
| `intrinsics_file` | 相机内参文件 |
| `board_type` | 标定板类型 |
| `handeye_mode` | 眼在手上或眼在手外 |
| `arm_pose_source` | 机械臂位姿来源 |
| `arm_fk_position_topic` | 末端位置话题 |
| `stable_duration_s` | 采样前静止时长 |
| `output_directory` | 结果保存目录 |
