# screw_pick — 螺丝视觉抓取系统

基于 ROS 2 的视觉抓取工程，包含相机标定、手眼标定、YOLO 目标检测、像素到机械臂基座坐标变换，以及抓取目标选择。

## 1. 项目组成

```text
screw_pick/
├── calib/                    # 相机内参和手眼标定脚本
├── handeye_bridge/           # 坐标变换、深度计算、机械臂控制
├── vison_topic/              # YOLO ONNX 检测节点
├── vison_topic_interfaces/   # 自定义 msg / srv
└── README.md
```

主要数据流：

```text
相机图像
  → vison_topic 检测目标中心
  → /detection_centers
  → handeye_bridge 匹配检测时间与机械臂位姿
  → 像素/深度转换为 base 坐标
  → /pick_target 选择目标和层
  → /mcu/set_arm_pose
```

## 2. 使用前必须确认

在允许机械臂自动运动前，依次确认：

1. 相机分辨率与内参标定一致，当前工程要求 `640×480`。
2. 更换相机、焦距、分辨率或内参后，重新采集手眼样本并求解。
3. `handeye_bridge/config/bridge_node.yaml` 中的初始观察位已经空载验证。
4. 三层工作高度或 PnP 目标尺寸已经实测。
5. 新标定文件已经复制到 `handeye_bridge/config/` 并重新构建。
6. 首次验证时保持 `auto_send: false`，只观察计算坐标。

> 当前配置文件中的 `initial_pose_configured`、`plane_heights_configured` 和
> `auto_send` 可能已经是 `true`。换设备或重新部署时，不要直接沿用；应先改为
> `false`，完成空载验证后再逐项开启。

## 3. 环境与构建

以下目录约定：

- 项目源码根目录：包含 `calib/`、`handeye_bridge/`、`vison_topic/`。
- ROS 2 工作空间根目录：包含 `src/`、`build/`、`install/`、`log/`。

若本项目位于 ROS 2 工作空间的 `src/screw_pick/`：

```bash
cd ~/ros2_ws
colcon build --packages-select \
    vison_topic_interfaces vison_topic handeye_bridge
source install/setup.bash
```

修改下列内容后必须重新构建并重新 `source`：

- `.msg` 或 `.srv` 接口；
- Python 节点代码；
- launch 文件；
- `handeye_bridge/config/` 中打包安装的 YAML。

## 4. 标定流程

所有标定命令都从项目源码根目录执行。

### 4.1 相机内参标定

先确认 `calib/camera_calib.py` 中的棋盘格内角点数量和方格尺寸与实物一致：

```bash
python calib/camera_calib.py
```

建议采集 20～30 张：棋盘覆盖画面中心、四角和边缘，并包含不同距离与倾角。不要只在画面中心平移棋盘。

输出文件：

```text
calib/camera_intrinsics.yaml
```

内参标定完成后应检查 RMS 和单张重投影误差。畸变系数数值大不一定代表错误，但如果 RMS 很大、边缘直线校正后仍明显弯曲，通常需要检查：

- 棋盘格行列数是否填写反了；
- 方格尺寸和单位是否正确；
- 图片是否模糊、反光或棋盘不平；
- 标定图片是否缺少边缘和倾斜姿态；
- 标定与运行分辨率是否都是 `640×480`；
- 自动对焦是否在采集过程中变化。

### 4.2 采集手眼样本

棋盘格必须固定不动，机械臂带着相机改变姿态：

```bash
# 正常模式，推荐
python calib/collect_samples.py

# 跳过质量门禁，仅用于诊断
python calib/collect_samples.py --minimal
```

建议采集 20～30 组，优先增加绕两个不同轴的旋转变化，避免大量重复或只有平移的姿态。

程序会拒绝近重复姿态，并拦截相邻样本间超过 120° 的棋盘格翻转。出现翻转提示时，不要手改 YAML；保持棋盘固定，换一个幅度较小的机械臂姿态重新采集。

### 4.3 诊断、求解与验证

```bash
# 求解前检查样本分布
python calib/diagnose.py calib/samples.yaml

# 常规求解：RANSAC + 全量相对运动 + LM 精化
python calib/solve.py calib/samples.yaml

# 可选：Bundle Adjustment
python calib/solve.py calib/samples.yaml --ba

# 使用已有结果验证
python calib/verify.py calib/samples.yaml \
    --result calib/samples_result.yaml
```

输出文件：

```text
calib/samples.yaml
calib/samples_result.yaml
```

求解至少需要 6 组有效样本，建议 20～30 组。不要只看“求解成功”，还要检查平移 RMS、旋转误差、内点数量和实际抓取验证结果。

## 5. 部署标定结果

先把标定结果从当前电脑同步到机器人。以下内容保留为项目的标准操作方式。

```bash
# rsync — 增量同步 (推荐)
rsync -avz calib/samples_result.yaml \
    wheeltec@192.168.0.100:screw_pick/calib/

# 或者用 scp — 单文件拷贝
scp calib/samples_result.yaml \
    wheeltec@192.168.0.100:screw_pick/calib/
```

相机内参有变化时也要同步：

```bash
rsync -avz calib/camera_intrinsics.yaml \
    wheeltec@192.168.0.100:screw_pick/calib/
```

部署前还需把结果复制到 `handeye_bridge/config/` 并重新构建；运行时读取安装空间中的相对配置路径。

```bash
cp calib/camera_intrinsics.yaml handeye_bridge/config/
cp calib/samples_result.yaml handeye_bridge/config/

cd ~/ros2_ws
colcon build --packages-select \
    vison_topic_interfaces vison_topic handeye_bridge
source install/setup.bash
```

## 6. SSH 显示转发 (X11 Forwarding)

脚本使用 OpenCV `imshow` 显示画面，SSH 到机器人时默认没有显示器。两种解决办法：

### 方案 A: SSH X11 转发（不改代码，推荐）

PC 上：

```bash
# Linux: X11 自带
# Windows: 先装 VcXsrv 并启动
# Mac: 先装 XQuartz 并启动

ssh -X wheeltec@192.168.0.100
```

机器人端确认 X11 转发已开启：

```bash
grep X11Forwarding /etc/ssh/sshd_config   # 应该是 yes
# 如果不是: sudo sed -i 's/.*X11Forwarding.*/X11Forwarding yes/' /etc/ssh/sshd_config
# sudo systemctl restart sshd
sudo apt install xauth  # 如果提示 xauth not found
```

### 方案 B: Headless 无头模式

加 `--headless` 参数，不弹窗口，终端输出状态，截图保存在项目内的相对目录 `calib/debug/`：

```bash
python calib/camera_calib.py --headless
# 终端显示: ✓ 检测成功 / ✗ 未检测到 | 已拍:N张 | [回车拍照 / C计算 / Q退出]
# 截图: calib/debug/calib_camera_debug.jpg

python calib/collect_samples.py --headless
# 终端显示: 距离=XXXmm | XXpx/格(优/可/差) | 重投影=X.XXXpx | 静止✓ | [回车采集 / Q退出]
# 截图: calib/debug/calib_debug.jpg
```

需要把调试截图拉回 PC 时：

```bash
scp wheeltec@192.168.0.100:screw_pick/calib/debug/calib_debug.jpg .
```

## 7. 深度与坐标变换

深度模式在 `handeye_bridge/config/bridge_node.yaml` 中选择。

### 7.1 manual：固定工作平面，默认

```yaml
depth_mode: "manual"
plane1_z_m: -0.03
plane2_z_m: 0.12
plane3_z_m: 0.19
target_z_offset_m: 0.03
```

计算过程：像素去畸变后形成相机射线，经手眼矩阵和机械臂 FK 转换到 base 坐标系，再与 `Z=planeX_z_m` 的平面求交。

`planeX_z_m` 是机械臂基座坐标系中的平面高度，不是相机到平面的距离。测量方式：

```bash
ros2 topic echo /arm/pose --once
```

让末端到达对应工作平面，读取位置中的 `z`，分别填入三层参数。最终发送高度为：

```text
目标 Z = planeX_z_m + target_z_offset_m + manual_offset_z_m
```

当前代码中的 `camera_to_plane1_distance_m`、`camera_to_plane2_distance_m`、
`camera_to_plane3_distance_m` 仅作为测量记录和像素比例参考，manual 模式的
射线求交不使用这些参数。

### 7.2 pnp：四点矩形深度

```yaml
depth_mode: "pnp"
pnp_target_width_m: 0.05
pnp_target_height_m: 0.05
pnp_max_reprojection_px: 2.0
pnp_min_depth_m: 0.05
pnp_max_depth_m: 0.80
```

PnP 模式要求同时检测到四个角，并且四点确实对应一个已知尺寸的平面矩形。宽高必须填写四个目标中心之间的真实距离。缺点、排序错误或目标并非矩形时，PnP 深度不可靠。

## 8. 检测排序与目标编号

四目标编号约定：

```text
0 左上 ───────── 1 右上
  │                 │
  │                 │
3 左下 ───────── 2 右下
```

`corner_index` 的含义：

| 值 | 位置 |
|---:|---|
| 0 | 左上 TL |
| 1 | 右上 TR |
| 2 | 右下 BR |
| 3 | 左下 BL |

当前检测代码最多取置信度最高的 4 个目标参与分配。PnP 必须有完整四点；少于四点时不要把缺失角的身份当作已经可靠恢复。

## 9. 启动与操作

### 9.1 一键启动

```bash
# 默认无预览，适合机器人部署
ros2 launch handeye_bridge screw_pick.launch.py

# 显示 OpenCV 预览，需要本地桌面或 X11
ros2 launch handeye_bridge screw_pick.launch.py no_preview:=false
```

bridge 启动后会按照配置尝试回到初始观察位。确认到位门禁：

```bash
ros2 topic echo /initial_pose_ready --once
```

需要重新发送初始观察位时：

```bash
ros2 service call /move_to_initial_pose std_srvs/srv/Trigger "{}"
```

### 9.2 开始和停止检测

```bash
# 开始持续检测
ros2 service call /vision_detect \
    vison_topic_interfaces/srv/VisionDetect "{start: true}"

# 查看实时检测结果
ros2 topic echo /detection_centers

# 停止检测并返回本次检测的最佳结果
ros2 service call /vision_detect \
    vison_topic_interfaces/srv/VisionDetect "{start: false}"
```

停止时的最佳帧会以 `is_final_best: true` 再发布一次，并保留原始采集时间戳。bridge 使用该时间戳匹配采集当时的机械臂位姿。

### 9.3 选择目标与工作层

```bash
# 选择左上目标，第 1 层
ros2 topic pub /pick_target \
    vison_topic_interfaces/msg/PickTarget \
    "{corner_index: 0, layer: 1}" --once

# 选择右下目标，第 3 层
ros2 topic pub /pick_target \
    vison_topic_interfaces/msg/PickTarget \
    "{corner_index: 2, layer: 3}" --once
```

`auto_send: false` 时只计算并打印坐标；`auto_send: true` 才调用机械臂服务。建议按以下顺序验证：

1. `auto_send: false`，检查中心和边缘多个目标的坐标。
2. 检查三层高度、同步误差和 workspace 门禁。
3. 让机械臂在目标上方空载运动，确认方向和量级。
4. 最后再开启 `auto_send: true`。

## 10. ROS 接口

| 名称 | 类型 | 用途 |
|---|---|---|
| `/vision_detect` | `VisionDetect` 服务 | 启动或停止检测 |
| `/vision_detections` | `Float32MultiArray` | 原始实时检测数据 |
| `/detection_centers` | `DetectionCenterArray` | 带时间戳和角编号的中心点 |
| `/pick_target` | `PickTarget` | 选择目标编号和高度层 |
| `/arm/pose` | `PoseStamped` | MCU FK 末端位姿 |
| `/move_to_initial_pose` | `Trigger` 服务 | 返回初始观察位 |
| `/initial_pose_ready` | `Bool` | 初始观察位到位门禁 |
| `/mcu/set_arm_pose` | 机械臂服务 | 发送目标位姿 |

## 11. 常见问题

### 第二天使用相同深度，结果仍发生变化

优先检查相机支架、机械臂零位、相机焦距/自动对焦、工作平面是否移动，以及检测框中心是否漂移。手眼矩阵和内参都默认相机相对末端刚性不变；相机发生轻微转动，也会让画面边缘产生明显坐标误差。

### 一个目标准确，四个目标误差不同

这通常不是一个统一的常量偏置。重点检查内参和畸变、手眼旋转误差、检测点定义是否一致，以及平面是否真的与 base 的 XY 平面平行。只调 `manual_offset_x/y` 只能补偿整体偏差，不能修复随画面位置变化的误差。

### PnP 深度跳动

PnP 对四点中心误差、点顺序、物理宽高和矩形假设非常敏感。先看重投影误差；若四个目标不是刚性固定的矩形，不应使用 PnP 推算共同深度。

### 无法显示 OpenCV 窗口

使用 `ssh -X`，或按第 6 节使用 `--headless`。完整应用部署时可使用默认的 `no_preview:=true`。

## 12. 详细文档

- 标定算法和参数：`calib/README.md`
- 检测节点与服务：`vison_topic/README.md`
- 坐标变换与安全门禁：`handeye_bridge/README.md`
