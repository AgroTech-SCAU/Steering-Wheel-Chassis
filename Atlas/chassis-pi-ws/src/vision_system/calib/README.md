# calib — 相机内参与手眼标定

本目录负责生成在线抓取所需的两个标定文件：

```text
camera_intrinsics.yaml   相机内参和畸变系数
samples_result.yaml      相机到机械臂末端的手眼变换
```

推荐执行顺序：

```text
camera_calib.py
  → collect_samples.py
  → diagnose.py
  → solve.py
  → verify.py
  → 部署到 handeye_bridge/config/
```

## 1. 坐标系约定

```text
机械臂基座 arm_base_link:
  X/Y/Z 由 MCU 的 /arm/pose 定义

相机 camera_optical_frame（OpenCV）:
  X 向右，Y 向下，Z 从镜头向前

棋盘格 target:
  X 沿列方向，Y 沿行方向，Z 按右手定则
```

结果文件保存 `^gripper T_camera`，即把相机坐标转换到机械臂末端坐标，也就是相机在 `tool0` 中的位姿。

## 2. 文件说明

| 文件 | 作用 | 输入 | 输出 |
|---|---|---|---|
| `camera_calib.py` | 标定相机内参 | 棋盘格图像 | `camera_intrinsics.yaml` |
| `collect_samples.py` | 采集手眼样本 | 内参、图像、机械臂位姿 | `samples.yaml` |
| `diagnose.py` | 求解前检查数据 | `samples.yaml` | 终端诊断报告 |
| `solve.py` | 求解手眼矩阵 | `samples.yaml` | `samples_result.yaml` |
| `verify.py` | 验证标定一致性 | 样本和结果 | 误差报告 |
| `bundle_adjust.py` | 可选像素级精化 | 含角点的样本 | BA 精化结果 |
| `calib_utils.py` | 公共角点、PnP、数据工具 | — | — |
| `fk_utils.py` | FK 和矩阵工具 | 关节角/位姿 | 变换矩阵 |
| `robot_params.yaml` | 机械臂 MDH 和工具参数 | 实测参数 | FK 配置 |

所有命令均从项目源码根目录执行，例如：

```bash
cd ~/screw_pick
python calib/camera_calib.py
```

## 3. 相机内参标定

### 3.1 填写棋盘参数

`camera_calib.py` 与 `collect_samples.py` 顶部的参数必须相同，并与实物一致：

```python
CHESSBOARD_COLS = 11   # 横向内角点数量，不是方格数量
CHESSBOARD_ROWS = 8    # 纵向内角点数量
SQUARE_SIZE_MM = 15    # 单个方格边长，单位 mm
```

例如棋盘有 `12×9` 个方格，则内角点为 `11×8`。

### 3.2 采集图像

```bash
python calib/camera_calib.py

# 极简模式：至少 3 张，全部图像直接使用 OpenCV 默认模型求解
python calib/camera_calib.py --minimal
```

操作：

- 空格：保存当前有效棋盘图像；
- `C`：计算并保存内参；
- `Q` 或 `Esc`：退出。

建议采集 20～30 张：

- 棋盘覆盖中心、四角和四条边；
- 包含近、中、远距离；
- 包含绕水平和竖直方向的不同倾角；
- 保持清晰、无强反光，棋盘本身必须平整；
- 标定期间不要改变焦距或自动对焦状态。

### 3.3 判断结果

输出：`calib/camera_intrinsics.yaml`。

不要只看畸变系数数值大小，重点检查：

1. 标定分辨率是否为实际运行分辨率 `640×480`；
2. RMS 和每张图的重投影误差是否稳定；
3. 去畸变后画面边缘直线是否仍明显弯曲；
4. 主点 `cx/cy` 是否离画面中心异常远；
5. 焦距 `fx/fy` 是否为合理正数且量级接近。

畸变系数异常大的常见原因：棋盘行列写反、内角点数量错误、图像只覆盖中心、棋盘不平、画面模糊、分辨率不一致或自动对焦变化。

> 重新生成内参后，旧 `samples.yaml` 中的 PnP 位姿已经失效，必须重新采集手眼样本。

## 4. 采集手眼样本

### 4.1 前提

- 棋盘固定在基座环境中，整个采样过程不能移动；
- 相机刚性安装在机械臂末端；
- `calib/camera_intrinsics.yaml` 是当前相机、当前分辨率的结果；
- 默认 `INPUT_MODE = "ros"`，从 `/arm/pose` 获取末端位姿。

### 4.2 运行

```bash
# 正常模式，推荐
python calib/collect_samples.py

# 极简采集只用于定位相机问题；会跳过采集质量门禁
python calib/collect_samples.py --minimal
```

ROS 模式下，机械臂停止后按回车采集。正常模式会检查：

| 检查项 | 建议/门限 |
|---|---|
| 方格像素大小 | 建议 `≥20 px/格` |
| PnP 重投影误差 | 默认不超过脚本门限 |
| 清晰度 | 建议 Laplacian `≥120` |
| 亚像素角点 RMS | 建议 `≤0.2 px` |
| 帧与位姿时间差 | 默认 `≤80 ms` |
| 机械臂稳定性 | 停稳后采集 |

采样策略：

1. 采集 20～30 组；
2. 在安全范围内绕两个不同方向分别倾斜约 `±5°～10°`；
3. 配合少量 XYZ 位置变化；
4. 最大相对旋转建议 `≥8°`；
5. 双轴比建议 `≥0.05`；
6. 不要重复采集几乎相同的姿态。

只做平移或只绕一个轴旋转，无法充分观测手眼旋转和平移。

### 4.3 翻转保护

普通棋盘格存在 180° 对称性。程序检测到相邻 `target_to_camera` 旋转变化超过 120° 时，会拒绝当前样本。

出现提示后：

1. 不要移动棋盘；
2. 不要手工修改 `samples.yaml` 矩阵；
3. 将机械臂改到相邻、幅度较小的姿态重新采集。

输出：`calib/samples.yaml`。

## 5. 诊断、求解和验证

### 5.1 诊断

```bash
python calib/diagnose.py calib/samples.yaml
```

重点看：

- 是否存在姿态跳变或疑似翻转；
- 相机到棋盘的距离与 `px/格`；
- 机械臂旋转是否覆盖至少两个方向；
- `angle(A)` 与 `angle(B)` 是否一致；
- 不同快速求解方法是否给出接近结果。

诊断失败时先补采或重采，不要通过放宽求解门限强行输出。

### 5.2 求解

```bash
# 推荐
python calib/solve.py calib/samples.yaml

# 可选：像素级 Bundle Adjustment
python calib/solve.py calib/samples.yaml --ba

# 极简求解：全部样本直接交给 OpenCV，不筛选、不做 RANSAC
python calib/solve.py calib/samples.yaml --minimal
```

推荐模式包含角点网格完整性检查、重投影/同步硬门禁、样本级 RANSAC、
全相对运动求解和鲁棒精化。质量筛选后至少需要 6 个样本。误差指标会
完整写入结果供人工判断，但不会因为超过 `10 mm / 3°` 阻止保存。

结果：`calib/samples_result.yaml`。

### 5.3 验证

```bash
python calib/verify.py calib/samples.yaml \
    --result calib/samples_result.yaml
```

建议参考：

| 指标 | 较好 | 可接受 |
|---|---:|---:|
| 平移 RMS | `<5 mm` | `<10 mm` |
| 旋转 RMS | `<1°` | `<3°` |

这只是数据内部一致性指标，最终仍需在画面中心和边缘、不同工作层做空载实测。

## 6. 部署到机器人

从项目根目录同步结果：

```bash
# rsync — 增量同步 (推荐)
rsync -avz calib/samples_result.yaml \
    wheeltec@192.168.0.100:screw_pick/calib/

# 或者用 scp — 单文件拷贝
scp calib/samples_result.yaml \
    wheeltec@192.168.0.100:screw_pick/calib/
```

内参改变时一并同步：

```bash
rsync -avz calib/camera_intrinsics.yaml \
    wheeltec@192.168.0.100:screw_pick/calib/
```

新版脚本会自动同步通过质量门禁的内参，并且只在运行配置内参与采集内参
完全一致时自动部署手眼结果。手眼结果内还保存了内参绑定；Bridge 默认拒绝
旧版未绑定结果或内参不匹配结果。

部署后仍需重新构建；运行时读取安装空间中的相对配置路径。

```bash
cp calib/camera_intrinsics.yaml handeye_bridge/config/
cp calib/samples_result.yaml handeye_bridge/config/

cd ~/ros2_ws
colcon build --packages-select handeye_bridge
source install/setup.bash
```

## 7. SSH 显示转发 (X11 Forwarding)

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

查看截图：

```bash
scp wheeltec@192.168.0.100:screw_pick/calib/debug/calib_debug.jpg .
```

## 8. 常见问题

### 平移 RMS 很大

按顺序检查：棋盘是否移动、内参与样本是否同一版本、棋盘尺寸是否正确、`/arm/pose` 是否与采样时刻匹配、是否有足够的双轴旋转。

### 第二天结果发生变化

检查相机固定是否松动、机械臂零位是否一致、焦距/自动对焦是否变化、棋盘或工作平台是否移动。手眼标定默认相机到末端是刚性不变关系。

### 求解提示旋转不可观测

增加另一个方向的倾斜。继续增加平移或同轴旋转样本不能解决问题。

### 新结果部署后方向错误

确认部署的是 `^gripper T_camera` 格式结果，并同步部署当前 `handeye_bridge` 代码后重新构建。不要只替换安装目录中的单个 YAML。
