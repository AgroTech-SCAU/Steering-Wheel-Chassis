# handeye_calibration_tool v0.1.2

用于 `chassis-pi-ws` 的交互式单目手眼标定工具。UVC 相机通过 OpenCV `VideoCapture` 打开，终端完成配置、控制、采样和求解，画面显示在独立 OpenCV 窗口中。

## v0.1.2 修复

- 支持**普通黑白棋盘格**，不再要求必须使用 ChArUco；
- 默认标定板类型改为 `chessboard`；
- 保留 ChArUco 支持，可在菜单 3 中切换；
- 配置标定板时使用分步向导，明确说明每一步应该输入什么；
- 普通棋盘格参数使用**内角点数量**，不是黑白方格数量；
- 相机窗口分别显示：图案是否找到、内参是否加载、PnP 位姿是否有效；
- 即使未加载内参，也会绘制识别到的棋盘格角点，不再统一显示为“视觉无效”；
- 终端可直接输入 `c` 采样、`q` 退出；
- 保留 v0.1.1 的 `/dev/tty` 修复，可通过 `ros2 launch` 交互。

## 标定板选择

### 普通黑白棋盘格

如果标定板只有黑白方格，没有 ArUco 编码图案，菜单 3 选择：

```text
1. 普通黑白棋盘格
```

配置时需要输入：

1. 横向内角点数；
2. 纵向内角点数；
3. 单个方格实际边长，单位 m；
4. 最大重投影误差，初始建议 `1.5` px。

例如标定板有 `10 × 7` 个黑白方格，则内部交点为：

```text
9 × 6 内角点
```

如果方格边长为 25 mm，则输入：

```text
0.025
```

普通棋盘格具有 180° 对称性。采集时应保持角点编号方向一致，避免相机绕光轴旋转 180° 或让检测顺序翻转。若条件允许，后续可在固定角落增加明显方向标记，或者换用 ChArUco。

### ChArUco

棋盘格中带 ArUco 编码时，菜单 3 选择：

```text
2. ChArUco 标定板
```

ChArUco 的 `squares_x/squares_y` 是完整方格数量，不是内角点数量。

## 相机窗口状态

窗口左上角会显示：

```text
board: chessboard 9x6 INNER corners
pattern: FOUND 54/54
intrinsics: LOADED
pose: VALID
```

状态含义：

- `pattern: NOT FOUND`：棋盘格尺寸配置错误、棋盘格不完整、过远、过暗或模糊；
- `pattern: FOUND`：角点已经识别；
- `intrinsics: MISSING (menu 4)`：尚未加载相机内参；
- `pose: NOT READY`：不能采样，查看后面的具体原因；
- `pose: VALID`：视觉侧 `camera_T_target` 可用于采样。

手眼采样必须先加载相机内参。没有内参时仍会绘制角点，但不能通过 PnP 计算标定板位姿。

## 依赖

```bash
sudo apt update
sudo apt install -y \
  python3-opencv \
  libopencv-contrib-dev \
  python3-numpy \
  python3-yaml
```

工具依赖 `mcu_comm_bridge` 中的机械臂服务接口。

## 替换与构建

```bash
cd /home/wheeltec/wheeltec_ros2/src
rm -rf handeye_calibration_tool
unzip /压缩包所在路径/handeye_calibration_tool-fixed-v1.2.zip

cd /home/wheeltec/wheeltec_ros2
rm -rf build/handeye_calibration_tool install/handeye_calibration_tool
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select handeye_calibration_tool
source install/setup.bash
```

## 启动

```bash
ros2 launch handeye_calibration_tool handeye_tool.launch.py
```

启动日志应显示：

```text
[INFO] 终端输入源：/dev/tty
```

菜单输入方式：

```text
输入菜单编号并按 Enter
c：采样
q：退出
```

相机窗口获得焦点时也支持：

```text
C：采样
Q / Esc：退出
```

## 默认配置

默认按普通棋盘格启动：

```yaml
board_type: "chessboard"
chessboard_inner_corners_x: 9
chessboard_inner_corners_y: 6
chessboard_square_length_m: 0.025
```

必须按照你的实物重新测量和配置，不能直接使用默认尺寸。

## 数据目录

```text
~/handeye_calibration/时间戳/
├── config.yaml
├── samples.yaml
├── handeye_result.yaml
└── images/
```

每个样本保存：

- 标定板类型；
- 检测角点数量；
- 原始图像；
- 五个关节角；
- `base_T_gripper`；
- `camera_T_target`；
- PnP 重投影误差。

## 采集前检查

菜单 1 中至少应看到：

```text
相机内参       : 已加载
图案识别       : 已识别
PnP 位姿       : 有效
机械臂状态     : 有效
```

否则菜单 7 和快捷键 `C` 会拒绝采样并打印具体原因。
