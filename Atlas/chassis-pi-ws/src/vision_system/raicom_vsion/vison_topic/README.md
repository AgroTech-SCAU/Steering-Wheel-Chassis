## Atlas 当前集成说明

本包是 RACOM/RAICOM 检测服务源码目录，目录名 `raicom_vsion` 和包名 `vison_topic` 沿用历史拼写。Atlas 当前默认通过 `atlas_racom_vision_backend` 调用本包的 `/vision_detect` 服务，再适配成 `/vision/detect_camera_target` 给任务状态机使用。

树莓派无显示或远程运行时，推荐关闭预览窗口：

```bash
ros2 run vison_topic vision_detect_server --camera 0 --no-preview
```

常用参数：

```text
--camera             摄像头编号
--no-preview         不打开 OpenCV 预览窗口
--conf               置信度阈值
--process-every-n    每 N 帧推理一次，用于降低 PI 端负载
--rate-hz            主循环频率上限
--service-name       检测启停服务名，默认 vision_detect
--topic-name         检测结果话题名，默认 vision_detections
```

---

## 架构

```
                        ┌─ /vision_detect (服务) ──┐
   其他模块 ──────────────┤  start=true  开始持续检测  ├──→ vision_detect_server
   (C++/Python)          │  start=false 停止并返结果 │    (detect_and_send.py)
                         └─────────────────────────┘         │
                                                      ┌──────┴──────┐
                        ┌─ /vision_detections (话题) ─┤  摄像头+ONNX  │
   其他模块 ── 订阅 ────┤  Float32MultiArray 实时数据  │  运动学变换   │
                        └─────────────────────────────└─────────────┘
```

| 接口 | 类型 | 方向 | 用途 |
|------|------|------|------|
| `/vision_detect` | `vison_topic_interfaces/srv/VisionDetect` | 请求→响应 | **控制面**：启停检测 |
| `/vision_detections` | `std_msgs/Float32MultiArray` | 发布→订阅 | **数据面**：实时检测结果流 |

## 功能

| 模块 | 说明 |
|------|------|
| `detect_and_send.py` | 服务端：等待启停指令，持续采集 → ONNX 推理 → 坐标变换 → 话题发布 |
| `vision_receiver.py` | 客户端：发送启停指令，可选实时监听话题数据 |

## 依赖

- **ROS2** (rclpy, std_msgs, ament_index_python)
- **vison_topic_interfaces** (自定义服务接口包)
- **ONNX Runtime** (`pip install onnxruntime`)
- **OpenCV** (`pip install opencv-python`)
- **NumPy, PyYAML**

## 构建

```bash
cd ~/RK_Vison/RK_Vison_WS

# 先构建接口包，再构建主包
colcon build --packages-select vison_topic_interfaces vison_topic
source install/setup.bash
```

## 快速开始

```bash
# 终端 1：启动服务端（PC 版）
ros2 run vison_topic vision_detect_server

# 树莓派无显示或远程模式
ros2 run vison_topic vision_detect_server --camera 0 --no-preview --process-every-n 2

# 终端 2：发送启停指令
ros2 run vison_topic vision_detect_client

# 实时监听模式（等待期间打印话题数据）
ros2 run vison_topic vision_detect_client --monitor

# 终端 3（可选）：纯监听
ros2 topic echo /vision_detections
```

---

## 其他模块集成指南

### 1. Python 模块 — 发送启停指令

```python
import rclpy
from rclpy.node import Node
from vison_topic_interfaces.srv import VisionDetect

class MyModule(Node):
    def __init__(self):
        super().__init__("my_module")
        self._client = self.create_client(VisionDetect, "vision_detect")

        # 等待服务就绪
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待视觉检测服务...")

    def start_detection(self):
        """开启持续检测"""
        req = VisionDetect.Request()
        req.start = True
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp and resp.success:
            self.get_logger().info(f"检测已启动: {resp.message}")

    def stop_detection(self):
        """停止检测，获取最后帧结果"""
        req = VisionDetect.Request()
        req.start = False
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp and resp.success:
            for i in range(resp.count):
                self.get_logger().info(
                    f"[{resp.cls_names[i]}] "
                    f"u={resp.u_px[i]:.1f}px v={resp.v_px[i]:.1f}px"
                )
```

### 2. C++ 模块 — 发送启停指令

```cpp
#include <rclcpp/rclcpp.hpp>
#include <vison_topic_interfaces/srv/vision_detect.hpp>

class MyModule : public rclcpp::Node {
public:
    MyModule() : Node("my_module") {
        client_ = this->create_client<vison_topic_interfaces::srv::VisionDetect>(
            "vision_detect");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "等待视觉检测服务...");
        }
    }

    void start_detection() {
        auto req = std::make_shared<
            vison_topic_interfaces::srv::VisionDetect::Request>();
        req->start = true;
        auto future = client_->async_send_request(req);
        if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            auto resp = future.get();
            RCLCPP_INFO(get_logger(), "检测已启动: %s", resp->message.c_str());
        }
    }

    // stop 同理，req->start = false
private:
    rclcpp::Client<vison_topic_interfaces::srv::VisionDetect>::SharedPtr client_;
};
```

### 3. Python 模块 — 订阅话题实时数据

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DetectionListener(Node):
    def __init__(self):
        super().__init__("detection_listener")
        self._sub = self.create_subscription(
            Float32MultiArray,
            "vision_detections",
            self._on_detections,
            10,  # QoS: 只保留最新 10 条
        )

    def _on_detections(self, msg: Float32MultiArray):
        """解析检测数据"""
        data = msg.data
        if len(data) < 1:
            return

        count = int(data[0])          # 检测数量
        if count == 0:
            return

        for i in range(count):
            offset = 1 + i * 4
            if offset + 3 >= len(data):
                break
            cls_id = int(data[offset])
            u_px = data[offset + 1]     # 检测框底部中心 u 像素
            v_px = data[offset + 2]     # 检测框底部中心 v 像素
            conf = data[offset + 3]     # 置信度

            self.get_logger().info(
                f"检测: cls={cls_id}  u={u_px:.1f}px  v={v_px:.1f}px  conf={conf:.2f}"
            )
```

### 4. C++ 模块 — 订阅话题实时数据

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class DetectionListener : public rclcpp::Node {
public:
    DetectionListener() : Node("detection_listener") {
        sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "vision_detections", 10,
            std::bind(&DetectionListener::on_detections, this,
                      std::placeholders::_1));
    }

private:
    void on_detections(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        const auto& data = msg->data;
        if (data.empty()) return;

        int count = static_cast<int>(data[0]);  // 检测数量
        for (int i = 0; i < count; ++i) {
            int offset = 1 + i * 4;
            if (offset + 3 >= static_cast<int>(data.size())) break;

            int cls_id = static_cast<int>(data[offset]);
            float u_px = data[offset + 1];     // 检测框底部中心 u 像素
            float v_px = data[offset + 2];     // 检测框底部中心 v 像素
            float conf = data[offset + 3];     // 置信度

            RCLCPP_INFO(get_logger(),
                "检测: cls=%d  u=%.1fpx  v=%.1fpx  conf=%.2f", cls_id, u_px, v_px, conf);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
};
```

### 5. 话题数据布局

```
Float32MultiArray.data 布局:

data[0]          = N        检测目标数量
data[1 + 4×i]    = cls_id   类别 ID  (0=螺丝, 1=齿轮, …)
data[2 + 4×i]    = u_px     检测框底部中心 u 像素
data[3 + 4×i]    = v_px     检测框底部中心 v 像素
data[4 + 4×i]    = conf     置信度
```

**示例** — 检测到 1 个螺丝和 1 个齿轮：

```
data = [2.0,  0.0, 320.5, 512.0, 0.91,  1.0, 240.0, 500.0, 0.86]
         N    cls   u      v      conf   cls   u      v      conf
              └──── 螺丝 ────────┘       └──── 齿轮 ────────┘
```

### 6. 命令行手动控制

```bash
# 开始检测
ros2 service call /vision_detect vison_topic_interfaces/srv/VisionDetect "{start: true}"

# 停止检测（返回最后帧结果）
ros2 service call /vision_detect vison_topic_interfaces/srv/VisionDetect "{start: false}"

# 查看实时数据
ros2 topic echo /vision_detections
```

---

## 服务接口

| 服务 | 类型 | 说明 |
|------|------|------|
| `/vision_detect` | `vison_topic_interfaces/srv/VisionDetect` | 启停控制 |

### 请求

```
bool start          # true = 开始持续检测, false = 停止并返回最后帧结果
```

### 响应

```
int32 count                         # 检测到的目标数量
int32[] cls_ids                     # 类别 ID (0=螺丝, 1=齿轮, …)
float64[] u_px                      # 检测框底部中心 u 像素
float64[] v_px                      # 检测框底部中心 v 像素
string[] cls_names                  # 类别名称
string message                      # 状态消息
bool success                        # 是否成功
```

---

## 配置

编辑 `vison_topic/config.yaml`。**PC 端和树莓派的关键差异**：

```yaml
# ── PC 端配置 ──
camera:
  width: 640
  height: 640
  fps: 30

yolo:
  onnx_path: "/home/yuuy/RK_Vison/RK_Vison_WS/src/vison_topic/resource/best.onnx"
  conf_threshold: 0.7

service:
  rate_hz: 30            # 检测频率

optimization:
  onnx_threads: 4        # PC 可用更多线程
  process_every_n: 1     # 不跳帧
  publish_rate_hz: 0     # 不限制发布频率

display:
  show_preview: true
  print_coords: false

# ── 树莓派 4B / 4GB 推荐配置 ──
camera:
  width: 320             # 降低分辨率 →
  height: 320            # 减少 ONNX 输入尺寸，大幅降低内存
  fps: 15

yolo:
  conf_threshold: 0.6    # 稍降低以适应低分辨率

service:
  rate_hz: 10            # 检测频率降低

optimization:
  onnx_threads: 1        # 单线程，减少内存争用
  process_every_n: 2     # 每 2 帧处理 1 帧
  publish_rate_hz: 10    # 发布频率 ≤ 10Hz

display:
  show_preview: false    # 关闭预览节省 ~100MB 内存
  print_coords: false    # 关闭终端输出减少 IO
```

或直接使用 `--rpi` 标志自动覆盖为树莓派优化值：

```bash
ros2 run vison_topic vision_detect_server --rpi
```

---

## 树莓派性能调优

### 操作系统

```bash
# 增加 swap (4GB 内存紧张时)
sudo dphys-swapfile swapoff
sudo sed -i 's/CONF_SWAPSIZE=.*/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
sudo dphys-swapfile swapon

# GPU 内存降至最低 (不需要桌面)
sudo raspi-config  →  Performance Options  →  GPU Memory  →  16MB

# 关闭不必要的服务
sudo systemctl disable bluetooth.service
sudo systemctl disable avahi-daemon.service
```

### ONNX 模型优化

```bash
# 使用 ONNX 工具量化模型 (INT8)，可减少 ~70% 推理内存
pip install onnxruntime-tools
python3 -m onnxruntime.tools.convert_onnx_float32_to_float16 \
    best.onnx best_fp16.onnx
```

### 内存监控

服务端运行时会定期打印内存占用（Linux）：

```
[运行中] 处理FPS:8  总帧:120  当前:2个  推理:45ms  内存:380MB
```

或用外部工具：

```bash
watch -n 1 "ps aux | grep vision_detect_server | grep -v grep"
htop -p $(pgrep -f vision_detect_server)
```

---

## 目录结构

```
RK_Vison_WS/src/
├── vison_topic_interfaces/        # 接口定义包 (ament_cmake)
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── srv/VisionDetect.srv
└── vison_topic/                   # 实现包 (ament_python)
    ├── README.md
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   ├── vison_topic
    │   ├── best.onnx
    │   └── best.labels.txt
    └── vison_topic/
        ├── __init__.py
        ├── config.yaml
        ├── detect_and_send.py     # 服务端 (持续检测 + 话题发布)
        └── vision_receiver.py     # 客户端 (启停控制 + 监听)
```

---

## 坐标系

```
base_link (世界坐标系，机械臂基座)
  │
  ├── 正运动学 (URDF 5 关节)
  │
  └── camera (相机坐标系)
       │
       ├── 针孔模型: 像素 → 相机射线
       │
       └── 工作平面 (z=0.05m): 射线与平面交点 → 世界坐标
```

---

## 常见问题

**Q: 启动报 `FileNotFoundError: config.yaml`**
确保已 `colcon build` 并 `source install/setup.bash`。

**Q: `NoSuchFile: best.onnx`**
检查 `config.yaml` 中 `onnx_path` 是否为正确的绝对路径。

**Q: 服务调用报 `Service not available`**
确认服务端已启动，用 `ros2 service list | grep vision_detect` 检查。

**Q: 树莓派上跑不动 / 很卡**
1. 使用 `--rpi` 标志启动
2. 降低 `camera.width/height` 到 320
3. 设置 `optimization.process_every_n: 3`
4. 关闭 `display.show_preview`
5. 考虑使用 fp16 量化模型

**Q: 如何联合其他模块一起启动？**
使用 ROS2 launch 文件：

```python
# launch/vision_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="vison_topic", executable="vision_detect_server",
             arguments=["--rpi"], output="screen"),
        Node(package="your_other_module", executable="your_node",
             output="screen"),
    ])
```
