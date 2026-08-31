#!/usr/bin/env python3
"""
视觉检测 — 服务客户端

向 /vision_detect 服务发送控制指令:
  1. 发送开启指令 (start=true)  → 服务端开始持续循环检测
  2. 等待用户按 Enter 结束
  3. 发送停止指令 (start=false) → 服务端停止检测，返回最新结果

监听模式 (--monitor):
  等待期间实时订阅 /vision_detections 话题，打印每一帧的检测结果。

用法:
  ros2 run vison_topic vision_detect_client               # 基础模式
  ros2 run vison_topic vision_detect_client --monitor     # 实时监听模式
  ros2 run vison_topic vision_detect_client --listen-only # 纯监听（不控制启停）
"""

import argparse
import select
import sys
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# ── 导入自定义服务类型 ───────────────────────────────────────────
from vison_topic_interfaces.srv import VisionDetect


# ── 话题数据解析 ────────────────────────────────────────────────

def parse_topic_data(data: List[float]) -> Tuple[int, List[Tuple[int, float, float, float]]]:
    """解析 Float32MultiArray → (count, [(cls_id, u, v, conf), ...])

    data 布局 (原始像素模式):
      data[0]           = N    检测数量
      data[1 + 4*i]     = cls_id
      data[2 + 4*i]     = u (像素)
      data[3 + 4*i]     = v (像素)
      data[4 + 4*i]     = conf (置信度)
    """
    if len(data) < 1:
        return 0, []

    count = int(data[0])
    if count == 0:
        return 0, []

    detections: List[Tuple[int, float, float, float]] = []
    for i in range(count):
        offset = 1 + i * 4
        if offset + 3 >= len(data):
            break
        cls_id = int(data[offset])
        u = data[offset + 1]
        v = data[offset + 2]
        conf = data[offset + 3]
        detections.append((cls_id, u, v, conf))
    return count, detections


# ── 客户端类 ────────────────────────────────────────────────────

class VisionDetectClient(Node):
    """视觉检测服务客户端

    模式:
      - 默认: 发送启停指令，等待期间阻塞
      - monitor: 发送启停指令，等待期间订阅话题实时打印检测结果
      - listen-only: 不发送指令，仅订阅话题打印结果
    """

    def __init__(self, service_name: str = "vision_detect",
                 topic_name: str = "vision_detections",
                 monitor: bool = False,
                 listen_only: bool = False):
        super().__init__("vision_detect_client")

        self._monitor = monitor or listen_only
        self._listen_only = listen_only
        self._topic_name = topic_name
        self._latest_msg: Optional[Float32MultiArray] = None

        # ── 话题订阅（监听模式） ──
        if self._monitor:
            self._sub = self.create_subscription(
                Float32MultiArray,
                topic_name,
                self._on_topic_msg,
                10,
            )
            self.get_logger().info(f"已订阅话题 /{topic_name}")

        # ── 服务客户端（非纯监听模式） ──
        if not self._listen_only:
            self._client = self.create_client(VisionDetect, service_name)
            self.get_logger().info(f"等待服务 /{service_name} 就绪...")

            while not self._client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info("服务尚未就绪，继续等待...")

            self.get_logger().info(f"已连接到 /{service_name}")
        else:
            self._client = None

    # ── 话题回调 ────────────────────────────────────────────

    def _on_topic_msg(self, msg: Float32MultiArray) -> None:
        """话题回调：缓存最新消息"""
        self._latest_msg = msg

    def _print_topic_data(self) -> None:
        """打印最新话题数据"""
        msg = self._latest_msg
        if msg is None:
            return
        count, detections = parse_topic_data(msg.data)
        if count == 0:
            return

        print(f"\r  [{count}个]", end="")
        for cls_id, u, v, conf in detections:
            print(f" | cls{cls_id} u={u:.0f} v={v:.0f} c={conf:.2f}", end="")
        print(end="\r" if count > 0 else "\n")

    # ── 服务调用 ────────────────────────────────────────────

    def call_service(self, start: bool) -> VisionDetect.Response:
        """调用检测服务"""
        if self._client is None:
            return None

        request = VisionDetect.Request()
        request.start = start

        action = "开启" if start else "停止"
        self.get_logger().info(f">>> 发送{action}指令 (start={start})...")

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f"服务调用失败 ({action})")
            return None

        return future.result()

    # ── 等待用户输入（不阻塞 ROS2 spin） ─────────────────────

    def wait_for_enter(self) -> None:
        """等待用户按 Enter，同时持续 spin 以处理话题消息"""
        print("持续检测运行中... 按 Enter 停止检测\n")
        try:
            while rclpy.ok():
                # 非阻塞检查 stdin
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    sys.stdin.readline()
                    break

                # 处理话题消息
                rclpy.spin_once(self, timeout_sec=0.01)

                # 打印最新数据
                if self._monitor:
                    self._print_topic_data()

        except (KeyboardInterrupt, EOFError):
            print("\n")

    # ── 输出 ────────────────────────────────────────────────

    @staticmethod
    def print_response(response: VisionDetect.Response) -> None:
        """格式化输出服务响应"""
        if response is None:
            print("[ERROR] 无响应")
            return

        print(f"\n{'='*50}")
        print(f"  状态: {'✓ 成功' if response.success else '✗ 失败'}")
        print(f"  消息: {response.message}")

        if response.success and response.count > 0:
            print(f"  检测目标数: {response.count}")
            print(f"  {'─'*40}")
            for i in range(response.count):
                cls_id = response.cls_ids[i] if i < len(response.cls_ids) else -1
                u = response.u_px[i] if i < len(response.u_px) else 0.0
                v = response.v_px[i] if i < len(response.v_px) else 0.0
                name = (
                    response.cls_names[i]
                    if i < len(response.cls_names)
                    else f"cls_{cls_id}"
                )
                print(f"  [{i+1}] {name:8s}  "
                      f"u={u:8.1f} px  v={v:8.1f} px")
        elif response.success and response.count == 0:
            print("  检测目标数: 0 (无目标)")

        print(f"{'='*50}\n")


# ── 入口 ───────────────────────────────────────────────────────

def main() -> None:
    """入口"""
    parser = argparse.ArgumentParser(description="视觉检测 - 服务客户端")
    parser.add_argument(
        "--service", default="vision_detect",
        help="服务名称 (默认: vision_detect)"
    )
    parser.add_argument(
        "--topic", default="vision_detections",
        help="监听的话题名 (默认: vision_detections)"
    )
    parser.add_argument(
        "--monitor", action="store_true",
        help="实时监听模式: 等待期间订阅话题并打印检测结果"
    )
    parser.add_argument(
        "--listen-only", action="store_true",
        help="纯监听模式: 仅订阅话题，不发送启停指令"
    )
    args = parser.parse_args()

    rclpy.init(args=None)
    client = VisionDetectClient(
        service_name=args.service,
        topic_name=args.topic,
        monitor=args.monitor,
        listen_only=args.listen_only,
    )

    try:
        if args.listen_only:
            # ── 纯监听模式 ──
            print(f"纯监听模式 — 订阅 /{args.topic}，按 Ctrl+C 退出\n")
            while rclpy.ok():
                rclpy.spin_once(client, timeout_sec=0.1)
                client._print_topic_data()
        else:
            # ── 第 1 步：发送开启指令 ──
            response = client.call_service(start=True)
            client.print_response(response)

            if response is None or not response.success:
                print("[ERROR] 开启检测失败，退出")
                return

            # ── 第 2 步：等待用户结束（monitor 模式实时打印话题数据） ──
            client.wait_for_enter()

            # ── 第 3 步：发送停止指令 ──
            response = client.call_service(start=False)
            client.print_response(response)

    except KeyboardInterrupt:
        print("\n[INFO] 收到中断信号，发送停止指令...")
        response = client.call_service(start=False)
        client.print_response(response)
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
