"""
Atlas 完整导航后端适配器

该节点把 atlas_autonomous_transport_manager 的 StartNavigation/CancelNavigation 服务转换为
Nav2 NavigateToPose action；这样任务状态机不用直接依赖 Nav2 的 action 细节，
后续也可以继续保留 pseudo 后端作为无导航实车安全联调方案
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from atlas_mission_interfaces.msg import NavigationStatus
from atlas_mission_interfaces.srv import CancelNavigation, StartNavigation
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """把平面 yaw 角转换为 ROS 四元数"""
    half = 0.5 * float(yaw)
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    """从四元数中提取 yaw；这里只用于 odom 平面位姿"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """归一化到 [-pi, pi)，便于日志和状态误差阅读"""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class FullNavBackend(Node):
    """任务状态机使用的 Nav2 NavigateToPose 后端"""

    def __init__(self) -> None:
        super().__init__('atlas_nav_full_backend')

        self.backend_name = str(self.declare_parameter('backend_name', 'full').value)
        self.status_topic = str(self.declare_parameter('status_topic', '/atlas/navigation/status').value)
        self.start_service = str(self.declare_parameter('start_service', '/atlas/navigation/start').value)
        self.cancel_service = str(self.declare_parameter('cancel_service', '/atlas/navigation/cancel').value)
        self.action_name = str(self.declare_parameter('action_name', 'navigate_to_pose').value)
        self.coordinate_mode = str(self.declare_parameter('coordinate_mode', 'task_relative_odom').value)
        self.odom_topic = str(self.declare_parameter('odom_topic', '/odom').value)
        self.odom_frame = str(self.declare_parameter('odom_frame', 'odom').value)
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.action_server_timeout_s = float(self.declare_parameter('action_server_timeout_s', 3.0).value)
        self.status_publish_rate_hz = float(self.declare_parameter('status_publish_rate_hz', 5.0).value)
        self.unknown_distance_error_m = float(self.declare_parameter('unknown_distance_error_m', -1.0).value)

        self.status_pub = self.create_publisher(NavigationStatus, self.status_topic, 10)
        self.start_srv = self.create_service(StartNavigation, self.start_service, self.on_start)
        self.cancel_srv = self.create_service(CancelNavigation, self.cancel_service, self.on_cancel)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.nav_action = ActionClient(self, NavigateToPose, self.action_name)
        self.status_timer = self.create_timer(1.0 / max(1.0, self.status_publish_rate_hz), self.on_status_timer)

        self.latest_odom: Optional[Odometry] = None
        self.task_origin: Optional[Tuple[float, float, float]] = None
        self.active_goal_handle = None
        self.result_future = None
        self.active_waypoint_id = ''
        self.active_target_x_m = 0.0
        self.active_target_y_m = 0.0
        self.active_target_yaw_rad = 0.0
        self.active_timeout_s = 0.0
        self.active_started_at: Optional[Time] = None
        self.distance_error_m = self.unknown_distance_error_m
        self.angle_error_rad = 0.0
        self.state = NavigationStatus.STATE_IDLE
        self.error_code = 0
        self.message = '空闲'

        self.get_logger().info(
            f'完整导航后端已启动 backend={self.backend_name} action={self.action_name} mode={self.coordinate_mode}'
        )

    def on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def current_odom_pose(self) -> Optional[Tuple[float, float, float]]:
        if self.latest_odom is None:
            return None
        p = self.latest_odom.pose.pose.position
        yaw = quaternion_to_yaw(self.latest_odom.pose.pose.orientation)
        return float(p.x), float(p.y), yaw

    def build_goal_pose(self, request: StartNavigation.Request) -> Optional[PoseStamped]:
        """根据配置把任务点转换为 Nav2 goal pose"""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()

        if self.coordinate_mode == 'absolute_map':
            goal.header.frame_id = self.map_frame
            goal.pose.position.x = float(request.x_m)
            goal.pose.position.y = float(request.y_m)
            goal.pose.position.z = 0.0
            goal.pose.orientation = yaw_to_quaternion(float(request.yaw_rad))
            return goal

        if self.coordinate_mode != 'task_relative_odom':
            self.fail(1101, f'不支持的坐标模式: {self.coordinate_mode}')
            return None

        odom_pose = self.current_odom_pose()
        if odom_pose is None:
            self.fail(1102, '尚未收到 /odom，无法建立任务相对坐标目标')
            return None

        if request.reset_origin or self.task_origin is None:
            self.task_origin = odom_pose
            self.get_logger().info(
                f'记录完整导航任务原点 odom=[{odom_pose[0]:.3f}, {odom_pose[1]:.3f}, {odom_pose[2]:.3f}]'
            )

        origin_x, origin_y, origin_yaw = self.task_origin
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)
        rel_x = float(request.x_m)
        rel_y = float(request.y_m)
        target_x = origin_x + cos_yaw * rel_x - sin_yaw * rel_y
        target_y = origin_y + sin_yaw * rel_x + cos_yaw * rel_y
        target_yaw = normalize_angle(origin_yaw + float(request.yaw_rad))

        goal.header.frame_id = self.odom_frame
        goal.pose.position.x = target_x
        goal.pose.position.y = target_y
        goal.pose.position.z = 0.0
        goal.pose.orientation = yaw_to_quaternion(target_yaw)
        return goal

    def on_start(self, request: StartNavigation.Request, response: StartNavigation.Response):
        if request.backend and request.backend != self.backend_name:
            response.success = False
            response.message = f'后端不匹配，请求 {request.backend}，当前 {self.backend_name}'
            return response
        if self.state == NavigationStatus.STATE_RUNNING:
            response.success = False
            response.message = '完整导航后端正在执行目标，请先取消当前目标'
            return response
        if not self.nav_action.wait_for_server(timeout_sec=max(0.1, self.action_server_timeout_s)):
            response.success = False
            response.message = f'Nav2 action server 未就绪: {self.action_name}'
            self.fail(1103, response.message)
            return response

        goal_pose = self.build_goal_pose(request)
        if goal_pose is None:
            response.success = False
            response.message = self.message
            return response

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        goal.behavior_tree = ''

        self.active_waypoint_id = request.waypoint_id
        self.active_target_x_m = float(goal_pose.pose.position.x)
        self.active_target_y_m = float(goal_pose.pose.position.y)
        self.active_target_yaw_rad = quaternion_to_yaw(goal_pose.pose.orientation)
        self.active_timeout_s = float(request.timeout_s) if request.timeout_s > 0.0 else 0.0
        self.active_started_at = self.get_clock().now()
        self.distance_error_m = self.unknown_distance_error_m
        self.angle_error_rad = 0.0
        self.error_code = 0
        self.state = NavigationStatus.STATE_RUNNING
        self.message = 'Nav2 目标已发送，等待 Nav2 接受'
        self.publish_status()

        send_future = self.nav_action.send_goal_async(goal, feedback_callback=self.on_feedback)
        send_future.add_done_callback(self.on_goal_response)

        response.success = True
        response.message = '完整导航目标已提交给 Nav2'
        self.get_logger().info(
            f'提交完整导航目标 waypoint={request.waypoint_id} frame={goal_pose.header.frame_id} '
            f'x={goal_pose.pose.position.x:.3f} y={goal_pose.pose.position.y:.3f} yaw={request.yaw_rad:.3f}'
        )
        return response

    def on_cancel(self, request: CancelNavigation.Request, response: CancelNavigation.Response):
        reason = request.reason or '收到取消请求'
        if self.active_goal_handle is not None:
            cancel_future = self.active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _: self.get_logger().info('Nav2 cancel 请求已发送'))
        self.state = NavigationStatus.STATE_CANCELLED
        self.message = reason
        self.error_code = 0
        self.active_goal_handle = None
        self.result_future = None
        response.success = True
        response.message = reason
        self.publish_status()
        return response

    def on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.fail(1104, f'Nav2 goal 请求异常: {exc}')
            return
        if not goal_handle.accepted:
            self.fail(1105, 'Nav2 拒绝导航目标')
            return
        self.active_goal_handle = goal_handle
        self.state = NavigationStatus.STATE_RUNNING
        self.message = 'Nav2 正在执行导航目标'
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.on_result)
        self.publish_status()

    def on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        try:
            self.distance_error_m = float(feedback.distance_remaining)
        except Exception:  # noqa: BLE001
            self.distance_error_m = self.unknown_distance_error_m
        self.message = f'Nav2 运行中，剩余距离={self.distance_error_m:.3f} m'

    def on_result(self, future) -> None:
        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as exc:  # noqa: BLE001
            self.fail(1106, f'Nav2 结果异常: {exc}')
            return

        self.active_goal_handle = None
        self.result_future = None
        # action_msgs/GoalStatus: 4=SUCCEEDED，5=CANCELED，6=ABORTED
        if status == 4:
            self.state = NavigationStatus.STATE_SUCCEEDED
            self.error_code = 0
            self.message = '完整导航完成'
        elif status == 5:
            self.state = NavigationStatus.STATE_CANCELLED
            self.error_code = 0
            self.message = '完整导航已取消'
        else:
            self.state = NavigationStatus.STATE_FAILED
            self.error_code = 1107
            self.message = f'完整导航失败，Nav2 status={status}'
        self.publish_status()

    def on_status_timer(self) -> None:
        if self.state == NavigationStatus.STATE_RUNNING:
            if self.active_timeout_s > 0.0 and self.active_started_at is not None:
                elapsed = (self.get_clock().now() - self.active_started_at).nanoseconds * 1e-9
                if elapsed > self.active_timeout_s:
                    if self.active_goal_handle is not None:
                        self.active_goal_handle.cancel_goal_async()
                    self.fail(1108, f'完整导航超时 {elapsed:.1f}s > {self.active_timeout_s:.1f}s')
                    return
        self.publish_status()

    def fail(self, code: int, message: str) -> None:
        self.state = NavigationStatus.STATE_FAILED
        self.error_code = int(code)
        self.message = str(message)
        self.active_goal_handle = None
        self.result_future = None
        self.get_logger().error(self.message)
        self.publish_status()

    def publish_status(self) -> None:
        msg = NavigationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.backend = self.backend_name
        msg.waypoint_id = self.active_waypoint_id
        msg.target_x_m = float(self.active_target_x_m)
        msg.target_y_m = float(self.active_target_y_m)
        msg.target_yaw_rad = float(self.active_target_yaw_rad)
        msg.distance_error_m = float(self.distance_error_m)
        msg.angle_error_rad = float(self.angle_error_rad)
        msg.error_code = int(self.error_code)
        msg.message = self.message
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FullNavBackend()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
