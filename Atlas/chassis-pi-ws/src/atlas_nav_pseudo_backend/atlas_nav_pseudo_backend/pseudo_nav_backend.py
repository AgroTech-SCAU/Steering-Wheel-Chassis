#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from atlas_mission_interfaces.msg import NavigationStatus
from atlas_mission_interfaces.srv import StartNavigation, CancelNavigation


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class OdomPose:
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    stamp: rclpy.time.Time


@dataclass
class Target:
    waypoint_id: str
    x: float
    y: float
    yaw: float
    timeout_s: float
    start_time: rclpy.time.Time


class PseudoNavBackend(Node):
    def __init__(self) -> None:
        super().__init__('atlas_nav_pseudo_backend')
        self.backend_name = self.declare_parameter('backend_name', 'pseudo').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/atlas/navigation/cmd_vel').value
        self.status_topic = self.declare_parameter('status_topic', '/atlas/navigation/status').value
        self.start_service = self.declare_parameter('start_service', '/atlas/navigation/start').value
        self.cancel_service = self.declare_parameter('cancel_service', '/atlas/navigation/cancel').value

        self.control_rate_hz = float(self.declare_parameter('control_rate_hz', 50.0).value)
        self.odom_timeout_s = float(self.declare_parameter('odom_timeout_s', 0.30).value)
        self.default_waypoint_timeout_s = float(self.declare_parameter('default_waypoint_timeout_s', 20.0).value)
        self.kp_xy = float(self.declare_parameter('kp_xy', 0.8).value)
        self.kp_yaw = float(self.declare_parameter('kp_yaw', 1.5).value)
        self.max_linear_speed = float(self.declare_parameter('max_linear_speed_m_s', 0.20).value)
        self.max_angular_speed = float(self.declare_parameter('max_angular_speed_rad_s', 0.40).value)
        self.max_linear_accel = float(self.declare_parameter('max_linear_accel_m_s2', 0.30).value)
        self.max_angular_accel = float(self.declare_parameter('max_angular_accel_rad_s2', 0.80).value)
        self.slowdown_distance = float(self.declare_parameter('slowdown_distance_m', 0.25).value)
        self.position_tolerance = float(self.declare_parameter('position_tolerance_m', 0.04).value)
        self.yaw_tolerance = float(self.declare_parameter('yaw_tolerance_rad', 0.08).value)
        self.settle_linear_speed = float(self.declare_parameter('settle_linear_speed_m_s', 0.015).value)
        self.settle_angular_speed = float(self.declare_parameter('settle_angular_speed_rad_s', 0.03).value)
        self.settle_duration_s = float(self.declare_parameter('settle_duration_s', 0.50).value)
        self.settle_timeout_s = float(self.declare_parameter('settle_timeout_s', 2.0).value)

        self.latest_odom: Optional[OdomPose] = None
        self.origin: Optional[OdomPose] = None
        self.target: Optional[Target] = None
        self.state = NavigationStatus.STATE_IDLE
        self.message = 'idle'
        self.error_code = 0
        self.distance_error = 0.0
        self.yaw_error = 0.0
        self.last_cmd = Twist()
        self.last_update_time = self.get_clock().now()
        self.arrived_time: Optional[rclpy.time.Time] = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(NavigationStatus, self.status_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.start_srv = self.create_service(StartNavigation, self.start_service, self.on_start)
        self.cancel_srv = self.create_service(CancelNavigation, self.cancel_service, self.on_cancel)
        self.timer = self.create_timer(1.0 / max(1.0, self.control_rate_hz), self.on_timer)
        self.get_logger().info('pseudo navigation backend ready')

    def on_odom(self, msg: Odometry) -> None:
        stamp = rclpy.time.Time.from_msg(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else self.get_clock().now()
        self.latest_odom = OdomPose(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=yaw_from_quat(msg.pose.pose.orientation),
            vx=float(msg.twist.twist.linear.x),
            vy=float(msg.twist.twist.linear.y),
            wz=float(msg.twist.twist.angular.z),
            stamp=stamp,
        )

    def odom_fresh(self, now) -> bool:
        if self.latest_odom is None:
            return False
        return 0.0 <= (now - self.latest_odom.stamp).nanoseconds * 1e-9 <= self.odom_timeout_s

    def on_start(self, request: StartNavigation.Request, response: StartNavigation.Response):
        now = self.get_clock().now()
        if request.backend and request.backend != self.backend_name:
            response.success = False
            response.message = f"backend mismatch: requested {request.backend}, this is {self.backend_name}"
            return response
        if not self.odom_fresh(now):
            response.success = False
            response.message = 'no fresh odom'
            self.message = response.message
            self.error_code = 2002
            self.state = NavigationStatus.STATE_FAILED
            # 保留轻量目标对象，便于总状态机匹配点位编号
            self.target = Target(request.waypoint_id, 0.0, 0.0, 0.0, 0.0, now)
            self.publish_status()
            return response
        if self.state == NavigationStatus.STATE_RUNNING:
            response.success = False
            response.message = 'navigation is already running'
            self.message = response.message
            self.error_code = 2005
            self.publish_status()
            return response

        if request.reset_origin or self.origin is None:
            self.origin = self.latest_odom
            self.get_logger().info('task origin reset at x=%.3f y=%.3f yaw=%.3f', self.origin.x, self.origin.y, self.origin.yaw)

        assert self.origin is not None
        c = math.cos(self.origin.yaw)
        s = math.sin(self.origin.yaw)
        target_x = self.origin.x + c * request.x_m - s * request.y_m
        target_y = self.origin.y + s * request.x_m + c * request.y_m
        target_yaw = normalize_angle(self.origin.yaw + request.yaw_rad)
        timeout = request.timeout_s if request.timeout_s > 0.0 else self.default_waypoint_timeout_s
        self.target = Target(request.waypoint_id, target_x, target_y, target_yaw, timeout, now)
        self.state = NavigationStatus.STATE_RUNNING
        self.message = 'running'
        self.error_code = 0
        self.distance_error = 0.0
        self.yaw_error = 0.0
        self.arrived_time = None
        self.last_cmd = Twist()
        response.success = True
        response.message = 'navigation accepted'
        self.get_logger().info('start waypoint %s relative=(%.3f %.3f %.3f) target=(%.3f %.3f %.3f)', request.waypoint_id, request.x_m, request.y_m, request.yaw_rad, target_x, target_y, target_yaw)
        return response

    def on_cancel(self, request: CancelNavigation.Request, response: CancelNavigation.Response):
        reason = request.reason or 'cancel requested'
        self.stop_with_state(NavigationStatus.STATE_CANCELLED, reason, 0)
        response.success = True
        response.message = reason
        return response

    def stop_with_state(self, state: int, message: str, error_code: int) -> None:
        self.state = state
        self.message = message
        self.error_code = error_code
        self.target = None if state != NavigationStatus.STATE_SUCCEEDED else self.target
        self.arrived_time = None
        self.last_cmd = Twist()
        self.cmd_pub.publish(Twist())
        self.publish_status()

    def limit_accel(self, desired: Twist, dt: float) -> Twist:
        out = Twist()
        max_dv = self.max_linear_accel * max(0.001, dt)
        max_dw = self.max_angular_accel * max(0.001, dt)
        out.linear.x = clamp(desired.linear.x, self.last_cmd.linear.x - max_dv, self.last_cmd.linear.x + max_dv)
        out.linear.y = clamp(desired.linear.y, self.last_cmd.linear.y - max_dv, self.last_cmd.linear.y + max_dv)
        out.angular.z = clamp(desired.angular.z, self.last_cmd.angular.z - max_dw, self.last_cmd.angular.z + max_dw)
        return out

    def on_timer(self) -> None:
        now = self.get_clock().now()
        dt = max(0.001, (now - self.last_update_time).nanoseconds * 1e-9)
        self.last_update_time = now
        if self.state == NavigationStatus.STATE_RUNNING:
            self.update_running(now, dt)
        else:
            self.publish_status()

    def update_running(self, now, dt: float) -> None:
        if self.target is None:
            self.stop_with_state(NavigationStatus.STATE_FAILED, 'internal target missing', 2001)
            return
        if not self.odom_fresh(now):
            self.stop_with_state(NavigationStatus.STATE_FAILED, 'odom timeout', 2002)
            return
        if (now - self.target.start_time).nanoseconds * 1e-9 > self.target.timeout_s:
            self.stop_with_state(NavigationStatus.STATE_FAILED, 'waypoint timeout', 2003)
            return

        odom = self.latest_odom
        dx = self.target.x - odom.x
        dy = self.target.y - odom.y
        self.yaw_error = normalize_angle(self.target.yaw - odom.yaw)
        self.distance_error = math.hypot(dx, dy)
        ex_body = math.cos(odom.yaw) * dx + math.sin(odom.yaw) * dy
        ey_body = -math.sin(odom.yaw) * dx + math.cos(odom.yaw) * dy

        if self.distance_error <= self.position_tolerance and abs(self.yaw_error) <= self.yaw_tolerance:
            self.cmd_pub.publish(Twist())
            if (abs(odom.vx) <= self.settle_linear_speed and abs(odom.vy) <= self.settle_linear_speed and abs(odom.wz) <= self.settle_angular_speed):
                if self.arrived_time is None:
                    self.arrived_time = now
                if (now - self.arrived_time).nanoseconds * 1e-9 >= self.settle_duration_s:
                    self.state = NavigationStatus.STATE_SUCCEEDED
                    self.message = 'waypoint reached and settled'
                    self.error_code = 0
                    self.cmd_pub.publish(Twist())
                    self.publish_status()
                    self.get_logger().info('waypoint %s succeeded', self.target.waypoint_id)
                    return
            else:
                if self.arrived_time and (now - self.arrived_time).nanoseconds * 1e-9 > self.settle_timeout_s:
                    self.stop_with_state(NavigationStatus.STATE_FAILED, 'settle timeout', 2004)
                    return
            self.message = 'settling'
            self.publish_status()
            return
        self.arrived_time = None

        scale = 1.0
        if self.slowdown_distance > 1e-6:
            scale = clamp(self.distance_error / self.slowdown_distance, 0.2, 1.0)
        desired = Twist()
        desired.linear.x = clamp(self.kp_xy * ex_body, -self.max_linear_speed, self.max_linear_speed) * scale
        desired.linear.y = clamp(self.kp_xy * ey_body, -self.max_linear_speed, self.max_linear_speed) * scale
        desired.angular.z = clamp(self.kp_yaw * self.yaw_error, -self.max_angular_speed, self.max_angular_speed)
        cmd = self.limit_accel(desired, dt)
        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)
        self.message = 'moving'
        self.publish_status()

    def publish_status(self) -> None:
        msg = NavigationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.backend = self.backend_name
        msg.state = self.state
        msg.waypoint_id = self.target.waypoint_id if self.target else ''
        msg.target_x_m = self.target.x if self.target else 0.0
        msg.target_y_m = self.target.y if self.target else 0.0
        msg.target_yaw_rad = self.target.yaw if self.target else 0.0
        msg.distance_error_m = float(self.distance_error)
        msg.yaw_error_rad = float(self.yaw_error)
        msg.error_code = int(self.error_code)
        msg.message = self.message
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PseudoNavBackend()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
