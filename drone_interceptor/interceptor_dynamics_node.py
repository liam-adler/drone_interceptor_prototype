#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from drone_interceptor.dynamics import PointMassState, SimplePointMassDynamics


class InterceptorDynamicsNode(Node):
    """
    Simulates the interceptor drone as a simple acceleration-limited point mass.

    Subscribes:
        /interceptor/cmd_vel          geometry_msgs/Vector3

    Publishes:
        /interceptor/state            nav_msgs/Odometry
        /interceptor/marker           visualization_msgs/Marker
        /interceptor/path_marker      visualization_msgs/Marker
    """

    def __init__(self) -> None:
        super().__init__("interceptor_dynamics_node")

        # Parameters
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("child_frame_id", "interceptor")
        self.declare_parameter("update_rate_hz", 50.0)

        self.declare_parameter("initial_x", -8.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_z", 2.0)

        self.declare_parameter("max_speed", 4.0)
        self.declare_parameter("max_accel", 3.0)
        self.declare_parameter("min_z", 0.3)
        self.declare_parameter("max_z", 20.0)

        self.declare_parameter("path_max_length", 1000)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)

        initial_x = float(self.get_parameter("initial_x").value)
        initial_y = float(self.get_parameter("initial_y").value)
        initial_z = float(self.get_parameter("initial_z").value)

        max_speed = float(self.get_parameter("max_speed").value)
        max_accel = float(self.get_parameter("max_accel").value)
        min_z = float(self.get_parameter("min_z").value)
        max_z = float(self.get_parameter("max_z").value)

        self.path_max_length = int(self.get_parameter("path_max_length").value)

        self.state = PointMassState(
            position=np.array([initial_x, initial_y, initial_z], dtype=float),
            velocity=np.zeros(3, dtype=float),
        )

        self.velocity_command = np.zeros(3, dtype=float)

        self.dynamics = SimplePointMassDynamics(
            max_speed=max_speed,
            max_accel=max_accel,
            min_z=min_z,
            max_z=max_z,
        )

        self.path_points: list[np.ndarray] = []

        # ROS interfaces
        self.cmd_sub = self.create_subscription(
            Vector3,
            "/interceptor/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        self.state_pub = self.create_publisher(Odometry, "/interceptor/state", 10)
        self.marker_pub = self.create_publisher(Marker, "/interceptor/marker", 10)
        self.path_marker_pub = self.create_publisher(
            Marker,
            "/interceptor/path_marker",
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time: Optional[rclpy.time.Time] = None

        timer_period = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Interceptor dynamics node started")

    def cmd_vel_callback(self, msg: Vector3) -> None:
        self.velocity_command = np.array([msg.x, msg.y, msg.z], dtype=float)

    def timer_callback(self) -> None:
        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            self.publish_all(now)
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        self.state = self.dynamics.step_velocity_command(
            state=self.state,
            velocity_command=self.velocity_command,
            dt=dt,
        )

        self.publish_all(now)

    def publish_all(self, now: rclpy.time.Time) -> None:
        yaw = self.compute_yaw_from_velocity(self.state.velocity)
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)

        self.publish_odometry(now, qx, qy, qz, qw)
        self.publish_tf(now, qx, qy, qz, qw)
        self.publish_marker(now, qx, qy, qz, qw)
        self.publish_path_marker(now)

    def publish_odometry(
        self,
        now: rclpy.time.Time,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> None:
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        msg.pose.pose.position.x = float(self.state.position[0])
        msg.pose.pose.position.y = float(self.state.position[1])
        msg.pose.pose.position.z = float(self.state.position[2])

        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = float(self.state.velocity[0])
        msg.twist.twist.linear.y = float(self.state.velocity[1])
        msg.twist.twist.linear.z = float(self.state.velocity[2])

        self.state_pub.publish(msg)

    def publish_tf(
        self,
        now: rclpy.time.Time,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.frame_id
        tf_msg.child_frame_id = self.child_frame_id

        tf_msg.transform.translation.x = float(self.state.position[0])
        tf_msg.transform.translation.y = float(self.state.position[1])
        tf_msg.transform.translation.z = float(self.state.position[2])

        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_marker(
        self,
        now: rclpy.time.Time,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> None:
        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = self.frame_id

        marker.ns = "interceptor"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(self.state.position[0])
        marker.pose.position.y = float(self.state.position[1])
        marker.pose.position.z = float(self.state.position[2])

        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        marker.scale.x = 0.45
        marker.scale.y = 0.45
        marker.scale.z = 0.45

        marker.color = ColorRGBA()
        marker.color.r = 0.1
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    def publish_path_marker(self, now: rclpy.time.Time) -> None:
        self.path_points.append(self.state.position.copy())

        if len(self.path_points) > self.path_max_length:
            self.path_points = self.path_points[-self.path_max_length :]

        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = self.frame_id

        marker.ns = "interceptor_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05

        marker.color = ColorRGBA()
        marker.color.r = 0.1
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 1.0

        for point in self.path_points:
            marker_point = Point()
            marker_point.x = float(point[0])
            marker_point.y = float(point[1])
            marker_point.z = float(point[2])
            marker.points.append(marker_point)

        self.path_marker_pub.publish(marker)

    @staticmethod
    def compute_yaw_from_velocity(velocity: np.ndarray) -> float:
        speed_xy = math.hypot(float(velocity[0]), float(velocity[1]))

        if speed_xy < 1e-6:
            return 0.0

        return math.atan2(float(velocity[1]), float(velocity[0]))

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
        half_yaw = 0.5 * yaw
        return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = InterceptorDynamicsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()