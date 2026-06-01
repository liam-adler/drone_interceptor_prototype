#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import numpy as np
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

Array3 = np.ndarray
NORM_TOLERANCE = 1e-9


class InterceptorControllerBase(Node):
    def setup_controller_interfaces(
        self,
        *,
        interceptor_state_topic: str,
        intercept_marker_topic: str,
        intercept_marker_namespace: str,
        update_rate_hz: float,
        frame_id: str,
        capture_radius: float,
    ) -> None:
        self.frame_id = frame_id
        self.capture_radius = capture_radius
        self.intercept_marker_namespace = intercept_marker_namespace

        self.target_state: Optional[Odometry] = None
        self.interceptor_state: Optional[Odometry] = None

        self.create_subscription(
            Odometry,
            "/target/state",
            self.target_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            interceptor_state_topic,
            self.interceptor_callback,
            10,
        )

        self.marker_pub = self.create_publisher(
            Marker,
            intercept_marker_topic,
            10,
        )
        self.create_timer(1.0 / update_rate_hz, self.timer_callback)

    def target_callback(self, msg: Odometry) -> None:
        self.target_state = msg

    def interceptor_callback(self, msg: Odometry) -> None:
        self.interceptor_state = msg

    def timer_callback(self) -> None:
        if self.target_state is None or self.interceptor_state is None:
            self.publish_zero_command()
            return

        target_position = self.extract_position(self.target_state)
        target_velocity = self.extract_velocity(self.target_state)
        interceptor_position = self.extract_position(self.interceptor_state)
        interceptor_velocity = self.extract_velocity(self.interceptor_state)

        self.control_step(
            target_position=target_position,
            target_velocity=target_velocity,
            interceptor_position=interceptor_position,
            interceptor_velocity=interceptor_velocity,
        )

    def control_step(
        self,
        *,
        target_position: Array3,
        target_velocity: Array3,
        interceptor_position: Array3,
        interceptor_velocity: Array3,
    ) -> None:
        raise NotImplementedError

    def publish_zero_command(self) -> None:
        self.publish_command(np.zeros(3, dtype=float))

    def publish_command(self, vector: Array3) -> None:
        raise NotImplementedError

    def publish_intercept_marker(
        self,
        *,
        position: Array3,
        captured: bool,
        active_scale: float,
        captured_scale: float,
        active_color: tuple[float, float, float, float],
        captured_color: tuple[float, float, float, float],
    ) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = self.intercept_marker_namespace
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0

        marker_scale = captured_scale if captured else active_scale
        marker.scale.x = marker_scale
        marker.scale.y = marker_scale
        marker.scale.z = marker_scale
        marker.color = self.make_color(*(captured_color if captured else active_color))
        self.marker_pub.publish(marker)

    @staticmethod
    def extract_position(msg: Odometry) -> Array3:
        return np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=float,
        )

    @staticmethod
    def extract_velocity(msg: Odometry) -> Array3:
        return np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ],
            dtype=float,
        )

    @staticmethod
    def limit_vector(vector: Array3, max_norm: float) -> Array3:
        norm = float(np.linalg.norm(vector))
        if norm < NORM_TOLERANCE:
            return np.zeros(3, dtype=float)
        if norm <= max_norm:
            return vector.copy()
        return vector / norm * max_norm

    @staticmethod
    def to_vector3(vector: Array3) -> Vector3:
        msg = Vector3()
        msg.x = float(vector[0])
        msg.y = float(vector[1])
        msg.z = float(vector[2])
        return msg

    @staticmethod
    def make_color(r: float, g: float, b: float, a: float) -> ColorRGBA:
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a
        return color

    @staticmethod
    def make_point(coords: Array3) -> Point:
        point = Point()
        point.x = float(coords[0])
        point.y = float(coords[1])
        point.z = float(coords[2])
        return point
