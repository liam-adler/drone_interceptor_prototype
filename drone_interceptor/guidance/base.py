#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import numpy as np
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from drone_interceptor.core.math_utils import NORM_TOLERANCE, clamp_norm
from drone_interceptor.visualization.rviz_markers import (
    build_sphere_marker,
    make_color,
    make_point,
)

Array3 = np.ndarray


class InterceptorControllerBase(Node):
    def setup_controller_interfaces(
        self,
        *,
        target_state_topic: str,
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
            target_state_topic,
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
        marker_scale = captured_scale if captured else active_scale
        marker = build_sphere_marker(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self.frame_id,
            namespace=self.intercept_marker_namespace,
            marker_id=0,
            position=position,
            scale=marker_scale,
            color=captured_color if captured else active_color,
        )
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
        return clamp_norm(vector, max_norm)

    @staticmethod
    def to_vector3(vector: Array3) -> Vector3:
        msg = Vector3()
        msg.x = float(vector[0])
        msg.y = float(vector[1])
        msg.z = float(vector[2])
        return msg

    @staticmethod
    def make_color(r: float, g: float, b: float, a: float) -> ColorRGBA:
        return make_color(r, g, b, a)

    @staticmethod
    def make_point(coords: Array3) -> Point:
        return make_point(coords)
