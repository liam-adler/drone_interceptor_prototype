#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from drone_interceptor.dynamics import PointMassState, SimplePointMassDynamics
from drone_interceptor.visualization.path_markers import build_line_strip_marker
from drone_interceptor.visualization.rviz_markers import build_sphere_marker

Array3 = np.ndarray
CommandMode = Literal["velocity", "acceleration"]
ZERO_TOLERANCE = 1e-9


@dataclass(frozen=True)
class PointMassNodeConfig:
    node_name: str
    log_label: str
    child_frame_id: str
    command_mode: CommandMode
    state_topic: str
    marker_topic: str
    path_marker_topic: str
    marker_namespace: str
    path_marker_namespace: str
    marker_scale: float
    marker_color: tuple[float, float, float, float]
    command_topic: str
    command_topic_parameter: str
    command_parameter_default: str
    enable_heading_speed_input: bool = False
    desired_heading_topic: str = ""
    desired_speed_topic: str = ""


class PointMassDynamicsNode(Node):
    def __init__(self, config: PointMassNodeConfig) -> None:
        super().__init__(config.node_name)
        self.config = config

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("child_frame_id", config.child_frame_id)
        self.declare_parameter("update_rate_hz", 50.0)

        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_z", 2.0)

        self.declare_parameter("max_speed", 3.0)
        self.declare_parameter("max_accel", 2.0)
        self.declare_parameter("min_z", 0.3)
        self.declare_parameter("max_z", 20.0)
        self.declare_parameter("path_max_length", 1000)

        self.declare_parameter(
            config.command_topic_parameter,
            config.command_parameter_default,
        )

        if config.enable_heading_speed_input:
            self.declare_parameter("desired_heading_topic", config.desired_heading_topic)
            self.declare_parameter("desired_speed_topic", config.desired_speed_topic)

        self.frame_id = self.get_string_parameter("frame_id")
        self.child_frame_id = self.get_string_parameter("child_frame_id")
        self.update_rate_hz = self.get_float_parameter("update_rate_hz")

        initial_position = np.array(
            [
                self.get_float_parameter("initial_x"),
                self.get_float_parameter("initial_y"),
                self.get_float_parameter("initial_z"),
            ],
            dtype=float,
        )
        max_speed = self.get_float_parameter("max_speed")
        max_accel = self.get_float_parameter("max_accel")
        min_z = self.get_float_parameter("min_z")
        max_z = self.get_float_parameter("max_z")
        self.path_max_length = self.get_int_parameter("path_max_length")

        self.command_topic = self.get_string_parameter(config.command_topic_parameter)
        self.desired_heading = np.array([1.0, 0.0, 0.0], dtype=float)
        self.desired_speed = 0.0
        self.command_vector = np.zeros(3, dtype=float)

        self.state = PointMassState(
            position=initial_position,
            velocity=np.zeros(3, dtype=float),
        )
        self.dynamics = SimplePointMassDynamics(
            max_speed=max_speed,
            max_accel=max_accel,
            min_z=min_z,
            max_z=max_z,
        )

        self.path_points: list[Array3] = []
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_time: Optional[rclpy.time.Time] = None

        self.create_subscription(
            Vector3,
            self.command_topic,
            self.command_callback,
            10,
        )

        if config.enable_heading_speed_input:
            self.create_subscription(
                Vector3,
                self.get_string_parameter("desired_heading_topic"),
                self.desired_heading_callback,
                10,
            )
            self.create_subscription(
                Float32,
                self.get_string_parameter("desired_speed_topic"),
                self.desired_speed_callback,
                10,
            )

        self.state_pub = self.create_publisher(Odometry, config.state_topic, 10)
        self.marker_pub = self.create_publisher(Marker, config.marker_topic, 10)
        self.path_marker_pub = self.create_publisher(
            Marker,
            config.path_marker_topic,
            10,
        )

        self.create_timer(1.0 / self.update_rate_hz, self.timer_callback)

        self.get_logger().info(f"{config.log_label} started")

    def command_callback(self, msg: Vector3) -> None:
        self.command_vector = self.vector_from_xyz(msg.x, msg.y, msg.z)

    def desired_heading_callback(self, msg: Vector3) -> None:
        heading = self.vector_from_xyz(msg.x, msg.y, msg.z)
        norm = float(np.linalg.norm(heading))
        if norm < ZERO_TOLERANCE:
            return

        self.desired_heading = heading / norm
        self.update_velocity_command_from_behavior()

    def desired_speed_callback(self, msg: Float32) -> None:
        self.desired_speed = max(0.0, float(msg.data))
        self.update_velocity_command_from_behavior()

    def update_velocity_command_from_behavior(self) -> None:
        if self.config.command_mode != "velocity":
            return
        self.command_vector = self.desired_heading * self.desired_speed

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

        if self.config.command_mode == "velocity":
            self.state = self.dynamics.step_velocity_command(
                state=self.state,
                velocity_command=self.command_vector,
                dt=dt,
            )
        else:
            self.state = self.dynamics.step_acceleration_command(
                state=self.state,
                acceleration_command=self.command_vector,
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
        position = self.state.position
        velocity = self.state.velocity

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = float(position[2])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = float(velocity[0])
        msg.twist.twist.linear.y = float(velocity[1])
        msg.twist.twist.linear.z = float(velocity[2])
        self.state_pub.publish(msg)

    def publish_tf(
        self,
        now: rclpy.time.Time,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> None:
        position = self.state.position

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.frame_id
        tf_msg.child_frame_id = self.child_frame_id
        tf_msg.transform.translation.x = float(position[0])
        tf_msg.transform.translation.y = float(position[1])
        tf_msg.transform.translation.z = float(position[2])
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
        position = self.state.position
        marker = build_sphere_marker(
            stamp=now.to_msg(),
            frame_id=self.frame_id,
            namespace=self.config.marker_namespace,
            marker_id=0,
            position=position,
            orientation=(qx, qy, qz, qw),
            scale=self.config.marker_scale,
            color=self.config.marker_color,
        )
        self.marker_pub.publish(marker)

    def publish_path_marker(self, now: rclpy.time.Time) -> None:
        self.path_points.append(self.state.position.copy())
        if len(self.path_points) > self.path_max_length:
            self.path_points = self.path_points[-self.path_max_length:]

        marker = build_line_strip_marker(
            stamp=now.to_msg(),
            frame_id=self.frame_id,
            namespace=self.config.path_marker_namespace,
            marker_id=0,
            points=self.path_points,
            line_width=0.05,
            color=self.config.marker_color,
        )
        self.path_marker_pub.publish(marker)

    def get_float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def get_int_parameter(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def get_string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    @staticmethod
    def vector_from_xyz(x: float, y: float, z: float) -> Array3:
        return np.array([x, y, z], dtype=float)

    @staticmethod
    def compute_yaw_from_velocity(velocity: Array3) -> float:
        speed_xy = math.hypot(float(velocity[0]), float(velocity[1]))
        if speed_xy < ZERO_TOLERANCE:
            return 0.0
        return math.atan2(float(velocity[1]), float(velocity[0]))

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
        half_yaw = 0.5 * yaw
        return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)
