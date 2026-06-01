#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from drone_interceptor.mpc import AccelerationMPC, MpcConfig, MpcWeights


class InterceptorMpcNode(Node):
    def __init__(self) -> None:
        super().__init__("interceptor_mpc_node")

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("update_rate_hz", 20.0)

        self.declare_parameter("capture_radius", 0.5)
        self.declare_parameter("max_speed", 4.0)
        self.declare_parameter("max_accel", 3.0)
        self.declare_parameter("min_z", 0.3)
        self.declare_parameter("max_z", 20.0)

        self.declare_parameter("horizon_steps", 24)
        self.declare_parameter("prediction_dt", 0.12)
        self.declare_parameter("solver_iterations", 48)
        self.declare_parameter("solver_step_size", 0.045)

        self.declare_parameter("weight_position", 7.0)
        self.declare_parameter("weight_relative_velocity", 0.25)
        self.declare_parameter("weight_non_closing_rate", 10.0)
        self.declare_parameter("weight_capture_set", 22.0)
        self.declare_parameter("weight_control", 0.008)
        self.declare_parameter("weight_control_delta", 0.025)
        self.declare_parameter("weight_terminal_position", 70.0)
        self.declare_parameter("weight_terminal_relative_velocity", 0.6)
        self.declare_parameter("weight_terminal_capture_set", 80.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.capture_radius = float(self.get_parameter("capture_radius").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_accel = float(self.get_parameter("max_accel").value)

        self.mpc = AccelerationMPC(
            config=MpcConfig(
                horizon_steps=int(self.get_parameter("horizon_steps").value),
                dt=float(self.get_parameter("prediction_dt").value),
                iterations=int(self.get_parameter("solver_iterations").value),
                step_size=float(self.get_parameter("solver_step_size").value),
                max_speed=self.max_speed,
                max_accel=self.max_accel,
                min_z=float(self.get_parameter("min_z").value),
                max_z=float(self.get_parameter("max_z").value),
                capture_radius=self.capture_radius,
            ),
            weights=MpcWeights(
                position=float(self.get_parameter("weight_position").value),
                relative_velocity=float(
                    self.get_parameter("weight_relative_velocity").value
                ),
                non_closing_rate=float(
                    self.get_parameter("weight_non_closing_rate").value
                ),
                capture_set=float(self.get_parameter("weight_capture_set").value),
                control=float(self.get_parameter("weight_control").value),
                control_delta=float(self.get_parameter("weight_control_delta").value),
                terminal_position=float(
                    self.get_parameter("weight_terminal_position").value
                ),
                terminal_relative_velocity=float(
                    self.get_parameter("weight_terminal_relative_velocity").value
                ),
                terminal_capture_set=float(
                    self.get_parameter("weight_terminal_capture_set").value
                ),
            ),
        )

        self.target_state: Optional[Odometry] = None
        self.interceptor_state: Optional[Odometry] = None

        self.target_sub = self.create_subscription(
            Odometry,
            "/target/state",
            self.target_callback,
            10,
        )
        self.interceptor_sub = self.create_subscription(
            Odometry,
            "/interceptor_mpc/state",
            self.interceptor_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Vector3,
            "/interceptor_mpc/cmd_accel",
            10,
        )
        self.marker_pub = self.create_publisher(
            Marker,
            "/interceptor_mpc/intercept_marker",
            10,
        )
        self.predicted_path_pub = self.create_publisher(
            Marker,
            "/interceptor_mpc/predicted_path_marker",
            10,
        )
        self.target_prediction_pub = self.create_publisher(
            Marker,
            "/interceptor_mpc/target_prediction_marker",
            10,
        )

        timer_period = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Interceptor MPC node started")

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

        distance = float(np.linalg.norm(target_position - interceptor_position))

        if distance <= self.capture_radius:
            desired_velocity = self.limit_vector(target_velocity, self.max_speed)
            command_accel = self.limit_vector(
                (desired_velocity - interceptor_velocity) * self.update_rate_hz,
                self.max_accel,
            )
            self.publish_command(command_accel)
            self.publish_intercept_marker(position=target_position, captured=True)
            return

        solution = self.mpc.solve(
            interceptor_position=interceptor_position,
            interceptor_velocity=interceptor_velocity,
            target_position=target_position,
            target_velocity=target_velocity,
        )

        self.publish_command(solution.first_acceleration)
        self.publish_intercept_marker(
            position=solution.predicted_target_positions[0],
            captured=False,
        )
        self.publish_prediction_markers(
            interceptor_positions=solution.predicted_positions,
            target_positions=solution.predicted_target_positions,
        )

    def publish_command(self, acceleration: np.ndarray) -> None:
        msg = Vector3()
        msg.x = float(acceleration[0])
        msg.y = float(acceleration[1])
        msg.z = float(acceleration[2])
        self.cmd_pub.publish(msg)

    def publish_zero_command(self) -> None:
        self.publish_command(np.zeros(3, dtype=float))

    def publish_intercept_marker(self, position: np.ndarray, captured: bool) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "interceptor_mpc_intercept_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3 if not captured else 0.75
        marker.scale.y = 0.3 if not captured else 0.75
        marker.scale.z = 0.3 if not captured else 0.75
        marker.color = ColorRGBA()
        marker.color.r = 0.1 if captured else 1.0
        marker.color.g = 0.9
        marker.color.b = 0.9
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def publish_prediction_markers(
        self,
        interceptor_positions: list[np.ndarray],
        target_positions: list[np.ndarray],
    ) -> None:
        now = self.get_clock().now().to_msg()

        interceptor_marker = Marker()
        interceptor_marker.header.stamp = now
        interceptor_marker.header.frame_id = self.frame_id
        interceptor_marker.ns = "interceptor_mpc_prediction"
        interceptor_marker.id = 0
        interceptor_marker.type = Marker.LINE_STRIP
        interceptor_marker.action = Marker.ADD
        interceptor_marker.scale.x = 0.035
        interceptor_marker.color = ColorRGBA()
        interceptor_marker.color.r = 0.0
        interceptor_marker.color.g = 0.9
        interceptor_marker.color.b = 0.6
        interceptor_marker.color.a = 0.9

        target_marker = Marker()
        target_marker.header.stamp = now
        target_marker.header.frame_id = self.frame_id
        target_marker.ns = "target_prediction"
        target_marker.id = 0
        target_marker.type = Marker.LINE_STRIP
        target_marker.action = Marker.ADD
        target_marker.scale.x = 0.025
        target_marker.color = ColorRGBA()
        target_marker.color.r = 1.0
        target_marker.color.g = 0.6
        target_marker.color.b = 0.1
        target_marker.color.a = 0.8

        for position in interceptor_positions:
            point = Point()
            point.x = float(position[0])
            point.y = float(position[1])
            point.z = float(position[2])
            interceptor_marker.points.append(point)

        for position in target_positions:
            point = Point()
            point.x = float(position[0])
            point.y = float(position[1])
            point.z = float(position[2])
            target_marker.points.append(point)

        self.predicted_path_pub.publish(interceptor_marker)
        self.target_prediction_pub.publish(target_marker)

    @staticmethod
    def extract_position(msg: Odometry) -> np.ndarray:
        return np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=float,
        )

    @staticmethod
    def extract_velocity(msg: Odometry) -> np.ndarray:
        return np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ],
            dtype=float,
        )

    @staticmethod
    def limit_vector(vector: np.ndarray, max_norm: float) -> np.ndarray:
        norm = float(np.linalg.norm(vector))

        if norm < 1e-9:
            return np.zeros(3, dtype=float)

        if norm <= max_norm:
            return vector.copy()

        return vector / norm * max_norm


def main(args=None) -> None:
    rclpy.init(args=args)

    node = InterceptorMpcNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
