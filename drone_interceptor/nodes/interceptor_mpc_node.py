#!/usr/bin/env python3

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from visualization_msgs.msg import Marker

from drone_interceptor.guidance.base import InterceptorControllerBase
from drone_interceptor.guidance.mpc_guidance import (
    AccelerationMPC,
    MpcConfig,
    MpcWeights,
)
from drone_interceptor.visualization.path_markers import build_line_strip_marker


class InterceptorMpcNode(InterceptorControllerBase):
    def __init__(self) -> None:
        Node.__init__(self, "interceptor_mpc_node")

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("target_state_topic", "/target/estimated_state")

        self.declare_parameter("capture_radius", 0.5)
        self.declare_parameter("max_speed", 4.0)
        self.declare_parameter("max_accel", 3.0)
        self.declare_parameter("min_z", 0.3)
        self.declare_parameter("max_z", 20.0)

        self.declare_parameter("horizon_steps", 24)
        self.declare_parameter("prediction_dt", 0.12)
        self.declare_parameter("prediction_dt_min", 0.06)
        self.declare_parameter("prediction_dt_max", 0.18)
        self.declare_parameter("prediction_dt_growth", 1.08)
        self.declare_parameter("solver_iterations", 48)
        self.declare_parameter("solver_step_size", 0.045)

        self.declare_parameter("weight_position", 8.0)
        self.declare_parameter("weight_relative_velocity", 0.35)
        self.declare_parameter("weight_non_closing_rate", 16.0)
        self.declare_parameter("weight_capture_set", 32.0)
        self.declare_parameter("weight_control", 0.008)
        self.declare_parameter("weight_control_delta", 0.02)
        self.declare_parameter("weight_terminal_position", 110.0)
        self.declare_parameter("weight_terminal_relative_velocity", 0.8)
        self.declare_parameter("weight_terminal_capture_set", 180.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.target_state_topic = str(self.get_parameter("target_state_topic").value)
        self.capture_radius = float(self.get_parameter("capture_radius").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_accel = float(self.get_parameter("max_accel").value)

        self.mpc = AccelerationMPC(
            config=MpcConfig(
                horizon_steps=int(self.get_parameter("horizon_steps").value),
                dt=float(self.get_parameter("prediction_dt").value),
                min_dt=float(self.get_parameter("prediction_dt_min").value),
                max_dt=float(self.get_parameter("prediction_dt_max").value),
                dt_growth=float(self.get_parameter("prediction_dt_growth").value),
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

        self.setup_controller_interfaces(
            target_state_topic=self.target_state_topic,
            interceptor_state_topic="/interceptor_mpc/state",
            intercept_marker_topic="/interceptor_mpc/intercept_marker",
            intercept_marker_namespace="interceptor_mpc_intercept_point",
            update_rate_hz=self.update_rate_hz,
            frame_id=self.frame_id,
            capture_radius=self.capture_radius,
        )

        self.cmd_pub = self.create_publisher(
            Vector3,
            "/interceptor_mpc/cmd_accel",
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

        self.get_logger().info("Interceptor MPC node started")

    def control_step(
        self,
        *,
        target_position: np.ndarray,
        target_velocity: np.ndarray,
        interceptor_position: np.ndarray,
        interceptor_velocity: np.ndarray,
    ) -> None:
        distance = float(np.linalg.norm(target_position - interceptor_position))

        if distance <= self.capture_radius:
            desired_velocity = self.limit_vector(target_velocity, self.max_speed)
            command_accel = self.limit_vector(
                (desired_velocity - interceptor_velocity) * self.update_rate_hz,
                self.max_accel,
            )
            self.publish_command(command_accel)
            self.publish_intercept_marker(
                position=target_position,
                captured=True,
                active_scale=0.3,
                captured_scale=0.75,
                active_color=(1.0, 0.9, 0.9, 1.0),
                captured_color=(0.1, 0.9, 0.9, 1.0),
            )
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
            active_scale=0.3,
            captured_scale=0.75,
            active_color=(1.0, 0.9, 0.9, 1.0),
            captured_color=(0.1, 0.9, 0.9, 1.0),
        )
        self.publish_prediction_markers(
            interceptor_positions=solution.predicted_positions,
            target_positions=solution.predicted_target_positions,
        )

    def publish_command(self, acceleration: np.ndarray) -> None:
        self.cmd_pub.publish(self.to_vector3(acceleration))

    def publish_prediction_markers(
        self,
        interceptor_positions: list[np.ndarray],
        target_positions: list[np.ndarray],
    ) -> None:
        now = self.get_clock().now().to_msg()

        interceptor_marker = build_line_strip_marker(
            stamp=now,
            frame_id=self.frame_id,
            namespace="interceptor_mpc_prediction",
            marker_id=0,
            points=interceptor_positions,
            line_width=0.035,
            color=(0.0, 0.9, 0.6, 0.9),
        )
        target_marker = build_line_strip_marker(
            stamp=now,
            frame_id=self.frame_id,
            namespace="target_prediction",
            marker_id=0,
            points=target_positions,
            line_width=0.025,
            color=(1.0, 0.6, 0.1, 0.8),
        )

        self.predicted_path_pub.publish(interceptor_marker)
        self.target_prediction_pub.publish(target_marker)


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
