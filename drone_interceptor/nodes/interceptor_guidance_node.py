#!/usr/bin/env python3

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node

from drone_interceptor.guidance.base import (
    InterceptorControllerBase,
    NORM_TOLERANCE,
)


class InterceptorGuidanceNode(InterceptorControllerBase):
    """
    Guidance controller for the interceptor drone.

    Subscribes:
        /target/state         nav_msgs/Odometry
        /interceptor/state    nav_msgs/Odometry

    Publishes:
        /interceptor/cmd_vel  geometry_msgs/Vector3
        /intercept/marker     visualization_msgs/Marker

    Modes:
        pure_pursuit:
            Aim directly at current target position.

        lead_pursuit:
            Predict where the target will be and aim there.
    """

    def __init__(self) -> None:
        Node.__init__(self, "interceptor_guidance_node")

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("update_rate_hz", 30.0)

        self.declare_parameter("mode", "lead_pursuit")
        self.declare_parameter("interceptor_speed", 4.0)
        self.declare_parameter("capture_radius", 0.5)

        self.declare_parameter("max_prediction_time", 8.0)
        self.declare_parameter("min_prediction_time", 0.1)

        self.frame_id = self.get_string_parameter("frame_id")
        self.update_rate_hz = self.get_float_parameter("update_rate_hz")

        self.mode = self.get_string_parameter("mode")
        self.interceptor_speed = self.get_float_parameter("interceptor_speed")
        self.capture_radius = self.get_float_parameter("capture_radius")

        self.max_prediction_time = self.get_float_parameter("max_prediction_time")
        self.min_prediction_time = self.get_float_parameter("min_prediction_time")

        self.setup_controller_interfaces(
            interceptor_state_topic="/interceptor/state",
            intercept_marker_topic="/intercept/marker",
            intercept_marker_namespace="intercept_point",
            update_rate_hz=self.update_rate_hz,
            frame_id=self.frame_id,
            capture_radius=self.capture_radius,
        )

        self.cmd_pub = self.create_publisher(
            Vector3,
            "/interceptor/cmd_vel",
            10,
        )

        self.get_logger().info(
            f"Interceptor guidance node started in mode='{self.mode}'"
        )

    def control_step(
        self,
        *,
        target_position: np.ndarray,
        target_velocity: np.ndarray,
        interceptor_position: np.ndarray,
        interceptor_velocity: np.ndarray,
    ) -> None:
        del interceptor_velocity
        relative_position = target_position - interceptor_position
        distance = float(np.linalg.norm(relative_position))

        if distance <= self.capture_radius:
            follow_velocity = self.limit_vector(
                target_velocity,
                self.interceptor_speed,
            )
            self.cmd_pub.publish(self.to_vector3(follow_velocity))

            self.publish_intercept_marker(
                position=target_position,
                captured=True,
                active_scale=0.35,
                captured_scale=0.8,
                active_color=(1.0, 1.0, 0.1, 1.0),
                captured_color=(0.1, 1.0, 0.1, 1.0),
            )
            return

        if self.mode == "pure_pursuit":
            aim_point = target_position

        elif self.mode == "lead_pursuit":
            aim_point = self.compute_lead_pursuit_aim_point(
                target_position=target_position,
                target_velocity=target_velocity,
                interceptor_position=interceptor_position,
                interceptor_speed=self.interceptor_speed,
            )

        else:
            self.get_logger().warn(
                f"Unknown guidance mode '{self.mode}', falling back to pure_pursuit"
            )
            aim_point = target_position

        command_direction = aim_point - interceptor_position
        command_norm = float(np.linalg.norm(command_direction))

        if command_norm < NORM_TOLERANCE:
            self.publish_zero_command()
            return

        command_velocity = (
            command_direction / command_norm * self.interceptor_speed
        )

        self.cmd_pub.publish(self.to_vector3(command_velocity))
        self.publish_intercept_marker(
            position=aim_point,
            captured=False,
            active_scale=0.35,
            captured_scale=0.8,
            active_color=(1.0, 1.0, 0.1, 1.0),
            captured_color=(0.1, 1.0, 0.1, 1.0),
        )

    def compute_lead_pursuit_aim_point(
        self,
        target_position: np.ndarray,
        target_velocity: np.ndarray,
        interceptor_position: np.ndarray,
        interceptor_speed: float,
    ) -> np.ndarray:
        """
        Constant-velocity lead pursuit.

        Solves approximately:

            ||target_position + target_velocity * t - interceptor_position||
            =
            interceptor_speed * t

        If no good solution exists, falls back to a bounded prediction time.
        """

        relative_position = target_position - interceptor_position

        a = float(np.dot(target_velocity, target_velocity) - interceptor_speed**2)
        b = float(2.0 * np.dot(relative_position, target_velocity))
        c = float(np.dot(relative_position, relative_position))

        candidate_times: list[float] = []

        if abs(a) < NORM_TOLERANCE:
            if abs(b) > NORM_TOLERANCE:
                t = -c / b
                if t > 0.0:
                    candidate_times.append(t)
        else:
            discriminant = b**2 - 4.0 * a * c

            if discriminant >= 0.0:
                sqrt_discriminant = float(np.sqrt(discriminant))

                t1 = (-b - sqrt_discriminant) / (2.0 * a)
                t2 = (-b + sqrt_discriminant) / (2.0 * a)

                if t1 > 0.0:
                    candidate_times.append(t1)

                if t2 > 0.0:
                    candidate_times.append(t2)

        if candidate_times:
            t_go = min(candidate_times)
        else:
            distance = float(np.linalg.norm(relative_position))
            t_go = distance / max(interceptor_speed, 1e-6)

        t_go = float(
            np.clip(
                t_go,
                self.min_prediction_time,
                self.max_prediction_time,
            )
        )

        return target_position + target_velocity * t_go

    def publish_command(self, vector: np.ndarray) -> None:
        self.cmd_pub.publish(self.to_vector3(vector))

    def get_float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def get_string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = InterceptorGuidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
