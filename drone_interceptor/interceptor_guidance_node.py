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


class InterceptorGuidanceNode(Node):
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
        super().__init__("interceptor_guidance_node")

        # Parameters
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("update_rate_hz", 30.0)

        self.declare_parameter("mode", "lead_pursuit")
        self.declare_parameter("interceptor_speed", 4.0)
        self.declare_parameter("capture_radius", 0.5)

        self.declare_parameter("max_prediction_time", 8.0)
        self.declare_parameter("min_prediction_time", 0.1)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)

        self.mode = str(self.get_parameter("mode").value)
        self.interceptor_speed = float(self.get_parameter("interceptor_speed").value)
        self.capture_radius = float(self.get_parameter("capture_radius").value)

        self.max_prediction_time = float(self.get_parameter("max_prediction_time").value)
        self.min_prediction_time = float(self.get_parameter("min_prediction_time").value)

        self.target_state: Optional[Odometry] = None
        self.interceptor_state: Optional[Odometry] = None

        # ROS interfaces
        self.target_sub = self.create_subscription(
            Odometry,
            "/target/state",
            self.target_callback,
            10,
        )

        self.interceptor_sub = self.create_subscription(
            Odometry,
            "/interceptor/state",
            self.interceptor_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Vector3,
            "/interceptor/cmd_vel",
            10,
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/intercept/marker",
            10,
        )

        timer_period = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"Interceptor guidance node started in mode='{self.mode}'"
        )

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

        relative_position = target_position - interceptor_position
        distance = float(np.linalg.norm(relative_position))

        if distance <= self.capture_radius:
            follow_velocity = self.limit_vector(
                target_velocity,
                self.interceptor_speed,
            )

            cmd_msg = Vector3()
            cmd_msg.x = float(follow_velocity[0])
            cmd_msg.y = float(follow_velocity[1])
            cmd_msg.z = float(follow_velocity[2])

            self.cmd_pub.publish(cmd_msg)

            self.publish_intercept_marker(
                position=target_position,
                captured=True,
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

        if command_norm < 1e-9:
            self.publish_zero_command()
            return

        command_velocity = (
            command_direction / command_norm * self.interceptor_speed
        )

        cmd_msg = Vector3()
        cmd_msg.x = float(command_velocity[0])
        cmd_msg.y = float(command_velocity[1])
        cmd_msg.z = float(command_velocity[2])

        self.cmd_pub.publish(cmd_msg)
        self.publish_intercept_marker(position=aim_point, captured=False)

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

        if abs(a) < 1e-9:
            if abs(b) > 1e-9:
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

    def publish_zero_command(self) -> None:
        msg = Vector3()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        self.cmd_pub.publish(msg)

    def publish_intercept_marker(
        self,
        position: np.ndarray,
        captured: bool,
    ) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id

        marker.ns = "intercept_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.35 if not captured else 0.8
        marker.scale.y = 0.35 if not captured else 0.8
        marker.scale.z = 0.35 if not captured else 0.8

        marker.color = ColorRGBA()

        if captured:
            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.1
            marker.color.a = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.1
            marker.color.a = 1.0

        self.marker_pub.publish(marker)

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