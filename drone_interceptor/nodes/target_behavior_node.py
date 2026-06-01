from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import Vector3
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32

from drone_interceptor.target_behavior.random_maneuver import (
    DEFAULT_HEADING,
    compute_desired_speed,
    normalized_heading,
    sample_cruise_speed,
    sample_random_heading,
)


class TargetBehaviorNode(Node):
    def __init__(self) -> None:
        super().__init__("target_behavior_node")

        self.declare_parameter("min_speed", 1.0)
        self.declare_parameter("max_speed", 8.0)
        self.declare_parameter("enable_threat_response", True)
        self.declare_parameter("threat_radius", 12.0)
        self.declare_parameter("escape_speed_min", 0.9)
        self.declare_parameter("heading_update_period", 4.0)
        self.declare_parameter("speed_update_period", 0.2)
        self.declare_parameter("interceptor_state_topic", "/interceptor/state")
        self.declare_parameter("random_seed", 0)

        self.min_speed = self.get_float_parameter("min_speed")
        self.max_speed = self.get_float_parameter("max_speed")
        self.enable_threat_response = self.get_bool_parameter(
            "enable_threat_response"
        )
        self.threat_radius = self.get_float_parameter("threat_radius")
        self.escape_speed_min_ratio = self.get_float_parameter("escape_speed_min")
        heading_update_period = self.get_float_parameter("heading_update_period")
        speed_update_period = self.get_float_parameter("speed_update_period")
        interceptor_state_topic = self.get_string_parameter("interceptor_state_topic")
        random_seed = self.get_int_parameter("random_seed")

        self.current_heading = DEFAULT_HEADING.copy()
        self.current_cruise_speed = self.min_speed
        self.target_position: Optional[np.ndarray] = None
        self.interceptor_position: Optional[np.ndarray] = None
        self.rng = np.random.default_rng(random_seed)

        self.heading_publisher = self.create_publisher(
            Vector3,
            "/target/desired_heading",
            10,
        )

        self.speed_publisher = self.create_publisher(
            Float32,
            "/target/desired_speed",
            10,
        )

        self.create_subscription(
            Odometry,
            "/target/state",
            self.target_state_callback,
            10,
        )

        self.create_subscription(
            Odometry,
            interceptor_state_topic,
            self.interceptor_state_callback,
            10,
        )

        self.create_timer(
            heading_update_period,
            self.heading_timer_callback,
        )
        self.create_timer(
            speed_update_period,
            self.speed_timer_callback,
        )

    def target_state_callback(self, msg: Odometry) -> None:
        self.target_position = self.extract_position(msg)

    def interceptor_state_callback(self, msg: Odometry) -> None:
        self.interceptor_position = self.extract_position(msg)

    def heading_timer_callback(self) -> None:
        direction = sample_random_heading(self.rng)
        heading = normalized_heading(direction)
        if heading is None:
            return

        self.current_heading = heading
        self.current_cruise_speed = sample_cruise_speed(
            self.rng,
            min_speed=self.min_speed,
            max_speed=self.max_speed,
        )
        self.publish_heading()

    def speed_timer_callback(self) -> None:
        speed_msg = Float32()
        speed_msg.data = self.compute_desired_speed()
        self.speed_publisher.publish(speed_msg)

    def publish_heading(self) -> None:
        self.heading_publisher.publish(self.to_vector3(self.current_heading))

    def compute_desired_speed(self) -> float:
        return compute_desired_speed(
            current_cruise_speed=self.current_cruise_speed,
            max_speed=self.max_speed,
            enable_threat_response=self.enable_threat_response,
            threat_radius=self.threat_radius,
            escape_speed_min_ratio=self.escape_speed_min_ratio,
            target_position=self.target_position,
            interceptor_position=self.interceptor_position,
        )

    def get_bool_parameter(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def get_float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def get_int_parameter(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def get_string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

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
    def to_vector3(vector: np.ndarray) -> Vector3:
        msg = Vector3()
        msg.x = float(vector[0])
        msg.y = float(vector[1])
        msg.z = float(vector[2])
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)

    node = TargetBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
