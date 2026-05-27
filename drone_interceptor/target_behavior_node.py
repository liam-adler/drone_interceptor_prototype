from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import Vector3
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32

HEADING_ZERO_TOLERANCE = 1e-6
DEFAULT_HEADING = np.array([1.0, 0.0, 0.0], dtype=float)


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
        direction = self.sample_random_heading()
        norm = float(np.linalg.norm(direction))
        if norm < HEADING_ZERO_TOLERANCE:
            return

        self.current_heading = direction / norm
        self.current_cruise_speed = self.sample_cruise_speed()
        self.publish_heading()

    def speed_timer_callback(self) -> None:
        speed_msg = Float32()
        speed_msg.data = self.compute_desired_speed()
        self.speed_publisher.publish(speed_msg)

    def publish_heading(self) -> None:
        self.heading_publisher.publish(self.to_vector3(self.current_heading))

    def compute_desired_speed(self) -> float:
        if (
            not self.enable_threat_response
            or self.target_position is None
            or self.interceptor_position is None
        ):
            return self.current_cruise_speed

        distance = float(
            np.linalg.norm(self.target_position - self.interceptor_position)
        )
        if distance >= self.threat_radius:
            return self.current_cruise_speed

        distance_ratio = max(distance, 0.0) / max(
            self.threat_radius,
            HEADING_ZERO_TOLERANCE,
        )
        urgency = 1.0 - distance_ratio
        escape_floor = self.max_speed * self.escape_speed_min_ratio
        boosted_speed = self.current_cruise_speed + urgency * (
            self.max_speed - self.current_cruise_speed
        )

        return float(np.clip(boosted_speed, escape_floor, self.max_speed))

    def sample_random_heading(self) -> np.ndarray:
        return np.array(
            [
                self.rng.uniform(-1.0, 1.0),
                self.rng.uniform(-1.0, 1.0),
                self.rng.uniform(-0.2, 0.2),
            ],
            dtype=float,
        )

    def sample_cruise_speed(self) -> float:
        return float(self.rng.uniform(self.min_speed, self.max_speed))

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
