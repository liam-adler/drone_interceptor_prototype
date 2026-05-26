from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class TargetBehaviorNode(Node):
    def __init__(self):
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

        self.min_speed = float(self.get_parameter("min_speed").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.enable_threat_response = bool(
            self.get_parameter("enable_threat_response").value
        )
        self.threat_radius = float(self.get_parameter("threat_radius").value)
        self.escape_speed_min_ratio = float(
            self.get_parameter("escape_speed_min").value
        )
        heading_update_period = float(
            self.get_parameter("heading_update_period").value
        )
        speed_update_period = float(
            self.get_parameter("speed_update_period").value
        )
        interceptor_state_topic = str(
            self.get_parameter("interceptor_state_topic").value
        )
        random_seed = int(self.get_parameter("random_seed").value)

        self.current_heading = np.array([1.0, 0.0, 0.0], dtype=float)
        self.current_cruise_speed = self.min_speed
        self.target_position: Optional[np.ndarray] = None
        self.interceptor_position: Optional[np.ndarray] = None
        self.rng = np.random.default_rng(random_seed)

        self.publisher_ = self.create_publisher(
            Vector3,
            "/target/desired_heading",
            10
        )

        self.speed_publisher_ = self.create_publisher(
            Float32,
            "/target/desired_speed",
            10
        )

        self.target_state_sub = self.create_subscription(
            Odometry,
            "/target/state",
            self.target_state_callback,
            10,
        )

        self.interceptor_state_sub = self.create_subscription(
            Odometry,
            interceptor_state_topic,
            self.interceptor_state_callback,
            10,
        )

        self.heading_timer = self.create_timer(
            heading_update_period,
            self.heading_timer_callback
        )
        self.speed_timer = self.create_timer(
            speed_update_period,
            self.speed_timer_callback
        )

    def target_state_callback(self, msg: Odometry):
        self.target_position = self.extract_position(msg)

    def interceptor_state_callback(self, msg: Odometry):
        self.interceptor_position = self.extract_position(msg)

    def heading_timer_callback(self):
        direction = np.array([
            self.rng.uniform(-1.0, 1.0),
            self.rng.uniform(-1.0, 1.0),
            self.rng.uniform(-0.2, 0.2),
        ])

        norm = float(np.linalg.norm(direction))

        if norm < 1e-6:
            return

        self.current_heading = direction / norm
        self.current_cruise_speed = float(
            self.rng.uniform(self.min_speed, self.max_speed)
        )
        self.publish_heading()

    def speed_timer_callback(self):
        speed_msg = Float32()
        speed_msg.data = self.compute_desired_speed()
        self.speed_publisher_.publish(speed_msg)

    def publish_heading(self):
        heading_msg = Vector3()
        heading_msg.x = float(self.current_heading[0])
        heading_msg.y = float(self.current_heading[1])
        heading_msg.z = float(self.current_heading[2])

        self.publisher_.publish(heading_msg)

    def compute_desired_speed(self) -> float:
        if not self.enable_threat_response:
            return self.current_cruise_speed

        if self.target_position is None or self.interceptor_position is None:
            return self.current_cruise_speed

        distance = float(
            np.linalg.norm(self.target_position - self.interceptor_position)
        )

        if distance >= self.threat_radius:
            return self.current_cruise_speed

        distance_ratio = max(distance, 0.0) / max(self.threat_radius, 1e-6)
        urgency = 1.0 - distance_ratio
        escape_floor = self.max_speed * self.escape_speed_min_ratio
        boosted_speed = self.current_cruise_speed + urgency * (
            self.max_speed - self.current_cruise_speed
        )

        return float(np.clip(boosted_speed, escape_floor, self.max_speed))

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


def main(args=None):
    rclpy.init(args=args)

    node = TargetBehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
