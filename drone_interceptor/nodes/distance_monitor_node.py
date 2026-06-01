#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32

from drone_interceptor.capture.radius_capture import compute_distance


class DistanceMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("distance_monitor_node")

        self.declare_parameter("target_state_topic", "/target/state")
        self.declare_parameter("interceptor_state_topic", "/interceptor/state")
        self.declare_parameter("distance_topic", "/intercept/distance")

        target_state_topic = self.get_string_parameter("target_state_topic")
        interceptor_state_topic = self.get_string_parameter(
            "interceptor_state_topic"
        )
        distance_topic = self.get_string_parameter("distance_topic")

        self.target_position: Optional[np.ndarray] = None
        self.interceptor_position: Optional[np.ndarray] = None

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
        self.distance_pub = self.create_publisher(
            Float32,
            distance_topic,
            10,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

    def target_callback(self, msg: Odometry) -> None:
        self.target_position = self.extract_position(msg)

    def interceptor_callback(self, msg: Odometry) -> None:
        self.interceptor_position = self.extract_position(msg)

    def timer_callback(self) -> None:
        if self.target_position is None or self.interceptor_position is None:
            return

        msg = Float32()
        msg.data = self.compute_distance()
        self.distance_pub.publish(msg)

    def compute_distance(self) -> float:
        assert self.target_position is not None
        assert self.interceptor_position is not None
        return compute_distance(self.target_position, self.interceptor_position)

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


def main(args=None) -> None:
    rclpy.init(args=args)

    node = DistanceMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
