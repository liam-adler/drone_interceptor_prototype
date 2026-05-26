#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


class DistanceMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("distance_monitor_node")

        self.target_position: Optional[np.ndarray] = None
        self.interceptor_position: Optional[np.ndarray] = None

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
        self.distance_pub = self.create_publisher(
            Float32,
            "/intercept/distance",
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

        distance = float(
            np.linalg.norm(self.target_position - self.interceptor_position)
        )

        msg = Float32()
        msg.data = distance
        self.distance_pub.publish(msg)

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
