#!/usr/bin/env python3

from __future__ import annotations

from copy import deepcopy
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker

from drone_interceptor.estimation import (
    ConstantVelocityKalmanFilter,
    KalmanFilterConfig,
)
from drone_interceptor.visualization.rviz_markers import build_sphere_marker


class EstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("estimator_node")

        self.declare_parameter("input_state_topic", "/target/state")
        self.declare_parameter("measurement_topic", "/target/state_noisy")
        self.declare_parameter("estimated_state_topic", "/target/estimated_state")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("measurement_marker_topic", "/target/measurement_marker")
        self.declare_parameter("estimated_marker_topic", "/target/estimated_marker")

        self.declare_parameter("noise_enabled", False)
        self.declare_parameter("position_noise_stddev", 0.0)
        self.declare_parameter("random_seed", 0)

        self.declare_parameter("filter_enabled", True)
        self.declare_parameter("process_acceleration_stddev", 2.0)
        self.declare_parameter("measurement_position_stddev", -1.0)
        self.declare_parameter("initial_position_stddev", 1.0)
        self.declare_parameter("initial_velocity_stddev", 1.0)

        input_state_topic = self.get_string_parameter("input_state_topic")
        measurement_topic = self.get_string_parameter("measurement_topic")
        estimated_state_topic = self.get_string_parameter("estimated_state_topic")
        self.frame_id = self.get_string_parameter("frame_id")
        measurement_marker_topic = self.get_string_parameter(
            "measurement_marker_topic"
        )
        estimated_marker_topic = self.get_string_parameter("estimated_marker_topic")

        self.noise_enabled = self.get_bool_parameter("noise_enabled")
        self.position_noise_stddev = self.get_nonnegative_float_parameter(
            "position_noise_stddev"
        )
        random_seed = self.get_int_parameter("random_seed")

        self.filter_enabled = self.get_bool_parameter("filter_enabled")
        measurement_position_stddev = self.resolve_measurement_stddev(
            parameter_name="measurement_position_stddev",
            fallback_stddev=self.position_noise_stddev,
        )

        self.filter = ConstantVelocityKalmanFilter(
            KalmanFilterConfig(
                process_acceleration_stddev=self.get_nonnegative_float_parameter(
                    "process_acceleration_stddev"
                ),
                measurement_position_stddev=measurement_position_stddev,
                initial_position_stddev=self.get_nonnegative_float_parameter(
                    "initial_position_stddev"
                ),
                initial_velocity_stddev=self.get_nonnegative_float_parameter(
                    "initial_velocity_stddev"
                ),
            )
        )
        self.rng = np.random.default_rng(random_seed)
        self.last_stamp_seconds: Optional[float] = None
        self.last_measurement_position: Optional[np.ndarray] = None
        self.last_measurement_velocity = np.zeros(3, dtype=float)

        self.measurement_pub = self.create_publisher(Odometry, measurement_topic, 10)
        self.estimate_pub = self.create_publisher(Odometry, estimated_state_topic, 10)
        self.measurement_marker_pub = self.create_publisher(
            Marker,
            measurement_marker_topic,
            10,
        )
        self.estimated_marker_pub = self.create_publisher(
            Marker,
            estimated_marker_topic,
            10,
        )
        self.create_subscription(
            Odometry,
            input_state_topic,
            self.input_state_callback,
            10,
        )

        self.get_logger().info(
            "Estimator node started with "
            f"noise_enabled={self.noise_enabled}, "
            f"filter_enabled={self.filter_enabled}, "
            f"input_topic='{input_state_topic}', "
            f"measurement_topic='{measurement_topic}', "
            f"estimated_topic='{estimated_state_topic}'"
        )

    def input_state_callback(self, msg: Odometry) -> None:
        raw_position = self.extract_position(msg)
        stamp_seconds = self.stamp_to_seconds(msg)

        measured_position = raw_position.copy()

        if self.noise_enabled:
            measured_position += self.rng.normal(
                loc=0.0,
                scale=self.position_noise_stddev,
                size=3,
            )
        measured_velocity = self.compute_measured_velocity(
            measured_position=measured_position,
            stamp_seconds=stamp_seconds,
        )

        measurement_msg = self.build_output_message(
            template=msg,
            position=measured_position,
            velocity=measured_velocity,
            child_frame_id_suffix="_measurement",
        )
        self.measurement_pub.publish(measurement_msg)
        self.publish_marker(
            publisher=self.measurement_marker_pub,
            stamp=msg.header.stamp,
            position=measured_position,
            namespace="target_measurement",
            marker_id=0,
            scale=0.34,
            color=(1.0, 0.55, 0.1, 0.9),
        )

        estimated_position = measured_position
        estimated_velocity = measured_velocity

        if self.filter_enabled:
            if not self.filter.initialized:
                self.filter.initialize(
                    position=measured_position,
                    velocity=measured_velocity,
                )
                self.last_stamp_seconds = self.stamp_to_seconds(msg)
            else:
                dt = self.compute_dt_seconds(msg)
                if dt is not None:
                    self.filter.predict(dt)
                self.filter.update(position=measured_position)

            estimated_position = self.filter.position
            estimated_velocity = self.filter.velocity

        estimate_msg = self.build_output_message(
            template=msg,
            position=estimated_position,
            velocity=estimated_velocity,
            child_frame_id_suffix="_estimated",
        )
        self.estimate_pub.publish(estimate_msg)
        self.publish_marker(
            publisher=self.estimated_marker_pub,
            stamp=msg.header.stamp,
            position=estimated_position,
            namespace="target_estimate",
            marker_id=0,
            scale=0.32,
            color=(0.1, 0.9, 0.3, 0.9),
        )

    def compute_dt_seconds(self, msg: Odometry) -> float | None:
        stamp_seconds = self.stamp_to_seconds(msg)
        if self.last_stamp_seconds is None:
            self.last_stamp_seconds = stamp_seconds
            return None

        dt = stamp_seconds - self.last_stamp_seconds
        self.last_stamp_seconds = stamp_seconds
        if dt <= 0.0 or dt > 1.0:
            return None
        return dt

    def resolve_measurement_stddev(
        self,
        *,
        parameter_name: str,
        fallback_stddev: float,
    ) -> float:
        configured_value = float(self.get_parameter(parameter_name).value)
        if configured_value >= 0.0:
            return configured_value
        return fallback_stddev

    def compute_measured_velocity(
        self,
        *,
        measured_position: np.ndarray,
        stamp_seconds: float,
    ) -> np.ndarray:
        if self.last_measurement_position is None or self.last_stamp_seconds is None:
            velocity = np.zeros(3, dtype=float)
        else:
            dt = stamp_seconds - self.last_stamp_seconds
            if dt <= 0.0 or dt > 1.0:
                velocity = self.last_measurement_velocity.copy()
            else:
                velocity = (
                    measured_position - self.last_measurement_position
                ) / dt

        self.last_measurement_position = measured_position.copy()
        self.last_measurement_velocity = velocity.copy()
        return velocity

    @staticmethod
    def stamp_to_seconds(msg: Odometry) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def build_output_message(
        self,
        *,
        template: Odometry,
        position: np.ndarray,
        velocity: np.ndarray,
        child_frame_id_suffix: str,
    ) -> Odometry:
        msg = deepcopy(template)
        msg.child_frame_id = f"{template.child_frame_id}{child_frame_id_suffix}"
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = float(position[2])
        msg.twist.twist.linear.x = float(velocity[0])
        msg.twist.twist.linear.y = float(velocity[1])
        msg.twist.twist.linear.z = float(velocity[2])
        return msg

    def publish_marker(
        self,
        *,
        publisher,
        stamp,
        position: np.ndarray,
        namespace: str,
        marker_id: int,
        scale: float,
        color: tuple[float, float, float, float],
    ) -> None:
        marker = build_sphere_marker(
            stamp=stamp,
            frame_id=self.frame_id,
            namespace=namespace,
            marker_id=marker_id,
            position=position,
            scale=scale,
            color=color,
        )
        publisher.publish(marker)

    def get_bool_parameter(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def get_int_parameter(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def get_nonnegative_float_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        return max(value, 0.0)

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
    def extract_velocity(msg: Odometry) -> np.ndarray:
        return np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ],
            dtype=float,
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = EstimatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
