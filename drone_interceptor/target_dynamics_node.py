#!/usr/bin/env python3

from __future__ import annotations

import rclpy

from drone_interceptor.point_mass_dynamics_node import (
    PointMassDynamicsNode,
    PointMassNodeConfig,
)


TARGET_DYNAMICS_CONFIG = PointMassNodeConfig(
    node_name="target_dynamics_node",
    log_label="Target dynamics node",
    child_frame_id="target",
    command_mode="velocity",
    state_topic="/target/state",
    marker_topic="/target/marker",
    path_marker_topic="/target/path_marker",
    marker_namespace="target",
    path_marker_namespace="target_path",
    marker_scale=0.45,
    marker_color=(1.0, 0.2, 0.2, 1.0),
    command_topic="/target/cmd_vel",
    command_topic_parameter="command_topic",
    command_parameter_default="/target/cmd_vel",
    enable_heading_speed_input=True,
    desired_heading_topic="/target/desired_heading",
    desired_speed_topic="/target/desired_speed",
)


class TargetDynamicsNode(PointMassDynamicsNode):
    def __init__(self) -> None:
        super().__init__(TARGET_DYNAMICS_CONFIG)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = TargetDynamicsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
