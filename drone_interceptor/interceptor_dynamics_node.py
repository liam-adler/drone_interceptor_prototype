#!/usr/bin/env python3

from __future__ import annotations

import rclpy

from drone_interceptor.point_mass_dynamics_node import (
    PointMassDynamicsNode,
    PointMassNodeConfig,
)


INTERCEPTOR_DYNAMICS_CONFIG = PointMassNodeConfig(
    node_name="interceptor_dynamics_node",
    log_label="Interceptor dynamics node",
    child_frame_id="interceptor",
    command_mode="velocity",
    state_topic="/interceptor/state",
    marker_topic="/interceptor/marker",
    path_marker_topic="/interceptor/path_marker",
    marker_namespace="interceptor",
    path_marker_namespace="interceptor_path",
    marker_scale=0.45,
    marker_color=(0.1, 0.4, 1.0, 1.0),
    command_topic="/interceptor/cmd_vel",
    command_topic_parameter="command_topic",
    command_parameter_default="/interceptor/cmd_vel",
)


class InterceptorDynamicsNode(PointMassDynamicsNode):
    def __init__(self) -> None:
        super().__init__(INTERCEPTOR_DYNAMICS_CONFIG)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = InterceptorDynamicsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
