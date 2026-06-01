#!/usr/bin/env python3

from __future__ import annotations

import rclpy

from drone_interceptor.nodes.point_mass_dynamics_node import (
    PointMassDynamicsNode,
    PointMassNodeConfig,
)


INTERCEPTOR_MPC_DYNAMICS_CONFIG = PointMassNodeConfig(
    node_name="interceptor_mpc_dynamics_node",
    log_label="Interceptor MPC dynamics node",
    child_frame_id="interceptor_mpc",
    command_mode="acceleration",
    state_topic="/interceptor_mpc/state",
    marker_topic="/interceptor_mpc/marker",
    path_marker_topic="/interceptor_mpc/path_marker",
    marker_namespace="interceptor_mpc",
    path_marker_namespace="interceptor_mpc_path",
    marker_scale=0.42,
    marker_color=(0.0, 0.85, 0.5, 1.0),
    command_topic="/interceptor_mpc/cmd_accel",
    command_topic_parameter="command_topic",
    command_parameter_default="/interceptor_mpc/cmd_accel",
)


class InterceptorMpcDynamicsNode(PointMassDynamicsNode):
    def __init__(self) -> None:
        super().__init__(INTERCEPTOR_MPC_DYNAMICS_CONFIG)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = InterceptorMpcDynamicsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
