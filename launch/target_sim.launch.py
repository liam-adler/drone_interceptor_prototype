import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PROFILES = {
    "matched": {
        "target_min_speed": 1.0,
        "target_max_speed": 4.0,
        "target_max_accel": 3.0,
        "interceptor_max_speed": 4.0,
        "interceptor_max_accel": 3.0,
    },
    "target_faster": {
        "target_min_speed": 1.5,
        "target_max_speed": 4.5,
        "target_max_accel": 3.0,
        "interceptor_max_speed": 4.0,
        "interceptor_max_accel": 3.0,
    },
    "target_more_maneuverable": {
        "target_min_speed": 1.0,
        "target_max_speed": 4.0,
        "target_max_accel": 3.8,
        "interceptor_max_speed": 4.0,
        "interceptor_max_accel": 3.0,
    },
    "target_advantaged": {
        "target_min_speed": 1.5,
        "target_max_speed": 4.5,
        "target_max_accel": 3.5,
        "interceptor_max_speed": 4.0,
        "interceptor_max_accel": 3.0,
    },
}


def _parse_bool(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def _build_nodes(context):
    package_share = get_package_share_directory("drone_interceptor")
    rviz_config = os.path.join(package_share, "rviz", "target_sim.rviz")

    profile_name = LaunchConfiguration("profile").perform(context)
    if profile_name not in PROFILES:
        valid_profiles = ", ".join(sorted(PROFILES))
        raise RuntimeError(
            f"Unknown profile '{profile_name}'. Choose one of: {valid_profiles}"
        )

    profile = PROFILES[profile_name]
    spawn_distance = float(LaunchConfiguration("spawn_distance").perform(context))
    open_rviz = _parse_bool(LaunchConfiguration("open_rviz").perform(context))
    open_plot = _parse_bool(LaunchConfiguration("open_plot").perform(context))
    threat_response = _parse_bool(
        LaunchConfiguration("threat_response").perform(context)
    )

    nodes = [
        Node(
            package="drone_interceptor",
            executable="target_dynamics_node",
            name="target_dynamics_node",
            output="screen",
            parameters=[{
                "initial_x": 0.0,
                "initial_y": 0.0,
                "initial_z": 2.0,
                "max_speed": profile["target_max_speed"],
                "max_accel": profile["target_max_accel"],
            }],
        ),
        Node(
            package="drone_interceptor",
            executable="target_behavior_node",
            name="target_behavior_node",
            output="screen",
            parameters=[{
                "min_speed": profile["target_min_speed"],
                "max_speed": profile["target_max_speed"],
                "enable_threat_response": threat_response,
                "threat_radius": 12.0,
                "escape_speed_min": 0.9,
            }],
        ),
        Node(
            package="drone_interceptor",
            executable="interceptor_dynamics_node",
            name="interceptor_dynamics_node",
            output="screen",
            parameters=[{
                "initial_x": -spawn_distance,
                "initial_y": 0.0,
                "initial_z": 2.0,
                "max_speed": profile["interceptor_max_speed"],
                "max_accel": profile["interceptor_max_accel"],
            }],
        ),
        Node(
            package="drone_interceptor",
            executable="interceptor_guidance_node",
            name="interceptor_guidance_node",
            output="screen",
            parameters=[{
                "interceptor_speed": profile["interceptor_max_speed"],
            }],
        ),
        Node(
            package="drone_interceptor",
            executable="distance_monitor_node",
            name="distance_monitor_node",
            output="screen",
        ),
    ]

    if open_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            )
        )

    if open_plot:
        nodes.append(
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "run",
                            "rqt_plot",
                            "rqt_plot",
                        ],
                        output="screen",
                    )
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "profile",
            default_value="target_advantaged",
            description=(
                "Vehicle capability profile: matched, target_faster, "
                "target_more_maneuverable, target_advantaged"
            ),
        ),
        DeclareLaunchArgument(
            "threat_response",
            default_value="true",
            description="Enable close-range target speed boost",
        ),
        DeclareLaunchArgument(
            "spawn_distance",
            default_value="30.0",
            description="Initial separation distance in meters",
        ),
        DeclareLaunchArgument(
            "open_rviz",
            default_value="true",
            description="Open RViz with the simulation config",
        ),
        DeclareLaunchArgument(
            "open_plot",
            default_value="true",
            description="Open rqt_plot for /intercept/distance",
        ),
        OpaqueFunction(function=_build_nodes),
    ])
