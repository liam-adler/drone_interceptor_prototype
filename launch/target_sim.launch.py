import os
from datetime import datetime

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
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    if controller_mode not in {"baseline", "mpc"}:
        raise RuntimeError(
            "Unknown controller_mode "
            f"'{controller_mode}'. Choose one of: baseline, mpc"
        )

    spawn_distance = float(LaunchConfiguration("spawn_distance").perform(context))
    spawn_distance_text = LaunchConfiguration("spawn_distance").perform(context)
    random_seed = int(LaunchConfiguration("random_seed").perform(context))
    open_rviz = _parse_bool(LaunchConfiguration("open_rviz").perform(context))
    open_plot = _parse_bool(LaunchConfiguration("open_plot").perform(context))
    threat_response = _parse_bool(
        LaunchConfiguration("threat_response").perform(context)
    )
    output_dir = LaunchConfiguration("output_dir").perform(context)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    threat_text = "threat_on" if threat_response else "threat_off"
    spawn_text = spawn_distance_text.replace(".", "p")
    run_label = (
        f"{timestamp}__mode_{controller_mode}__profile_{profile_name}"
        f"__{threat_text}__spawn_{spawn_text}m__seed_{random_seed}"
    )

    interceptor_state_topic = (
        "/interceptor/state"
        if controller_mode == "baseline"
        else "/interceptor_mpc/state"
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
                "interceptor_state_topic": interceptor_state_topic,
                "random_seed": random_seed,
            }],
        ),
    ]

    if controller_mode == "baseline":
        nodes.extend([
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
                parameters=[{
                    "target_state_topic": "/target/state",
                    "interceptor_state_topic": "/interceptor/state",
                    "distance_topic": "/intercept/distance",
                }],
            ),
            Node(
                package="drone_interceptor",
                executable="results_logger_node",
                name="results_logger_node",
                output="screen",
                parameters=[{
                    "distance_topic": "/intercept/distance",
                    "controller_label": "baseline",
                    "output_dir": output_dir,
                    "run_label": run_label,
                    "profile_name": profile_name,
                    "controller_mode": controller_mode,
                    "threat_response": threat_response,
                    "spawn_distance": spawn_distance,
                    "random_seed": random_seed,
                }],
            ),
        ])
    else:
        nodes.extend([
            Node(
                package="drone_interceptor",
                executable="interceptor_mpc_dynamics_node",
                name="interceptor_mpc_dynamics_node",
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
                executable="interceptor_mpc_node",
                name="interceptor_mpc_node",
                output="screen",
                parameters=[{
                    "max_speed": profile["interceptor_max_speed"],
                    "max_accel": profile["interceptor_max_accel"],
                }],
            ),
            Node(
                package="drone_interceptor",
                executable="distance_monitor_node",
                name="distance_monitor_node",
                output="screen",
                parameters=[{
                    "target_state_topic": "/target/state",
                    "interceptor_state_topic": "/interceptor_mpc/state",
                    "distance_topic": "/intercept/distance",
                }],
            ),
            Node(
                package="drone_interceptor",
                executable="results_logger_node",
                name="results_logger_node",
                output="screen",
                parameters=[{
                    "distance_topic": "/intercept/distance",
                    "controller_label": "mpc",
                    "output_dir": output_dir,
                    "run_label": run_label,
                    "profile_name": profile_name,
                    "controller_mode": controller_mode,
                    "threat_response": threat_response,
                    "spawn_distance": spawn_distance,
                    "random_seed": random_seed,
                }],
            ),
        ])

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
            "controller_mode",
            default_value="baseline",
            description="Interceptor controller selection: baseline or mpc",
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
            "random_seed",
            default_value="0",
            description="Seed for target behavior randomness; use matching seeds for fair comparisons",
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
        DeclareLaunchArgument(
            "output_dir",
            default_value="results/drone_interceptor",
            description="Folder for run CSV, summary JSON, and PNG plot outputs",
        ),
        OpaqueFunction(function=_build_nodes),
    ])
