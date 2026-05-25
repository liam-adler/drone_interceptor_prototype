from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_interceptor',
            executable='target_dynamics_node',
            name='target_dynamics_node',
            output='screen',
        ),
        Node(
            package='drone_interceptor',
            executable='target_behavior_node',
            name='target_behavior_node',
            output='screen',
        ),
    ])