import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ram_detect',
            executable='object_depth_detect',
            name='object_depth_detect_node',
            output='screen',
        ),
        Node(
            package='ram_detect',
            executable='two_three',
            name='two_three_node',
            output='screen',
        ),
    ])
