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
            package='py_graph',
            executable='second_image_save',
            name='second_image_save_node',
            output='screen',
        ),
    ])
