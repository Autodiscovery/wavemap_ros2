"""Wavemap ROS2 server launch file."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wavemap_ros2')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            pkg_share, 'config', 'wavemap_livox_mid360.yaml'),
        description='Path to the wavemap configuration YAML file.')

    wavemap_node = Node(
        package='wavemap_ros2',
        executable='ros_server',
        name='wavemap',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
        }],
    )

    return LaunchDescription([
        config_file_arg,
        wavemap_node,
    ])
