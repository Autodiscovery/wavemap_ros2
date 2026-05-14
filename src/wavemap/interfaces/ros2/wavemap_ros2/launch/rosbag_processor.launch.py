"""Wavemap rosbag processor launch file."""
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

    rosbag_path_arg = DeclareLaunchArgument(
        'rosbag_path',
        default_value='',
        description='Path to the ROS2 bag directory to process.')

    output_path_arg = DeclareLaunchArgument(
        'output_path',
        default_value='',
        description='Path to save the resulting map file.')

    rosbag_processor_node = Node(
        package='wavemap_ros2',
        executable='rosbag_processor',
        name='wavemap_rosbag_processor',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'rosbag_path': LaunchConfiguration('rosbag_path'),
            'output_path': LaunchConfiguration('output_path'),
        }],
    )

    return LaunchDescription([
        config_file_arg,
        rosbag_path_arg,
        output_path_arg,
        rosbag_processor_node,
    ])
