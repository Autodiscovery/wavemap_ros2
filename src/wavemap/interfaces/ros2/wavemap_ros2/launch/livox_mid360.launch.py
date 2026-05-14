"""Wavemap with Livox MID-360 launch file for ROS2."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wavemap_ros2')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            pkg_share, 'config', 'wavemap_livox_mid360.yaml'),
        description='Path to the wavemap configuration YAML file.')

    # Static TF: odom -> camera_init
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_odom_camera_init',
        arguments=[
            '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0',
            'odom', 'camera_init',
        ],
    )

    # Static TF: imu_forward_prop -> livox_frame
    static_tf_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_body_livox_frame',
        arguments=[
            '-0.0110', '-0.02329', '0.04412', '0.0', '0.0', '0.0', '1.0',
            'imu_forward_prop', 'livox_frame',
        ],
    )

    # Wavemap server
    wavemap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'wavemap_server.launch.py')),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
        }.items(),
    )

    return LaunchDescription([
        config_file_arg,
        static_tf_odom,
        static_tf_body,
        wavemap_launch,
    ])
