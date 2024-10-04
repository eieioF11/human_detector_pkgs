import os
import sys
from glob import glob
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('mono_depth')
    list = [
        Node(
            package='mono_depth',
            executable='mono_depth',
            namespace='',
            # theta v
            remappings=[('image_raw', '/thetav/image_raw'),
                        ('camera_info', '/thetav/camera_info'),],
            output="screen",
            respawn=True,
        ),
        Node(
            package='human_detector',
            executable='depth_human_detector',
            namespace='',
            # theta v
            remappings=[('image_raw', '/thetav/image_raw'),('depth', '/mono_depth/depth')],
            output="screen",
            respawn=True,
        ),
        Node(
            package='panodepth_to_pc',
            executable='panodepth_to_pc',
            namespace='all',
            remappings=[('points', '/points'),
                        ('depth', '/mono_depth/depth'),],
            output="screen",
            respawn=True,
        ),
        Node(
            package='panodepth_to_pc',
            executable='panodepth_to_pc',
            namespace='human',
            remappings=[('points', '/human/points'),
                        ('depth', '/human_detector/depth_human_image'),],
            output="screen",
            respawn=True,
        ),
    ]

    return LaunchDescription(list)