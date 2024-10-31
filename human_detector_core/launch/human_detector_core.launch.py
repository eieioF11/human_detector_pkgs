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
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "0.0",
                "--y",
                "0.08",
                "--z",
                "1.63",
                # "0.95",
                # "1.25",
                "--yaw",
                "0.0",
                # "-1.5707",
                "--pitch",
                "0.0",
                "--roll",
                "0.0",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "camera_link",
            ],
            # parameters=[{'use_sim_time': True}]
        ),
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
        Node(
            package='scale_adjuster',
            executable='scale_adjuster',
            namespace='',
            # theta v
            remappings=[('in_points', '/points'),
                        ('out_points', '/adjust/points'),],
            output="screen",
            respawn=True,
        ),
        Node(
            package='human_detector_core',
            executable='human_detector_core',
            namespace='',
            # theta v
            remappings=[('in_points', '/human/points'),('scale', '/scale_adjuster/scale')],
            output="screen",
            respawn=True,
        ),
    ]

    return LaunchDescription(list)