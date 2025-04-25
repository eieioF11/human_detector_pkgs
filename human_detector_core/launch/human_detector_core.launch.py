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
    pkg_dir = get_package_share_directory('human_detector_core')
    list = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "0.16",
                "--y",
                "0.0",
                "--z",
                "0.37",
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
            package='omnivision',
            executable='fusion',
            name='fuser',
            output='screen',
            parameters=[
                {
                    #mid360 and ThetaV on Go2
                    'transformation_matrix': [
                        1, 0, 0, 0.090,
                        0, 1, 0, 0,
                        0, 0, 1, -0.135,
                        0, 0, 0, 1.0
                    ],
                    'pointcloud_topic': '/livox/lidar',
                    'image_topic': '/thetav/image_raw',
                    'depth_map': '/omnivision/depth',
                    'texturized_pointcloud': '/omnivision/textured_pointcloud',
                    'texturized_depth_map': '/omnivision/texturized_depth_map',
                    'image_overlay': '/omnivision/image_overlay'
                }
            ]
        ),
        # Node(
        #     package='mono_depth',
        #     executable='mono_depth',
        #     namespace='',
        #     # theta v
        #     remappings=[('image_raw', '/thetav/image_raw'),
        #                 ('camera_info', '/thetav/camera_info'),],
        #     output="screen",
        #     respawn=True,
        # ),
        Node(
            package='human_detector',
            executable='depth_human_detector',
            namespace='',
            # theta v
            remappings=[('image_raw', '/thetav/image_raw'),('depth', '/omnivision/depth'),],
            # remappings=[('image_raw', '/thetav/image_raw'),('depth', '/mono_depth/depth'),],
            output="screen",
            respawn=True,
        ),
        # Node(
        #     package='human_detector',
        #     executable='depth_human_detector',
        #     namespace='',
        #     # theta v
        #     remappings=[('image_raw', '/thetav/image_raw'),('depth', '/omnivision/depth'),],
        #     # remappings=[('image_raw', '/thetav/image_raw'),('depth', '/mono_depth/depth'),],
        #     output="screen",
        #     respawn=True,
        # ),
        # Node(
        #     package='panodepth_to_pc',
        #     executable='panodepth_to_pc',
        #     namespace='all',
        #     remappings=[('points', '/points'),
        #                 ('depth', '/mono_depth/depth'),],
        #     output="screen",
        #     respawn=True,
        # ),
        Node(
            package='panodepth_to_pc',
            executable='panodepth_to_pc',
            namespace='human',
            parameters=[os.path.join(pkg_dir, "config", "human_panodepth_to_pc_param.yaml")],
            remappings=[('points', '/human/points'),
                        ('depth', '/human_detector/depth_human_image'),],
            output="screen",
            respawn=True,
        ),
        # Node(
        #     package='scale_adjuster',
        #     executable='scale_adjuster',
        #     namespace='',
        #     # theta v
        #     remappings=[('in_points', '/points'),
        #                 ('out_points', '/adjust/points'),],
        #     output="screen",
        #     respawn=True,
        # ),
        # Node(
        #     package='human_detector_core',
        #     executable='human_detector_core',
        #     namespace='',
        #     # theta v
        #     remappings=[('in_points', '/human/points'),('scale', '/scale_adjuster/scale')],
        #     output="screen",
        #     respawn=True,
        # ),
    ]

    return LaunchDescription(list)