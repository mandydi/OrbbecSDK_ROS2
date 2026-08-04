#!/usr/bin/env python3
"""Launch the left-arm Gemini 305 under /vision/camera/left_arm_rgbd."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    serial_number = LaunchConfiguration("serial_number")
    align_mode = LaunchConfiguration("align_mode")

    orbbec_launch = os.path.join(
        get_package_share_directory("orbbec_camera"),
        "launch",
        "gemini305.launch.py",
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_launch),
        launch_arguments={
            # camera_name must be a legal ROS node name; it cannot contain '/'.
            "camera_name": "left_arm_rgbd",
            "serial_number": serial_number,
            "color_width": "1280",
            "color_height": "800",
            "color_fps": "10",
            "color_format": "MJPG",
            "depth_registration": "true",
            "align_mode": align_mode,
            "align_target_stream": "COLOR",
            "enable_frame_sync": "false",
            "enable_point_cloud": "false",
            "enable_colored_point_cloud": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_number",
                default_value="CV2T661000VN",
                description="Left-arm Gemini 305 serial number",
            ),
            DeclareLaunchArgument(
                "align_mode",
                default_value="HW",
                description="D2C alignment mode: HW or SW",
                choices=["HW", "SW"],
            ),
            GroupAction(
                [
                    PushRosNamespace("vision/camera"),
                    camera,
                ]
            ),
        ]
    )
