#!/usr/bin/env python3

# Author: Brighten Lee

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mw_ahrs_ros2",
                executable="mw_ahrs_driver_node",
                name="mw_ahrs_driver_node",
                output="screen",
                parameters=[
                    {
                        "port": "/dev/ttyUSB0",
                        "frame_id": "imu_link",
                    }
                ],
            ),
        ]
    )
