import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("igvc_hall_odom"),
        "config",
        "hall_odom.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simulate",
                default_value="false",
                description="If true, generate fake wheel speeds (no GPIO).",
            ),
            Node(
                package="igvc_hall_odom",
                executable="hall_odom_node",
                output="screen",
                parameters=[
                    cfg,
                    {"simulate": LaunchConfiguration("simulate")},
                ],
            ),
        ]
    )
