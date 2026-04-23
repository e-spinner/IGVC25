import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("igvc_odom"),
        "config",
        "hall_odom.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="igvc_odom",
                executable="hall_odom_node",
                output="screen",
                parameters=[cfg],
            ),
        ]
    )
