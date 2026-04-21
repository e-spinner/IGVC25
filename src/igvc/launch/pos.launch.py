import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    igvc_share = get_package_share_directory("igvc")
    localization_params = os.path.join(igvc_share, "config", "localization.yaml")
    hall_params = os.path.join(
        get_package_share_directory("igvc_hall_odom"),
        "config",
        "hall_odom.yaml",
    )

    # Raw wheel integration (GPIO hall sensors). EKF publishes the authoritative odom->base_link.
    hall_odom = Node(
        package="igvc_hall_odom",
        executable="hall_odom_node",
        output="screen",
        parameters=[
            hall_params,
            {"simulate": LaunchConfiguration("hall_simulate")},
        ],
    )

    odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[localization_params, {"use_sim_time": False}],
        remappings=[("odometry/filtered", "/odometry/filtered/local")],
    )

    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[localization_params, {"use_sim_time": False}],
        remappings=[
            ("imu/data", "/vision/imu_front/imu/data_raw"),
            ("gps/fix", "/vision/fix"),
            ("odometry/filtered", "/odometry/filtered/local"),
            ("odometry/gps", "/odometry/gps"),
        ],
    )

    map_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[localization_params, {"use_sim_time": False}],
        remappings=[("odometry/filtered", "/odometry/filtered/global")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hall_simulate",
                default_value="false",
                description="Hall GPIO simulate mode (dev machines without RPi.GPIO).",
            ),
            hall_odom,
            odom_ekf,
            navsat,
            map_ekf,
        ]
    )
