import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
  IncludeLaunchDescription,
  DeclareLaunchArgument,
  GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
  LaunchConfiguration,
  PathJoinSubstitution,
  PythonExpression,
)
from launch.conditions import IfCondition

from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
  package_name = "igvc"

  localization_params = os.path.join(
    get_package_share_directory(package_name), "config", "localization.yaml"
  )

  # MARK: odom lztn
  odom_localization = Node(
    package="robot_localization",
    executable="ekf_node",
    name="ekf_filter_node_odom",
    output="screen",
    parameters=[localization_params, {"use_sim_time": True}],
    remappings=[("odometry/filtered", "odometry/filtered/local")],
  )
  # MARK: map  lztn
  map_localization = Node(
    package="robot_localization",
    executable="ekf_node",
    name="ekf_filter_node_map",
    output="screen",
    parameters=[localization_params, {"use_sim_time": True}],
    remappings=[("odometry/filtered", "odometry/filtered/global")],
  )

  navsat_transform = Node(
    package="robot_localization",
    executable="navsat_transform_node",
    name="navsat_transform",
    output="screen",
    parameters=[localization_params, {"use_sim_time": True}],
    remappings=[
      # input
      ("imu/data", "imu/data"),
      ("gps/fix", "gps/fix"),
      ("odometry/filtered", "odometry/global"),
      # output
      ("odometry/gps", "odometry/gps"),
    ],
  )
  # MARK: Launch!
  return LaunchDescription(
    [
      odom_localization,
      map_localization,
      navsat_transform,
    ]
  )
