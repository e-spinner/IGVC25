import os

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
  igvc_path = get_package_share_directory("igvc")

  urdf_file = os.path.basename(LaunchConfiguration("urdf_file").perform(context).strip())
  use_sim_time = (
    LaunchConfiguration("use_sim_time").perform(context).strip().lower()
    in ["true", "1", "yes", "on"]
  )

  robot_description_path = os.path.join(igvc_path, "description", urdf_file)
  robot_description = xacro.process_file(robot_description_path).toxml()  # type: ignore

  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[
      {
        "robot_description": robot_description,
        "use_sim_time": use_sim_time,
      }
    ],
  )

  return [robot_state_publisher]


def generate_launch_description():
  return LaunchDescription(
    [
      DeclareLaunchArgument(
        "urdf_file",
        default_value="ackermann_ac.urdf",
        description="URDF filename in the igvc package description share folder",
      ),
      DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true",
      ),
      OpaqueFunction(function=launch_setup),
    ]
  )
