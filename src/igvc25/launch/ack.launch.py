import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

import xacro


def generate_launch_description():
  package_name = "igvc25"

  cmd_interpreter = Node(
    package=package_name,
    executable="cmd_interpreter",
  )

  ack_calc = Node(
    package=package_name,
    executable="ack_calc.py",
  )

  transform = Node(
    package=package_name,
    executable="transform",
  )

  pkg_path = os.path.join(get_package_share_directory("hack13"))
  xacro_file = os.path.join(pkg_path, "description", "ackermann.urdf.xacro")
  robot_description = xacro.process_file(xacro_file).toxml()  # type: ignore

  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[
      {
        "robot_description": robot_description,
        "use_sim_time": False,
      }
    ],
  )

  # MARK: Launch!
  return LaunchDescription(
    [
      # Nodes
      cmd_interpreter,
      ack_calc,
      transform,
      robot_state_publisher,
    ]
  )
