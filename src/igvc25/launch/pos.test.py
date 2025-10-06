import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
  IncludeLaunchDescription,
  DeclareLaunchArgument,
  GroupAction,
)
from launch_ros.parameter_descriptions import ParameterValue

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
  LaunchConfiguration,
  PathJoinSubstitution,
  PythonExpression,
  Command,
)
from launch.conditions import IfCondition

from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
  package_name = "igvc25"

  # MARK: Args
  commands_file_arg = DeclareLaunchArgument(
    "commands_file",
    default_value=os.path.join(
      get_package_share_directory("igvc25"),
      "config",
      "root",
      "straight.json",
    ),
    description="Path to the commands JSON file",
  )

  # MARK: RSP
  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[
      {
        "robot_description": ParameterValue(
          Command(
            [
              "xacro ",
              os.path.join(
                get_package_share_directory(package_name),
                "description",
                "root.urdf",
              ),
            ]
          ),
          value_type=str,
        )
      }
    ],
  )

  # MARK: GTG
  truth_gen = Node(
    package="igvc25",
    executable="truth_gen",
    output="screen",
  )

  # MARK: ROOT
  root_controller = Node(
    package="igvc25",
    executable="root_controller.py",
    output="screen",
    parameters=[{"commands_file": LaunchConfiguration("commands_file")}],
  )

  # MARK: Launch!
  return LaunchDescription(
    [
      # Args
      commands_file_arg,
      # Nodes
      robot_state_publisher,
      truth_gen,
      root_controller,
    ]
  )
