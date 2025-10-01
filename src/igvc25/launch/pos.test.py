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
  temp = LaunchConfiguration("temp")

  temp_arg = DeclareLaunchArgument(
    "temp", default_value="temp", description="temp"
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
                "imu_test.urdf",
              ),
            ]
          ),
          value_type=str,
        )
      }
    ],
  )

  # MARK: Launch!
  return LaunchDescription(
    [
      # Args
      temp_arg,
      # Nodes
      robot_state_publisher,
    ]
  )
