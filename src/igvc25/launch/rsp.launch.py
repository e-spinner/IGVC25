import os
import xacro
import yaml

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


def load_robot_description(robot_description_path, robot_params_path):
  with open(robot_params_path, "r") as params:
    robot_params = yaml.safe_load(params)["/**"]["ros__parameters"]

  robot_description = xacro.process_file(
    robot_description_path,
    mappings={key: str(value) for key, value in robot_params.items()},
  )

  return robot_description.toxml()  # type: ignore


def generate_launch_description():
  package_name = "igvc25"
  package_path = get_package_share_directory(package_name)

  robot_description = load_robot_description(
    os.path.join(package_path, "description", "ackermann.urdf"),
    os.path.join(package_path, "config", "ackermann.yaml"),
  )

  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[
      {
        "robot_description": robot_description,
        "use_sim_time": True,
      }
    ],
  )

  # MARK

  # MARK: Launch!
  return LaunchDescription(
    [
      # Nodes
      robot_state_publisher,
    ]
  )
