import os
import xacro
import re
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription,
  OpaqueFunction,
  RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def load_robot_description(robot_description_path, robot_params_path):
  with open(robot_params_path, "r") as params:
    robot_params = yaml.safe_load(params)["/**"]["ros__parameters"]

  robot_description = xacro.process_file(
    robot_description_path,
    mappings={key: str(value) for key, value in robot_params.items()},
  )

  return robot_description.toxml()  # type: ignore


def launch_setup(context, *args, **kwargs):
  igvc_package = "igvc"

  igvc_path = get_package_share_directory(igvc_package)

  # MARK: Robot Description
  # -----------------------------------------------------------------------------
  robot_description = load_robot_description(
    os.path.join(igvc_path, "description", "ackermann_ac.urdf"),
    os.path.join(igvc_path, "config", "ackermann.yaml"),
  )

  ros2_control_config_file = os.path.join(igvc_path, "config", "ros2_control.yaml")

  # Robot State Publisher
  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[{"robot_description": robot_description}],
  )

  # MARK: Controller Manager
  # -----------------------------------------------------------------------------
  controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[ros2_control_config_file, {"robot_description": robot_description}],
    output="screen",
    remappings=[("/controller_manager/robot_description", "/robot_description")],
  )

  # MARK: Joint State Broadcaster
  # -----------------------------------------------------------------------------
  joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"],
    output="screen",
  )

  # MARK: Ackermann Angle Controller
  # -----------------------------------------------------------------------------
  ackermann_angle_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["ackermann_angle_controller"],
    output="screen",
  )

  return [
    robot_state_publisher,
    controller_manager,
    joint_state_broadcaster_spawner,
    ackermann_angle_controller_spawner,
  ]


# MARK: Launch!
# -----------------------------------------------------------------------------
def generate_launch_description():
  return LaunchDescription([OpaqueFunction(function=launch_setup)])
