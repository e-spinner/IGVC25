import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

import xacro


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
  pkg_path = get_package_share_directory(package_name)

  # Load ackermann linkage parameters from yaml
  linkage_config_file = os.path.join(
    pkg_path, "config", "ackermann_linkage_2.yaml"
  )
  with open(linkage_config_file, "r") as f:
    linkage_params = yaml.safe_load(f)["/**"]["ros__parameters"]

  cmd_interpreter = Node(
    package=package_name,
    executable="cmd_interpreter",
  )

  ack_calc = Node(
    package=package_name,
    executable="ack_calc.py",
    parameters=[linkage_params],  # Pass linkage parameters to the node
  )

  transform = Node(
    package=package_name,
    executable="transform",
  )

  robot_description = load_robot_description(
    os.path.join(pkg_path, "description", "ackermann_linkage.urdf"),
    os.path.join(pkg_path, "config", "ackermann_linkage_2.yaml"),
  )

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

  # https://github.com/ros-teleop/teleop_twist_joy
  joy_params = os.path.join(
    get_package_share_directory(package_name), "config", "joy_params.yaml"
  )
  joy_node = Node(
    package="joy",
    executable="joy_node",
    parameters=[joy_params],
  )
  teleop_joy = Node(
    package="teleop_twist_joy",
    executable="teleop_node",
    name="teleop_node",
    parameters=[joy_params],
  )

  # MARK: Launch!
  return LaunchDescription(
    [
      # Nodes
      cmd_interpreter,
      ack_calc,
      transform,
      robot_state_publisher,
      joy_node,
      teleop_joy,
    ]
  )
