import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
  IncludeLaunchDescription,
  DeclareLaunchArgument,
  OpaqueFunction,
)
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


def launch_setup(context, *args, **kwargs):
  package_name = "igvc_control"
  pkg_path = get_package_share_directory(package_name)

  # Get linkage config number from launch argument
  linkage_config_num = LaunchConfiguration("linkage_config").perform(context)

  # Load ackermann linkage parameters from selected yaml
  linkage_config_file = os.path.join(
    pkg_path, "config", f"ackermann_linkage_{linkage_config_num}.yaml"
  )
  with open(linkage_config_file, "r") as f:
    full_config = yaml.safe_load(f)

  linkage_params = full_config["/**"]["ros__parameters"]

  cmd_interpreter = Node(
    package=package_name,
    executable="cmd_interpreter",
    parameters=[
      linkage_params
    ],  # Pass linkage parameters (includes wheel_base)
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
    linkage_config_file,
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

  # Joy params are now in linkage config files
  joy_node = Node(
    package="joy",
    executable="joy_node",
    parameters=[linkage_config_file],
  )
  teleop_joy = Node(
    package="teleop_twist_joy",
    executable="teleop_node",
    name="teleop_node",
    parameters=[linkage_config_file],
  )

  return [
    cmd_interpreter,
    ack_calc,
    transform,
    robot_state_publisher,
    joy_node,
    teleop_joy,
  ]


def generate_launch_description():
  # Declare launch argument for linkage config selection
  linkage_config_arg = DeclareLaunchArgument(
    "linkage_config",
    default_value="2",
    description="Linkage configuration number",
  )

  # MARK: Launch!
  return LaunchDescription(
    [
      linkage_config_arg,
      OpaqueFunction(function=launch_setup),
    ]
  )
