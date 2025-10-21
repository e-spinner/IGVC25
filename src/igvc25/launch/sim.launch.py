import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
  IncludeLaunchDescription,
  DeclareLaunchArgument,
  GroupAction,
  RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
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

  world = os.path.join(package_path, "worlds", "plane.sdf")

  gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
      )
    ),
    launch_arguments={
      "gz_args": f"-r -v 4 {world}",
      "on_exit_shutdown": "true",
    }.items(),
  )

  robot_description = load_robot_description(
    os.path.join(package_path, "description", "ackermann.urdf"),
    os.path.join(package_path, "config", "ackermann.yaml"),
  )

  spawn_robot = Node(
    package="ros_gz_sim",
    executable="create",
    output="screen",
    arguments=[
      "-name",
      "robot",
      "-string",
      robot_description,
      "-x",
      "0",
      "-y",
      "0",
      "-z",
      "0.5",
    ],
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

  joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"],
  )

  controller_params = os.path.join(
    package_path, "config", "gz_ros2_control.yaml"
  )

  ackermann_steering_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
      "ackermann_steering_controller",
      "-p",
      controller_params,
    ],
  )

  bridge_params = os.path.join(package_path, "config", "ros_gz_bridge.yaml")

  gz_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    output="screen",
    arguments=[
      "--ros-args",
      "-p",
      f"config_file:={bridge_params}",
    ],
  )

  # MARK

  # MARK: Launch!
  return LaunchDescription(
    [
      # Nodes
      gazebo_launch,
      RegisterEventHandler(
        event_handler=OnProcessExit(
          target_action=spawn_robot,
          on_exit=[joint_state_broadcaster_spawner],
        )
      ),
      RegisterEventHandler(
        event_handler=OnProcessExit(
          target_action=joint_state_broadcaster_spawner,
          on_exit=[ackermann_steering_controller_spawner],
        )
      ),
      spawn_robot,
      robot_state_publisher,
      gz_bridge,
    ]
  )
