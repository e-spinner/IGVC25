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

  teleop = Node(
    package="teleop_twist_keyboard",
    executable="teleop_twist_keyboard",
    name="teleop_keyboard",
    output="screen",
    remappings=[("/cmd_vel", "/cmd_vel_teleop_key")],
    # parameters=[{"stamped": True}],
    prefix="xterm -e",  # Opens in a new terminal window
  )

  joystick_params = os.path.join(
    get_package_share_directory("igvc"), "config", "joystick.yaml"
  )

  joy_node = Node(
    package="joy",
    executable="joy_node",
    parameters=[joystick_params],
  )
  teleop_joy = Node(
    package="teleop_twist_joy",
    executable="teleop_node",
    name="teleop_node",
    parameters=[joystick_params],
  )

  # https://github.com/ros-teleop/twist_mux
  twist_mux_params = os.path.join(
    get_package_share_directory("igvc"), "config", "twist_mux.yaml"
  )

  twist_mux = Node(
    package="twist_mux",
    executable="twist_mux",
    parameters=[twist_mux_params, {"use_sim_time": True}],
    remappings=[("/cmd_vel_out", "/cmd_vel_unstamped")],
  )

  # https://github.com/joshnewans/twist_stamper
  twist_stamper = Node(
    package="twist_stamper",
    executable="twist_stamper",
    parameters=[{"use_sim_time": True}],
    remappings=[
      ("/cmd_vel_in", "/cmd_vel_unstamped"),
      ("/cmd_vel_out", "/cmd_vel_stamped"),
    ],
  )

  # MARK

  # MARK: Launch!
  return LaunchDescription(
    [
      # Nodes
      teleop,
      joy_node,
      teleop_joy,
      twist_mux,
      twist_stamper,
    ]
  )
