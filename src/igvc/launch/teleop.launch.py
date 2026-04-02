import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  package_name = "igvc"
  use_sim_time = LaunchConfiguration("use_sim_time")

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
    parameters=[joystick_params, {"use_sim_time": use_sim_time}],
  )
  teleop_joy = Node(
    package="teleop_twist_joy",
    executable="teleop_node",
    name="teleop_node",
    remappings=[("/cmd_vel", "/cmd_vel_teleop_joy")],
    parameters=[joystick_params, {"use_sim_time": use_sim_time}],
  )

  # https://github.com/ros-teleop/twist_mux
  twist_mux_params = os.path.join(
    get_package_share_directory("igvc"), "config", "twist_mux.yaml"
  )

  twist_mux = Node(
    package="twist_mux",
    executable="twist_mux",
    parameters=[twist_mux_params, {"use_sim_time": use_sim_time}],
    remappings=[("/cmd_vel_out", "/cmd_vel_unstamped")],
  )

  # https://github.com/joshnewans/twist_stamper
  twist_stamper = Node(
    package="twist_stamper",
    executable="twist_stamper",
    parameters=[{"use_sim_time": use_sim_time}],
    remappings=[
      ("/cmd_vel_in", "/cmd_vel_unstamped"),
      ("/cmd_vel_out", "/cmd_vel_stamped"),
    ],
  )

  # MARK

  # MARK: Launch!
  return LaunchDescription(
    [
      DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true",
      ),
      # Nodes
      teleop,
      joy_node,
      teleop_joy,
      twist_mux,
      twist_stamper,
    ]
  )
