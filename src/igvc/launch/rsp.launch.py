import os
import re
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
  igvc_path = get_package_share_directory("igvc")

  device = LaunchConfiguration("device").perform(context)
  baud_rate = LaunchConfiguration("baud_rate").perform(context)
  use_sim_time = (
    LaunchConfiguration("use_sim_time").perform(context).strip().lower()
    in ["true", "1", "yes", "on"]
  )

  robot_description = load_robot_description(
    os.path.join(igvc_path, "description", "ackermann_ac.urdf"),
    os.path.join(igvc_path, "config", "ackermann.yaml"),
  )

  # Keep runtime hardware params in sync with ctrl.launch.py values.
  robot_description = re.sub(
    r'(<param name="device">)([^<]+)(</param>)',
    r"\g<1>" + device + r"\3",
    robot_description,
    count=1,
  )
  robot_description = re.sub(
    r'(<param name="baud_rate">)([^<]+)(</param>)',
    r"\g<1>" + str(baud_rate) + r"\3",
    robot_description,
    count=1,
  )

  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[
      {
        "robot_description": robot_description,
        "use_sim_time": use_sim_time,
      }
    ],
  )

  return [robot_state_publisher]


def generate_launch_description():
  return LaunchDescription(
    [
      DeclareLaunchArgument(
        "device",
        default_value="/dev/ttyACM0",
        description="Serial device path for Arduino",
      ),
      DeclareLaunchArgument(
        "baud_rate",
        default_value="115200",
        description="Serial baud rate",
      ),
      DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true",
      ),
      OpaqueFunction(function=launch_setup),
    ]
  )
