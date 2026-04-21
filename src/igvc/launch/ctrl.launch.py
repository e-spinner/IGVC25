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
  igvc_control_package = "igvc_control"

  igvc_path = get_package_share_directory(igvc_package)
  igvc_control_path = get_package_share_directory(igvc_control_package)

  # Get launch arguments
  device = LaunchConfiguration("device").perform(context)
  baud_rate = LaunchConfiguration("baud_rate").perform(context)
  linkage_config_num = LaunchConfiguration("linkage_config").perform(context)
  use_sim_time = (
    LaunchConfiguration("use_sim_time").perform(context).strip().lower()
    in ["true", "1", "yes", "on"]
  )


  # MARK: Robot Description
  # -----------------------------------------------------------------------------
  robot_description = load_robot_description(
    os.path.join(igvc_path, "description", "ackermann_ac.urdf"),
    os.path.join(igvc_path, "config", "ackermann.yaml"),
  )

  # Inject runtime hardware params (device, baud_rate) into ros2_control block
  # so the hardware plugin reads launch-provided values instead of URDF defaults.
  robot_description = re.sub(
    r'(<param name="device">)([^<]+)(</param>)',
    r'\g<1>' + device + r'\3',
    robot_description,
    count=1,
  )
  robot_description = re.sub(
    r'(<param name="baud_rate">)([^<]+)(</param>)',
    r'\g<1>' + str(baud_rate) + r'\3',
    robot_description,
    count=1,
  )

  ros2_control_config_file = os.path.join(
    igvc_control_path, "config", f"ros2_control.yaml"
  )

  # Robot State Publisher
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

  # MARK: Controller Manager
  # -----------------------------------------------------------------------------
  controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
        ros2_control_config_file,
        {"use_sim_time": use_sim_time},
        {"robot_description": robot_description},
    ],
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


def generate_launch_description():
  # MARK: Args
  # -----------------------------------------------------------------------------
  device_arg = DeclareLaunchArgument(
    "device",
    default_value="/dev/ttyACM1",
    description="Serial device path for Arduino",
  )

  baud_rate_arg = DeclareLaunchArgument(
    "baud_rate",
    default_value="115200",
    description="Serial baud rate",
  )

  linkage_config_arg = DeclareLaunchArgument(
    "linkage_config",
    default_value="3",
    description="Linkage configuration number",
  )
  use_sim_time_arg = DeclareLaunchArgument(
    "use_sim_time",
    default_value="false",
    description="Use simulation time if true",
  )

  # MARK: Launch!
  # -----------------------------------------------------------------------------
  return LaunchDescription(
    [
      device_arg,
      baud_rate_arg,
      linkage_config_arg,
      use_sim_time_arg,
      OpaqueFunction(function=launch_setup),
    ]
  )
