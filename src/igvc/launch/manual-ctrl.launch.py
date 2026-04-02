import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  package_name = "igvc"
  package_path = get_package_share_directory(package_name)

  # MARK: Args
  device = LaunchConfiguration("device")
  baud_rate = LaunchConfiguration("baud_rate")
  linkage_config = LaunchConfiguration("linkage_config")
  use_sim_time = LaunchConfiguration("use_sim_time")

  device_arg = DeclareLaunchArgument(
    "device",
    default_value="/dev/ttyACM0",
    description="Serial device path for Arduino",
  )
  baud_rate_arg = DeclareLaunchArgument(
    "baud_rate",
    default_value="115200",
    description="Serial baud rate for Arduino",
  )
  linkage_config_arg = DeclareLaunchArgument(
    "linkage_config",
    default_value="3",
    description="Ackermann linkage configuration number",
  )
  use_sim_time_arg = DeclareLaunchArgument(
    "use_sim_time",
    default_value="false",
    description="Use simulation time if true",
  )

  # MARK: Includes
  ctrl_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(package_path, "launch", "ctrl.launch.py")
    ),
    launch_arguments={
      "device": device,
      "baud_rate": baud_rate,
      "linkage_config": linkage_config,
      "use_sim_time": use_sim_time,
    }.items(),
  )

  teleop_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(package_path, "launch", "teleop.launch.py")
    ),
    launch_arguments={
      "use_sim_time": use_sim_time,
    }.items(),
  )

  # MARK: Launch!
  return LaunchDescription(
    [
      device_arg,
      baud_rate_arg,
      linkage_config_arg,
      use_sim_time_arg,
      ctrl_launch,
      teleop_launch,
    ]
  )
