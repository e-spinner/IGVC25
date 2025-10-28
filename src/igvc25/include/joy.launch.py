import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
  package_name = "igvc25"

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
      # Args
      # Nodes
      teleop_joy,
      joy_node,
    ]
  )
