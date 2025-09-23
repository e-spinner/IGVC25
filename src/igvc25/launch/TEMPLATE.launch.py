import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    package_name = 'igvc23'

    # MARK: Args
    temp = LaunchConfiguration('temp')

    temp_arg = DeclareLaunchArgument(
        'temp',
        default_value='temp',
        description='temp'
    )

    # MARK


    # MARK: Launch!
    return LaunchDescription([
        # Args
        temp_arg,

        # Nodes

    ])