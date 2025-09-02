import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

# import xacro

# This is a python launch file that starts a robot_state_publisher node that tells other nodes
# how robot works and what it looks like using  the robot.urdf.xacro file

def generate_launch_description():

    target_robot = LaunchConfiguration('robot');
    simulation = LaunchConfiguration('simulation');

    package_name = 'igvc23';

    pkg_path = os.path.join(get_package_share_directory(package_name));
    xacro_file = os.path.join(pkg_path,'description','0_robot.urdf.xacro');

    robot_description_config = Command(
        [
            'xacro ', xacro_file,
            ' target_robot:=', target_robot,
            ' simulation:=', simulation
            ]
        );

    # https://github.com/ros/robot_state_publisher
    # create a robot_state_publisher node

    params = {
        'robot_description': ParameterValue(robot_description_config, value_type=str),
        'use_sim_time': simulation
        }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    );

    # Launch the nodes!
    return LaunchDescription([

        DeclareLaunchArgument(
            'simulation',
            default_value='true',
            description='treated as a simulation if true'),
        DeclareLaunchArgument(
            'robot',
            default_value='test',
            description='pick which robot to publish'),

        node_robot_state_publisher

    ]);