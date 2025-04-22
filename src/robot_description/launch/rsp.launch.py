import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

# import xacro

# This is a python launch file that starts a robot_state_publisher node that tells other nodes
# how robot works and what it looks like using  the robot.urdf.xacro file

def generate_launch_description():
    
    target_robot = LaunchConfiguration('robot');
    use_sim_time = LaunchConfiguration('use_sim_time');
    
    this_package = 'robot_description';

    pkg_path = os.path.join(get_package_share_directory(this_package));
    xacro_file = os.path.join(pkg_path,'description','0_robot.urdf.xacro');

    robot_description_config = Command(
        [
            'xacro ', xacro_file, 
            ' target_robot:=', target_robot
            ]
        )

    # create a robot_state_publisher node

    params = {
        'robot_description': robot_description_config, 
        'use_sim_time': use_sim_time
        }
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    # Launch the nodes!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'robot_description',
            default_value='test',
            description='pick which robot to publish'),

        node_robot_state_publisher

    ])