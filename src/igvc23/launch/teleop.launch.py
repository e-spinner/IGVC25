import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'igvc23';

    # MARK: Args
    use_sim_time = LaunchConfiguration('use_sim_time')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    );

    # MARK: Nodes
    # https://github.com/ros-teleop/teleop_twist_keyboard

    teleop_key = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        remappings=[('/cmd_vel', 'cmd_vel_teleop_key')],
        prefix='xterm -e'  # Opens in a new terminal window
    );


    # https://github.com/ros-teleop/teleop_twist_joy
    joy_params = os.path.join(get_package_share_directory(package_name),'config','joy_params.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
    );

    teleop_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[('/cmd_vel','cmd_vel_teleop_joy')]
    );


    # https://github.com/ros-teleop/twist_mux
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),'config','twist_mux.yaml'
    );

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out','cmd_vel_unstamped')]
    );


    # https://github.com/joshnewans/twist_stamper
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_in','cmd_vel_unstamped'),
                    ('/cmd_vel_out','/diff_drive_controller/cmd_vel')]
    );

    # MARK: Launch!
    return LaunchDescription([
        # Args
        sim_time_arg,

        # Nodes
        teleop_key,
        teleop_joy,
        joy_node,
        twist_mux,
        twist_stamper
    ]);