from launch import LaunchDescription
from launch_ros.actions import Node

import os 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        remappings=[('/cmd_vel', 'cmd_vel_teleop')],
        prefix='xterm -e'  # Opens in a new terminal window
    );
    
    
    # Launch them all!
    return LaunchDescription([
        teleop_node
    ]);