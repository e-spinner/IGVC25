import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'igvc25';
    
    
    # MARK: pc 2 ls
    # https://github.com/ros-perception/pointcloud_to_laserscan
    cloud2scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
        }],

    );
    
    slam_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'mapper_params_online_async.yaml'
    );
    
    # MARK: Launch!
    return LaunchDescription([
        
        # Nodes
        cloud2scan,
        
    ]);