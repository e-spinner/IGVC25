import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'igvc25';
    
    
    # MARK: Args
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    );
    
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
        remappings=[
            ('/scan', '/scan_be'),
            ('/cloud_in', '/cloud_in')
        ]
    
    );
    
    scan_repub = Node(
            package=f'{package_name}_scripts',
            executable='republish_scan',
            name='scan_qos_republisher',
    );
    
    # https://github.com/SteveMacenski/slam_toolbox
    slam_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'mapper_params_online_async.yaml'
    );
    
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params file': slam_params_file
        }.items()
    );
    
    # https://docs.nav2.org/index.html
    if use_sim_time:
        nav2_params = os.path.join(
            get_package_share_directory(package_name),
            'config',
            'sim_nav2_params.yaml'
        );
    else:
        nav2_params = None
    
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )]),
        launch_arguments={
            'params_file': nav2_params
        }.items()
    );
    
    
    # MARK: Launch!
    return LaunchDescription([
        # Args
        sim_time_arg,
        
        # Nodes
        cloud2scan,
        scan_repub,
        slam_toolbox,
        navigation,
    ]);