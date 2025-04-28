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
    
    localization_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'localization.yaml'
    );
    
    # MARK: odom lztn
    odom_localiztion = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[localization_params, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "odometry/local")],
    );
    
    # MARK: map  lztn
    map_localiztion = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[localization_params, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "odometry/global")],
    );
    
    # MARK: Nvst Trsm
    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[localization_params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("imu/data", "imu/data"),
            ("gps/fix", "gps/fix"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/global"),
        ],
    );
    
    
    # MARK: Launch!
    return LaunchDescription([
        # Args
        sim_time_arg,
        
        # Nodes
        odom_localiztion,
        map_localiztion,
        navsat_transform,
        
    ]);