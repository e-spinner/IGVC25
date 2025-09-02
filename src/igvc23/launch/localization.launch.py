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

    localization_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'localization.yaml'
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

    # MARK: Slam
    # simultaneous localization and mapping
    # respinsivle for creating static obstacle map

    # https://github.com/SteveMacenski/slam_toolbox

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params file': localization_params
        }.items()
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


    # Different options for Robot Localization:
    #
    # - [1] slam_toolbox will localize based on consistant walls in lidar scan data
    #
    # - [2] can use ekf filters to localize based on gps, odom, and imu data
    #
    # - [3] can use amcl to localize based on a given map and scan data


    # MARK: Launch!
    return LaunchDescription([
        # Args
        sim_time_arg,

        # Nodes
        slam_toolbox,


        # # odom_localiztion,
        # map_localiztion,
        # navsat_transform,

    ]);