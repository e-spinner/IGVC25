import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'igvc23';

    # MARK: rsp
    robot = LaunchConfiguration('robot');

    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='test',
        description='Robot to load'
    );

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )]), launch_arguments={
                'simulation': 'true',
                'robot': robot
            }.items()
    );

    # MARK: Twist
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','teleop.launch.py'
        )])
    );

    #MARK: Gazebo
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.sdf'
    );


    world = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'worlds',
        LaunchConfiguration('world')
    ]);

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    );


    # Include the Gazebo launch file, provided by the ros_gz_sim package
    # https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), launch_arguments={
                'gz_args': ['-r -v4 ', world],
                'on_exit_shutdown': 'true'
            }.items()
    );

    # Run the spawner node from the ros_gz_sim package.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'sarah',
            '-z', '0.3'
        ],
        output='screen'
    );


    # MARK: Control
    # https://github.com/ros-controls/ros2_control
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # MARK: Bridge
    # https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md
    bridge_params = os.path.join(
        get_package_share_directory(package_name),'config','gz_bridge.yaml'
    );

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
    );

    # https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_image
    left_ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/left_camera/image_raw"],
        name="left_image_bridge_node",
    );

    right_ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/right_camera/image_raw"],
        name="right_image_bridge_node",
    );

    # MARK: nav2
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','navigation.launch.py'
        )]), launch_arguments={
            'use_sim_time': 'true',
        }.items()
    );

    # MARK: localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),
            'launch',
            'localization.launch.py'
        )]), launch_arguments={
            'use_time_time': 'true'
        }.items()
    );

    # MARK: visualization
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),'config','sim.rviz'
    );

    visualization = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )


    # MARK: Launch!
    return LaunchDescription([
        # args
        robot_arg,
        world_arg,

        visualization,

        rsp,
        teleop,

        # sim
        gazebo,
        spawn_entity,

        # Control
        diff_drive_spawner,
        joint_broad_spawner,

        # bridge
        ros_gz_bridge,
        # right_ros_gz_image_bridge,
        # left_ros_gz_image_bridge,

        # nav2
        navigation,

        # localization
        localization,

    ]);