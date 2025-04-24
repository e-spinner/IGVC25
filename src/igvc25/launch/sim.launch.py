import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'igvc25';
    
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
        'empty.world'
    );
 
    world = LaunchConfiguration('world');
 
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    );
    
    # Include the Gazebo launch file, provided by the ros_gz_sim package
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
            '-z', '0.1'
        ],
        output='screen'
    );
    
    
    # MARK: Control
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
    
    # MARK: Launch!
    return LaunchDescription([
        # args
        robot_arg,
        world_arg,
        
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
        right_ros_gz_image_bridge,
        left_ros_gz_image_bridge,
    
    ]);