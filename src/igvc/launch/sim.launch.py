import os

import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  GroupAction,
  IncludeLaunchDescription,
  OpaqueFunction,
  RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node


# MARK: URDF
def load_robot_description(robot_description_path, robot_params_path):
  with open(robot_params_path, "r") as params:
    robot_params = yaml.safe_load(params)["/**"]["ros__parameters"]

  robot_description = xacro.process_file(
    robot_description_path,
    mappings={key: str(value) for key, value in robot_params.items()},
  )

  return robot_description.toxml()  # type: ignore


# MARK: Sim
def sim_ros_nodes(context, *args, **kwargs):
  package_name = "igvc"
  package_path = get_package_share_directory(package_name)
  gz_world = LaunchConfiguration("gz_world_name").perform(context)

  # MARK: Desc
  robot_description = load_robot_description(
    os.path.join(package_path, "description", "ackermann_gz.urdf"),
    os.path.join(package_path, "config", "ackermann.yaml"),
  )

  # MARK: Gz Spawn
  spawn_robot = Node(
    package="ros_gz_sim",
    executable="create",
    output="screen",
    arguments=[
      "-world",
      gz_world,
      "-name",
      "igvc_sim",
      "-string",
      robot_description,
      "-x",
      "0",
      "-y",
      "0",
      "-z",
      "0.35",
    ],
  )

  # MARK: RSP
  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[
      {
        "robot_description": robot_description,
        "use_sim_time": True,
      }
    ],
  )

  # MARK: JSB spwn
  joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    output="screen",
    arguments=[
      "joint_state_broadcaster",
      "-c",
      "/controller_manager",
    ],
  )

  # MARK: Ack spwn
  controller_params = os.path.join(package_path, "config", "gz_ros2_control.yaml")
  ackermann_steering_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    output="screen",
    arguments=[
      "ackermann_steering_controller",
      "-c",
      "/controller_manager",
      "-p",
      controller_params,
    ],
  )

  # MARK: Odom tf
  odom_to_tf = Node(
    package="igvc_odom",
    executable="odom_to_tf",
    output="screen",
    parameters=[{"odom_topic": "/odom"}],
  )

  # MARK: Twist mux
  twist_mux_params = os.path.join(package_path, "config", "twist_mux.yaml")
  twist_mux = Node(
    package="twist_mux",
    executable="twist_mux",
    parameters=[twist_mux_params, {"use_sim_time": True}],
    remappings=[("cmd_vel_out", "cmd_vel_unstamped")],
  )

  # MARK: Twist stmp
  twist_stamper = Node(
    package="twist_stamper",
    executable="twist_stamper",
    parameters=[{"use_sim_time": True}],
    remappings=[
      ("cmd_vel_in", "cmd_vel_unstamped"),
      ("cmd_vel_out", "cmd_vel_stamped"),
    ],
  )

  # MARK: Teleop
  teleop = Node(
    package="teleop_twist_keyboard",
    executable="teleop_twist_keyboard",
    name="teleop_keyboard",
    output="screen",
    remappings=[("cmd_vel", "cmd_vel_teleop_key")],
    prefix="xterm -e",
    condition=UnlessCondition(LaunchConfiguration("headless")),
  )

  # MARK: Nav2
  nav_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(package_path, "launch", "nav.launch.py")),
    launch_arguments={"use_sim_time": "true"}.items(),
    condition=IfCondition(LaunchConfiguration("nav2")),
  )

  # MARK: Grp + ev
  grouped = GroupAction(
    actions=[
      robot_state_publisher,
      RegisterEventHandler(
        event_handler=OnProcessExit(
          target_action=spawn_robot,
          on_exit=[joint_state_broadcaster_spawner],
        )
      ),
      RegisterEventHandler(
        event_handler=OnProcessExit(
          target_action=joint_state_broadcaster_spawner,
          on_exit=[ackermann_steering_controller_spawner],
        )
      ),
      spawn_robot,
      odom_to_tf,
      twist_mux,
      twist_stamper,
      teleop,
      nav_launch,
    ]
  )

  return [grouped]


# MARK: Launch!
def generate_launch_description():
  package_path = get_package_share_directory("igvc")
  world = os.path.join(package_path, "worlds", "plane.sdf")
  bridge_yaml = os.path.join(package_path, "config", "ros_gz_bridge.yaml")

  # MARK: Gazebo
  gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
      )
    ),
    launch_arguments={
      "gz_args": f"-r -v 3 {world}",
      "on_exit_shutdown": "true",
    }.items(),
  )

  # MARK: GZ brdg
  gz_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    output="screen",
    arguments=["--ros-args", "-p", f"config_file:={bridge_yaml}"],
  )

  # MARK: Lztn
  filter_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(package_path, "launch", "pos.launch.py")),
    condition=IfCondition(LaunchConfiguration("filter")),
  )

  # MARK: Args
  return LaunchDescription(
    [
      DeclareLaunchArgument(
        "filter",
        default_value="false",
        description="If true, include hall/GPS localization stack (not used for default sim)",
      ),
      DeclareLaunchArgument(
        "gz_world_name",
        default_value="plane",
        description="SDF world name (must match <world name=...> in worlds/plane.sdf)",
      ),
      DeclareLaunchArgument(
        "nav2",
        default_value="false",
        description="If true, include nav.launch.py (uses navigation.yaml unchanged; enable when ready)",
      ),
      DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="If true, skip xterm teleop keyboard",
      ),
      gazebo_launch,
      gz_bridge,
      OpaqueFunction(function=sim_ros_nodes),
      filter_launch,
    ]
  )
