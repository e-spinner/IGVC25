import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  OpaqueFunction,
  RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
  igvc_package = "igvc"
  igvc_control_package = "igvc_control"

  igvc_path = get_package_share_directory(igvc_package)
  igvc_control_path = get_package_share_directory(igvc_control_package)

  # Get launch arguments
  device = LaunchConfiguration("device").perform(context)
  baud_rate = LaunchConfiguration("baud_rate").perform(context)
  linkage_config_num = LaunchConfiguration("linkage_config").perform(context)

  # MARK: Robot Description
  # -----------------------------------------------------------------------------
  robot_description_path = os.path.join(
    igvc_path, "description", "ackermann_ac.urdf"
  )

  with open(robot_description_path, "r") as f:
    robot_description = f.read()

  # Load linkage parameters for controller
  linkage_config_file = os.path.join(
    igvc_control_path, "config", f"ackermann_linkage_{linkage_config_num}.yaml"
  )

  with open(linkage_config_file, "r") as f:
    linkage_config = yaml.safe_load(f)

  linkage_params = linkage_config["/**"]["ros__parameters"]

  # Robot State Publisher
  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[
      {
        "robot_description": robot_description,
        "use_sim_time": False,
      }
    ],
  )

  # MARK: Controller Manager
  # -----------------------------------------------------------------------------
  controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
      {
        "robot_description": robot_description,
        "use_sim_time": False,
      }
    ],
    output="screen",
  )

  # MARK: Joint State Broadcaster
  # -----------------------------------------------------------------------------
  joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"],
    output="screen",
  )

  # MARK: Ackermann Angle Controller
  # -----------------------------------------------------------------------------
  # Controller parameters from linkage config
  controller_params = {
    "ackermann_angle_controller": {
      "ros__parameters": {
        "pinion_radius": linkage_params.get("pinion_radius", 0.01905),
        "steering_arm_length": linkage_params.get("steering_arm_length", 0.0381),
        "tie_rod_length": linkage_params.get("tie_rod_length", 0.127),
        "rack_offset_x": linkage_params.get("rack_offset_x", -0.0315),
        "rack_neutral_y": linkage_params.get("rack_neutral_y", 0.131064),
        "pinion_gear_ratio": linkage_params.get("pinion_gear_ratio", 1.652),
        "max_pinion_angle": linkage_params.get("pinion_angle_limit", 2.0),
        "wheel_angle": linkage_params.get("wheel_angle", 0.32253),
        "wheelbase": linkage_params.get("wheel_base", 0.42),
        "track_width": linkage_params.get("track_width", 0.36),
        "wheel_radius": linkage_params.get("wheel_radius", 0.1524),
        "pinion_joint": "pinion_joint",
        "rear_motor_joint": "rear_motor_joint",
        "cmd_vel_topic": "/cmd_vel_stamped",
        "reference_timeout": 2.0,
        "calibration_sample_size": 1024,
      }
    }
  }

  ackermann_angle_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
      "ackermann_angle_controller",
      "--controller-manager",
      "/controller_manager",
      "--controller-manager-timeout",
      "10",
    ],
    parameters=[controller_params],
    output="screen",
  )

  return [
    robot_state_publisher,
    controller_manager,
    RegisterEventHandler(
      event_handler=OnProcessExit(
        target_action=controller_manager,
        on_exit=[joint_state_broadcaster_spawner],
      )
    ),
    RegisterEventHandler(
      event_handler=OnProcessExit(
        target_action=joint_state_broadcaster_spawner,
        on_exit=[ackermann_angle_controller_spawner],
      )
    ),
  ]


def generate_launch_description():
  # MARK: Args
  # -----------------------------------------------------------------------------
  device_arg = DeclareLaunchArgument(
    "device",
    default_value="/dev/ttyUSB0",
    description="Serial device path for Arduino",
  )

  baud_rate_arg = DeclareLaunchArgument(
    "baud_rate",
    default_value="9600",
    description="Serial baud rate",
  )

  linkage_config_arg = DeclareLaunchArgument(
    "linkage_config",
    default_value="2",
    description="Linkage configuration number",
  )

  # MARK: Launch!
  # -----------------------------------------------------------------------------
  return LaunchDescription(
    [
      device_arg,
      baud_rate_arg,
      linkage_config_arg,
      OpaqueFunction(function=launch_setup),
    ]
  )
