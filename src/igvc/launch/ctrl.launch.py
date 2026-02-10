import os
import xacro
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


def load_robot_description(robot_description_path, robot_params_path):
  with open(robot_params_path, "r") as params:
    robot_params = yaml.safe_load(params)["/**"]["ros__parameters"]

  robot_description = xacro.process_file(
    robot_description_path,
    mappings={key: str(value) for key, value in robot_params.items()},
  )

  return robot_description.toxml()  # type: ignore


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
  robot_description = load_robot_description(
    os.path.join(igvc_path, "description", "ackermann_ac.urdf"),
    os.path.join(igvc_path, "config", "ackermann.yaml"),
  )

  # Load ros2_control config
  ros2_control_config_file = os.path.join(
    igvc_control_path, "config", "ros2_control.yaml"
  )

  with open(ros2_control_config_file, "r") as f:
    ros2_control_config = yaml.safe_load(f)

  # Load linkage parameters for controller
  linkage_config_file = os.path.join(
    igvc_control_path, "config", f"ackermann_linkage_{linkage_config_num}.yaml"
  )

  with open(linkage_config_file, "r") as f:
    linkage_config = yaml.safe_load(f)

  linkage_params = linkage_config["/**"]["ros__parameters"]

  # Merge linkage parameters into controller config
  controller_params = ros2_control_config.get("ackermann_angle_controller", {}).get(
    "ros__parameters", {}
  )
  controller_params.update(
    {
      "pinion_radius": linkage_params.get(
        "pinion_radius", controller_params.get("pinion_radius", 0.01905)
      ),
      "steering_arm_length": linkage_params.get(
        "steering_arm_length", controller_params.get("steering_arm_length", 0.0381)
      ),
      "tie_rod_length": linkage_params.get(
        "tie_rod_length", controller_params.get("tie_rod_length", 0.127)
      ),
      "rack_offset_x": linkage_params.get(
        "rack_offset_x", controller_params.get("rack_offset_x", -0.0315)
      ),
      "rack_neutral_y": linkage_params.get(
        "rack_neutral_y", controller_params.get("rack_neutral_y", 0.131064)
      ),
      "pinion_gear_ratio": linkage_params.get(
        "pinion_gear_ratio", controller_params.get("pinion_gear_ratio", 1.652)
      ),
      "max_pinion_angle": linkage_params.get(
        "pinion_angle_limit", controller_params.get("max_pinion_angle", 2.0)
      ),
      "wheel_angle": linkage_params.get(
        "wheel_angle", controller_params.get("wheel_angle", 0.32253)
      ),
      "wheelbase": linkage_params.get(
        "wheel_base", controller_params.get("wheelbase", 0.42)
      ),
      "track_width": linkage_params.get(
        "track_width", controller_params.get("track_width", 0.36)
      ),
      "wheel_radius": linkage_params.get(
        "wheel_radius", controller_params.get("wheel_radius", 0.1524)
      ),
    }
  )

  # Update the config with merged parameters
  if "ackermann_angle_controller" not in ros2_control_config:
    ros2_control_config["ackermann_angle_controller"] = {}
  ros2_control_config["ackermann_angle_controller"]["ros__parameters"] = (
    controller_params
  )

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
  # Extract controller_manager parameters (without ros__parameters wrapper for programmatic passing)
  cm_params = ros2_control_config.get("controller_manager", {}).get(
    "ros__parameters", {}
  )

  # Ensure controller type definitions are present
  if "joint_state_broadcaster" not in cm_params:
    cm_params["joint_state_broadcaster"] = {
      "type": "joint_state_broadcaster/JointStateBroadcaster"
    }
  if "ackermann_angle_controller" not in cm_params:
    cm_params["ackermann_angle_controller"] = {
      "type": "igvc_control::AckermannAngleController"
    }

  controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
      cm_params,
      {
        "use_sim_time": False,
      },
    ],
    output="screen",
    remappings=[("/controller_manager/robot_description", "/robot_description")],
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
  ackermann_angle_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
      "ackermann_angle_controller",
    ],
    parameters=[
      {
        "ackermann_angle_controller": ros2_control_config[
          "ackermann_angle_controller"
        ]
      }
    ],
    output="screen",
  )

  return [
    robot_state_publisher,
    controller_manager,
    joint_state_broadcaster_spawner,
    ackermann_angle_controller_spawner,
  ]


def generate_launch_description():
  # MARK: Args
  # -----------------------------------------------------------------------------
  device_arg = DeclareLaunchArgument(
    "device",
    default_value="/dev/ttyACM0",
    description="Serial device path for Arduino",
  )

  baud_rate_arg = DeclareLaunchArgument(
    "baud_rate",
    default_value="115200",
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
