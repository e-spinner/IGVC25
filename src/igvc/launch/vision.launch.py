import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import Node

OUTPUT = "screen"  # 'log', 'both'
NAMESPACE = "vision"

GPS_USBID = "04d8:00dd"


def device_connected(device_id):
  try:
    output = subprocess.check_output(["lsusb"], text=True)
    return device_id in output
  except Exception:
    return False


def generate_launch_description():
  package_name = "igvc"
  igvc_share = get_package_share_directory(package_name)
  velodyne_calibration = os.path.join(
    get_package_share_directory("velodyne_pointcloud"),
    "params",
    "VLP16db.yaml",
  )

  phidgets_imu_front_yaml = os.path.join(igvc_share, "config", "phidgets_imu_front.yaml")
  phidgets_imu_back_yaml = os.path.join(igvc_share, "config", "phidgets_imu_back.yaml")

  # MARK: GPS
  gps_driver = Node(
    package="nmea_navsat_driver",
    executable="nmea_serial_driver",
    output=OUTPUT,
    name="garmin_18x",
    namespace=NAMESPACE,
    parameters=[{"port": "/dev/gps", "baud": 4800}],
    condition=IfCondition(
      PythonExpression([str(device_connected(GPS_USBID))])
    ),
  )

  # MARK: IMU — two Phidgets Spatial nodes (matches ackermann_ac.urdf imu_front / imu_back).
  # Topics: /vision/imu_front/imu/data_raw, /vision/imu_back/imu/data_raw
  # ComposableNode does NOT inherit the container namespace; without an explicit namespace here,
  # drivers default to "/" and publish /imu/data_raw (broken with two IMUs).
  # Set distinct serial: in config/phidgets_imu_front.yaml and phidgets_imu_back.yaml
  imu_drivers = ComposableNodeContainer(
    name="phidget_container",
    namespace=NAMESPACE,
    package="rclcpp_components",
    executable="component_container",
    composable_node_descriptions=[
      ComposableNode(
        package="phidgets_spatial",
        plugin="phidgets::SpatialRosI",
        name="imu_front",
        namespace=f"{NAMESPACE}/front",
        parameters=[phidgets_imu_front_yaml],
      ),
      ComposableNode(
        package="phidgets_spatial",
        plugin="phidgets::SpatialRosI",
        name="imu_back",
        namespace=f"{NAMESPACE}/back",
        parameters=[phidgets_imu_back_yaml],
      ),
    ],
    output=OUTPUT,
    # Do not gate on lsusb: Phidgets often enumerate late or drop off the bus; a false
    # skip meant zero IMU nodes. Use launch_imu:=false on machines without hardware.
    condition=IfCondition(LaunchConfiguration("launch_imu")),
  )

  # MARK: LiDAR
  velodyne_driver = Node(
    package="velodyne_driver",
    executable="velodyne_driver_node",
    output=OUTPUT,
    name="velodyne_driver",
    namespace=NAMESPACE,
    parameters=[
      {
        "model": "VLP16",
        # "device_ip": "192.168.1.201",
        "port": 2368,
        "frame_id": "velodyne",
        "rpm": 600.0,
      }
    ],

  )

  velodyne_pointcloud = Node(
    package="velodyne_pointcloud",
    executable="velodyne_transform_node",
    output=OUTPUT,
    name="velodyne_transform",
    namespace=NAMESPACE,
    parameters=[
      {
        "calibration": velodyne_calibration,
        "model": "VLP16",
        "fixed_frame": "velodyne",
        "min_range": 0.9,
        "max_range": 130.0,
      }
    ],
  )

  # MARK: Launch!
  return LaunchDescription(
    [
      DeclareLaunchArgument(
        "launch_imu",
        default_value="true",
        description="Start Phidgets IMU nodes. Set false if no IMUs (avoids startup errors).",
      ),
      gps_driver,
      imu_drivers,
      # velodyne_driver,
      # velodyne_pointcloud,
    ]
  )
