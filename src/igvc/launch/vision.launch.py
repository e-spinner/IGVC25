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
  params = os.path.join(get_package_share_directory("igvc"), "config", "vision.yaml")
  lidar_condition = IfCondition(LaunchConfiguration("lidar"))

  velodyne_calibration = os.path.join(
    get_package_share_directory("velodyne_pointcloud"),
    "params",
    "VLP16db.yaml",
  )

  # MARK: GPS
  gps_driver = Node(
    package="nmea_navsat_driver",
    executable="nmea_serial_driver",
    output=OUTPUT,
    name="garmin_18x",
    namespace=NAMESPACE,
    parameters=[params],
    condition=IfCondition(PythonExpression([str(device_connected(GPS_USBID))])),
  )

  # MARK: IMUS
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
        parameters=[params],
      ),
      ComposableNode(
        package="phidgets_spatial",
        plugin="phidgets::SpatialRosI",
        name="imu_back",
        namespace=f"{NAMESPACE}/back",
        parameters=[params],
      ),
    ],
    output=OUTPUT,
    condition=IfCondition(LaunchConfiguration("imu")),
  )

  # MARK: LiDAR
  velodyne_driver = Node(
    package="velodyne_driver",
    executable="velodyne_driver_node",
    output=OUTPUT,
    name="velodyne_driver",
    namespace=NAMESPACE,
    parameters=[params],
    condition=lidar_condition,
  )

  velodyne_pointcloud = Node(
    package="velodyne_pointcloud",
    executable="velodyne_transform_node",
    output=OUTPUT,
    name="velodyne_transform",
    namespace=NAMESPACE,
    parameters=[params, {"calibration": velodyne_calibration}],
    condition=lidar_condition,
  )

  # MARK: Launch!
  return LaunchDescription(
    [
      DeclareLaunchArgument(
        "imu", default_value="true", description="Start IMU nodes."
      ),
      DeclareLaunchArgument(
        "lidar", default_value="true", description="Start Lidar nodes."
      ),
      gps_driver,
      imu_drivers,
      velodyne_driver,
      velodyne_pointcloud,
    ]
  )
