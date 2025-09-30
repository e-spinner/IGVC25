import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import Node, PushRosNamespace

OUTPUT = 'screen' # 'log', 'both'
NAMESPACE = 'vision'


def device_connected(device_id):
    try:
        output = subprocess.check_output(["lsusb"], text=True)
        return device_id in output
    except Exception:
        return False

def generate_launch_description():

    package_name = 'igvc25'

    # MARK: Args
    temp = LaunchConfiguration('temp')

    temp_arg = DeclareLaunchArgument(
        'temp',
        default_value='temp',
        description='temp'
    )

    # MARK: GPS
    gps_driver = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output = OUTPUT,
        name='garmin_18x',
        namespace=NAMESPACE,
        parameters=[{
            'port': '/dev/gps',
            'baud': 4800
        }],
        condition=IfCondition(PythonExpression([str(device_connected("04d8:00dd"))]))
    )

    # MARK: IMU
    imu_params = {
        # optional param use_orientation, default is false
        'use_orientation': True,

        # optional param spatial_algorithm, default is 'ahrs'
        'spatial_algorithm': 'ahrs',

        # optional ahrs parameters
        'ahrs_angular_velocity_threshold': 1.0,
        'ahrs_angular_velocity_delta_threshold': 0.1,
        'ahrs_acceleration_threshold': 0.05,
        'ahrs_mag_time': 10.0,
        'ahrs_accel_time': 10.0,
        'ahrs_bias_time': 1.25,

        # optional param heating_enabled, not modified by default
        'heating_enabled': False,
    }



    imu_driver = ComposableNodeContainer(
            name='phidget_container',
            namespace=NAMESPACE,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='phidgets_spatial',
                    plugin='phidgets::SpatialRosI',
                    name='phidgets_spatial',
                    namespace=NAMESPACE,
                    parameters=[imu_params]),
            ],
            output=OUTPUT,
            condition=IfCondition(PythonExpression([str(device_connected("06c2:008d"))]))
    )



    # MARK: Launch!
    return LaunchDescription([
        # Args
        temp_arg,

        # Nodes
        gps_driver,
        imu_driver,

    ])