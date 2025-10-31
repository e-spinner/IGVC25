import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = "igvc"
    pkg_share = get_package_share_directory(pkg)

    # include the existing simulation launch (spawns robot, gz, bridge, controllers)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "sim.launch.py"))
    )

    # path to the lane_following script in the source tree (works for local dev)
    script_path = os.path.abspath(
        os.path.join(pkg_share, "..", "src", "igvc", "igvc_vision", "lane_following.py")
    )

    lane_follow_proc = ExecuteProcess(
        cmd=["python3", script_path],
        output="screen",
    )

    return LaunchDescription([sim_launch, lane_follow_proc])
