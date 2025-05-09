# IGVC25
ONU Capstone IGVC 2025-26


## Building The Workspace

>  This command builds the ws, you must cd into the top level of the ws to run it. if you have multiple packages it defaults to building upto 16 at once, and if your machine can't handle that add --executor-sequential to the end.

```bash
colcon build --symlink-install
```

 Now that you have built your workspace you have to tell linux where to find your programs. you must do this evey time you open a brand new terminal to run a process from this workspace.

```bash
source install/setup.bash
```

### Simulation Launch File

 After building and sourcing the workspace, the following command launches the sim.

```bash
ros2 launch igvc23 sim.launch.py robot:=fwd world:=cubes.sdf
```

### Robot Launch File

```bash
ros2 launch igvc23 
```

---

## Dependencies

```bash
sudo apt install ros-dev-tools ros-jazzy-desktop ros-jazzy-xacro gz-harmonic ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-twist-mux ros-jazzy-twist-stamper ros-jazzy-ros2-control ros-jazzy-ros2-controllers xterm ros-jazzy-velodyne ros-jazzy-imu-tools ros-jazzy-pointcloud-to-laserscan ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup 
```

## Bugs

- [#1] sim.launch.py only works in ubuntu terminal, not vscode terminal
    > Honestly who cares

- [#2] rviz2 and gz cannot 'find'? mesh files
    > Only matters if want sim / vizualization to look cooler

- [#3] ~~slam toolbox requires there to be walls around robot to give big enough map for nav2~~
    > nav2 costmaps were the actual problem, allowing for inf reads in the scan data to be treated as a far away obstacle, and empty space until that obsticle fixes this issue

- [#4] ekf localization odoom and map trasform broken
    > while not a solution, just using slam_toolbox for localization instead of sensor fusion for now.

- [#5] ~~nav2 randomly stops navigating~~
    > now that base_link and base_frame are on the same level, and everything is treating navigation and localization as a 2D problem, I have not noticed this issue, still need to make custom bt to allow for path smoothing, waypoint following, and hopefuly fix [#6]

- [#6] ~~nav2 drives very slow~~
    > avoiding issue by switching from mppi to dwb controller.

## Links

[velodyne package](https://github.com/ros-drivers/velodyne/tree/2.5.1)
[possibly helpful for sensors](https://docs.clearpathrobotics.com/docs/ros/config/yaml/sensors/lidar3d/)
[video for gazebo classic migration](https://youtu.be/fH4gkIFZ6W8?feature=shared)

*Possible info for ramp traversal*: [nav2-issue](https://github.com/ros-navigation/navigation2/issues/1224)

[imu](https://www.phidgets.com/?prodid=1205&srsltid=AfmBOopsWk3J6HLVH19Cp32uwF5x0yiZpEFWTLkfj78CUGBrVDpSwTwm)

[core nav logic](https://docs.nav2.org/concepts/index.html#behavior-trees)

[bt introduction](https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html)