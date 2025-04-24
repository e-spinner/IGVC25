## 04-24-2025 -
#### 10:00 AM -
> starting to look into using nav2

## 04-23-2025 - 4.75 hrs
#### 02:45 PM - 04:00: setting up ros-gz bridge
> ~~fixed lidar meshes, meshes should work from now on with gazebo + rviz2~~
>
> I lied, it works with gazebo with a hardcoded direct path, but nto rviz2 still
>
> Going to switch to just a cylinder, no meshes for now
>
> Set up passing of Velodyne sim points
#### 06:30 PM - 10:00 PM: fixing everything so ros-gz-bridge behaves
> lidar cloud succesfully gets to ros from sim
>
> twist cmd_vel info succesfully gets to sim from ros
>
> ![alt text](/files/assets/ros_gz_bridge-finally-works.png)

## 04-22-2025 - 8.5 hrs
#### 10:45 AM - 01:00 PM: Setup robot_description
> made basic urdf file to allow dynamic loading of different robots
>
> created launch file that makes a basic rsp node

#### 02:30 PM - 04:00PM: basic robot 1
> made simple adjustable 2-wheel + caster robot urdf
>
> want to make whole project one package called igvc25

#### 04:00 PM - 07:00 PM: setting up gazebo
> Working on getting the simulation set up,
> installed gazebo [Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/)
>
> Started working on sim.launch.py, and porting igvc23 code, and fixing it

> *sim.launch.py only works in ubbuntu terminal, not vscode terminal*

#### 09:00 PM - 10:45 PM: Simulation of Lidar
> got tf portion of vlp16 lidar set up, need to add simlated laser to gazebo harmonic

> Lidar meshes do not work

## 04-21-2025 - 2.75 hrs
#### 02:15 PM - 03:30 PM: Setup Ros2 Environment
> followed this tutorial to install ros2 [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html).
> 
> ran the basic testing talker / listener example in above link
> 
> Found and installed node visualization software: [ros2_graph](https://github.com/kiwicampus/ros2_graph)

#### 03:30 PM - 04:00 PM: Made this repo

#### 05:30 PM - 06:05 PM: Setup Repo + Ported from old team code
> Wrote basic readme that explains how to build a ros2 application
>
> Ported over and tested code from IGVC23 repo, went poorly:
> ![alt text](/files/assets/04-21-25-failed_build.png)
>
> Old code currently gitignored

#### 06:05 PM - 06:30 PM: Expieramented with URDF files
> found [vscode extension](https://github.com/MorningFrog/urdf-visualizer) that can display .urdf / .xacro files
> 
> created basic urdf for testing