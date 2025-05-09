
## 04-30-2025 -
#### 01:30 PM - 03:30 PM: simplified bt
> also robot is going faster now that i switched the mppi controller for a dwb controller, not sure if this is a fix for [#6], or a transfer of the problem 
#### 06:00 PM - 08:30 PM
> slowly building up bt logic, solid progress. dwb controller much easier to tune, [#6] is good.
>
>

## 04-28-2025 - 04.00 hrs
#### 01:00 PM - 04:30 PM: fix [#3] and [#5]!
> Stepping back rom the issues for a bit, start over from [slam](https://www.youtube.com/watch?v=ZaiA3hWaRzE&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=17)
>
> Using slam_toolbox for only localization, and scan data for costmaps seems better, might still want to use slam for a static layer, but might not need it all if I ever figure out how to get ekf sensor fusion to do anything
#### 04:30 PM - 05:00 PM
> time to investigate bt's w/ groot 2 to try to add path smoothing and waypoint following. Might be able to fix [#6].
>
> Made a basic start on custom bt

## 04-27-2025 - 03.50 hrs
#### 06:00 PM - 09:30 PM: sadness
> what to learn how to use groot for bt-navigator
>
> Actually, i want to set up better localization first
>
> tried to set up some namespace stuff, everything is broken
>
> ah

## 04-26-2025 - 03.00 hrs
#### 02:00 PM - 05:00 PM
> working on getting navigation actually working
> , want to switch to sensor fusion localization, not just lidar localiztion based on walls from slam.
>
> might be able to give a starter empty map to slam to fix [#3]?

## 04-25-2025 - 11.00 hrs
#### 08:00 AM - 12:00 PM: trying to get slam to work
> going to look in depth into nav2 stack to figure out me problems
>, [video](https://www.youtube.com/watch?v=idQb2pB-h2Q)
>
> Also got new worlds to simulate in
>
> Maybe cartographer-ros?
#### 01:00 PM - 02:30 PM: slam works, get nav2 back running
> The problem with slam is that the initial map it was creating was smaller than the registered footprint of the robt, eiter need to find a real way to intialize a biggere sized costmap, or have to start with things around robot so that map is big enough.
>
> Nav2 stack is launching properly now, adjusted robot velocities to match competition standards, but robot path planning seems to fail for no good reason.
#### 09:00 PM - 02:30 AM: manually launch nav2
> To debud nav2 issues, going to manualy launch nav stack, no bringup, to hopefully learn how all the bits actually work
>
> robot just randomly stops, also drives horribly slow

## 04-24-2025 - 10.75 hrs
#### 10:00 AM - 02:45 PM: Simulate additional sensors
> starting to look into using nav2
>
> setup imu, gps, and two cameras
>
> right camera is not properly aligned:
> ![alt text](files/assets/04-24-25-camera_alignment.png)
#### 03:15PM - 05:30: begin work on nav2 stack
> gonna try to get nav2 set up
>
> slam is working, added links to various sources
>
> scan qos = bad, need to republish at reliable level to get amcl working
#### 06:00 PM - 07:45 PM: nav2_bringup
> made scripts package for custom py nodes, needed to make custom republisher for /scan
>
> nav2 bringup failing part way through, only gets some nodes
#### 09:00 PM - 11:00 PM: costmap issues
> nav2 bringup fixed, however costmap issues
>
> no hope, costmaps and slam hates me
 
## 04-23-2025 - 04.75 hrs
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

## 04-22-2025 - 08.50 hrs
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

## 04-21-2025 - 02.75 hrs
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