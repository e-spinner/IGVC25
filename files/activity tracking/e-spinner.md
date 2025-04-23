
## 422-2025 - 7.5 hrs
#### 10:45 AM - 1:00 PM: Setup robot_description
> made basic urdf file to allow dynamic loading of different robots
>
> created launch file that makes a basic rsp node

#### 2:30 PM - 4:00PM: basic robot 1
> made simple adjustable 2-wheel + caster robot urdf
>
> want to make whole project one package called igvc25

#### 4:00 PM - 7:00 PM: setting up gazebo
> Working on getting the simulation set up,
> installed gazebo [Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/)
>
> Started working on sim.launch.py, and porting igvc23 code, and fixing it

> *sim.launch.py only works in ubbuntu terminal, not vscode terminal*

#### 9:00 PM - 10:45 PM: Simulation of Lidar
> got tf portion of vlp16 lidar set up, need to add simlated laser to gazebo harmonic

## 04-21-2025 - 2.75 hrs
#### 2:15 PM - 3:30 PM: Setup Ros2 Environment
> followed this tutorial to install ros2 [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html).
> 
> ran the basic testing talker / listener example in above link
> 
> Found and installed node visualization software: [ros2_graph](https://github.com/kiwicampus/ros2_graph)

#### 3:30 PM - 4:00 PM: Made this repo

#### 5:30 PM - 6:05 PM: Setup Repo + Ported from old team code
> Wrote basic readme that explains how to build a ros2 application
>
> Ported over and tested code from IGVC23 repo, went poorly:
> ![alt text](/files/assets/04-21-25-failed_build.png)
>
> Old code currently gitignored

#### 6:05 PM - 6:30 PM: Expieramented with URDF files
> found [vscode extension](https://github.com/MorningFrog/urdf-visualizer) that can display .urdf / .xacro files
> 
> created basic urdf for testing