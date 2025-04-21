# IGVC25
ONU Capstone IGVC 2025-26


[Repo breakdown](documentation/docs/breakdown.md)

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
ros2 launch robot_visualization sim.launch.py 
world:=./src/robot_visualization/worlds/nav_test.world
```

### Robot Launch File

```bash
ros2 launch robot_visualization robot.launch.py
```

---

##