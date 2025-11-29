# IGVC25 — Lane detector prototype (branch: lane_following_restart_from_main)

This branch contains a minimal prototype for camera-based lane detection that
projects detected lane pixels to the ground and publishes them so Nav2 can
avoid painted lanes in simulation.

Summary of what's included
- `src/igvc/igvc_vision/lane_detector.py` — prototype node (rclpy). Publishes:
  - `/lane_points` (sensor_msgs/PointCloud2)
  - `/scan_lane` (sensor_msgs/LaserScan) — useful for Nav2 obstacle layers
  - `/lane_follow/debug_left`, `/lane_follow/debug_right` (sensor_msgs/Image)
- `src/igvc/config/navigation.yaml` — updated to include `scan_lane` as an observation source (so Nav2 can observe `/scan_lane`).
- `tests/test_projection.py` and `tests/test_projection_extra.py` — unit tests for the projection math (fallback, TF-disabled mode).
- `tools/sim_publish_camera.py` — synthetic camera + CameraInfo publisher (5s) for quick integration checks.
- `tools/listen_lane_points.py` — timed listener that waits for `/lane_points` and prints metadata.

Branch and PR
- Branch: `lane_following_restart_from_main`
- A new remote branch was pushed; you can create a PR with the GitHub hint shown when the branch was pushed.

Quick usage

1) Run unit tests (fast, no ROS nodes required):

```bash
python3 tests/test_projection.py
python3 tests/test_projection_extra.py
```

2) Quick integration check (three separate terminals)

- Terminal A — start the lane detector node (it will listen for camera topics):

```bash
python3 src/igvc/igvc_vision/lane_detector.py
```

- Terminal B — publish synthetic camera frames for ~5s:

```bash
python3 tools/sim_publish_camera.py
```

- Terminal C — wait for `/lane_points` and print a summary:

```bash
python3 tools/listen_lane_points.py
```

Notes
- The node tries to use TF2 when available; if TF lookups fail it falls back to parameters:
  - `camera_height` (meters)
  - `camera_pitch_deg` (degrees)
  - `use_tf` (bool) — set to `False` to force fallback mode.
- For integration tests the node defaults `debug_publish_test_points` to `True`, which makes it publish a small synthetic line of points when no lane detections are found. Set this parameter to `False` in real runs.

Next recommended steps
- Create a ROS2 launch to start the node and the synthetic publisher together for deterministic testing.
- Tune Canny / Hough parameters and sampling density for better detection in sim.
- Add a small integration test that runs the node and asserts `/lane_points` is published.

If you'd like, I can open a PR description and add a short `CHANGELOG.md` summarizing the work for reviewers.

---
Commit: lane_following_restart_from_main (pushed to origin)
# igvc Capstone Software
> subtitle!
---

To use this software you first need an ubuntu 22 environment:

1.  Ubuntu computer
2.  [WSL](https://learn.microsoft.com/en-us/windows/wsl/install)
3.  [VM](https://www.virtualbox.org)

Once you are in, I reccomend setting up [ssh with git.](https://docs.github.com/en/authentication/connecting-to-github-with-ssh) This will make pushing and pulling eaiser.

Either way, clone this repo, and navigate to it via terminal:

```bash
git clone git@github.com:e-spinner/igvc.git && cd igvc
```

### Installing Ros

The following script will install ros-humble and the required packages for this repo + build the software:

```bash
./setup.sh
```

> If this raises a permision denied error. run `chmod +x ./setup.sh` and try again.

### Setting up Sensors

If you need to use the Sensors, run the following command to setup udev rules that the software relies on to identify which usb port devices are in:

```bash
sudo cp 99-usb-sensors.rules /etc/udev/rules.d/99-usb-sensors.rules
```

now reload the udev rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```


### Building the software again

```bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-select igcv26
```

### Package Structure

The igvc package contains:

- `behavior/` - Behavior tree configurations for autonomous navigation
- `config/` - Configuration files for ros nodes
- `description/` - Robot URDF models (ackermann, ackermann_linkage, ...)
- `igvc_py/` - Python nodes
- `launch/` - Launch files to start different systems
- `msg/` - Custom ROS message definitions
- `src/` - C++ nodes
- `worlds/` - Gazebo simulation worlds

### Running the System

After building, source the workspace:

```bash
source install/setup.bash
```

Run different system configurations:

```bash
# Ackermann linkage visualzation + control by joy-teleop
ros2 launch igvc ack.launch.py linkage_config:=2

# Navigation stack  --- intended to be used on top of sim
ros2 launch igvc nav.launch.py

# Simulation environment + basic ackermann robot
ros2 launch igvc sim.launch.py

# Vision sensors drivers (GPS and IMU)
ros2 launch igvc vision.launch.py
```