# GUI python interfaces over ROS2

Small ROS 2 Python package providing simple GUIs and demo nodes to interact with TurtleBot3 simulation and Navigation2 (Nav2).

- PyQt5 GUIs to launch Gazebo, start/stop demo motion nodes, and interact with Nav2 waypoints.
- Lightweight demo nodes that publish `geometry_msgs/Twist` on `/cmd_vel` for simple motions.

## Features
- Start/stop Gazebo (TurtleBot3 worlds) from a GUI.
- Run simple motion demos: circle, forward/backward, and square path.
- Launch Nav2 with TurtleBot3 in Gazebo and follow saved AMCL waypoints.

Key sources:
- `gui_interface/gui_tb3.py:1` — GUI to launch Gazebo and motion nodes.
- `gui_interface/gui_nav2.py:1` — GUI to launch TB3+Nav2, save current AMCL pose, and follow waypoints.
- `gui_interface/circle_motion.py:1` — Circular motion publisher on `/cmd_vel`.
- `gui_interface/forward_backward.py:1` — Alternates forward/backward on `/cmd_vel`.
- `gui_interface/square_path.py:1` — Drives a square on `/cmd_vel`.

## Prerequisites
- ROS 2 with Nav2 (e.g., Humble or newer) and TurtleBot3 packages.
- Python 3 with PyQt5.
- Gazebo/Ignition and TurtleBot3 Gazebo worlds.

Suggested installs (adapt for your distro):

```bash
sudo apt install \
  ros-$ROS_DISTRO-turtlebot3-gazebo \
  ros-$ROS_DISTRO-turtlebot3-navigation2 \
  ros-$ROS_DISTRO-nav2-simple-commander \
  python3-pyqt5
```

Environment setup commonly needed by TurtleBot3:

```bash
export TURTLEBOT3_MODEL=burger   # or waffle, waffle_pi
```

## Build & Install
Place this package in a ROS 2 workspace and build with colcon.

```bash
# From your workspace root (e.g., ~/ros2_ws)
colcon build --packages-select gui_interface
source install/setup.bash
```

## How to Run
You can run the GUIs directly with Python or as a module. The demo motion nodes are exposed as ROS 2 executables via `console_scripts`.

### TB3 Demo GUI
Launch Gazebo and control demo motion nodes (`/cmd_vel`).

```bash
python3 src/gui_interface/gui_interface/gui_tb3.py
```

GUI buttons:
- Launch Gazebo (TB3 Empty World): starts `turtlebot3_gazebo empty_world.launch.py`.
- Start: circle_motion | forward_backward | square_path: runs the corresponding node.
- Close All Nodes: stops all motion nodes.
- Stop All (Nodes + Gazebo): stops nodes and cleans up Gazebo processes.

### Nav2 Waypoints GUI
Launch TB3 simulation + Nav2, save AMCL poses, and follow them as waypoints.

```bash
python3 src/gui_interface/gui_interface/gui_nav2.py
```

Workflow:
- Launch TB3 Sim + Nav2: starts `turtlebot3_gazebo turtlebot3_world.launch.py` and `turtlebot3_navigation2 navigation2.launch.py`.
- Save Pose: stores the current AMCL pose (topic `/amcl_pose`).
- Go To Saved Poses: waits for Nav2 to activate, then follows saved poses via `nav2_simple_commander.BasicNavigator`.
- Stop All: stops launched processes.

### Demo Nodes (CLI)
These publish `geometry_msgs/Twist` to `/cmd_vel`:

```bash
ros2 run gui_interface circle_motion
ros2 run gui_interface forward_backward
ros2 run gui_interface square_path
```

## Notes & Troubleshooting
- Always `source install/setup.bash` for your current terminal before running.
- Ensure TurtleBot3 model is set: `export TURTLEBOT3_MODEL=burger`.
- Gazebo leftovers: the GUIs try to clean up, but you can also manually run `pkill -f gzserver` or `pkill -f gazebo` if needed.
- Nav2 activation: the waypoint action waits for Nav2 to become active; initial map load and AMCL convergence can take a moment.
- Dependencies are used at runtime (PyQt5, TurtleBot3 stacks, Nav2 Simple Commander) and may need to be installed separately on your system.
