# project1_reactive

CS 4023/5023 Projects 1 and 2: a TurtleBot 4 (ROS 2 Jazzy) that wanders using
six prioritized reactive behaviors and builds an occupancy-grid map as it goes.
Project 1 ran it in Gazebo; Project 2 runs the same code on the physical robot
in REPF B4 (see "Physical robot" below, `TUNING.md`, and `CHANGES_PROJECT2.md`).

## Files

| Path | What it is |
| --- | --- |
| `worlds/project1.sdf` | Room (10 x 15 ft) + 5 ft hallway wrapped around its east and south sides |
| `launch/project1.launch.py` | Starts Gazebo, the TurtleBot 4, our two nodes and RViz |
| `config/params.yaml` | Tunable settings for the simulation (speeds, 1 ft threshold, lidar mounting) |
| `config/robot_params.yaml` | Settings for the physical robot (Project 2) |
| `launch/robot.launch.py` | Starts our nodes for the physical robot (no simulator) |
| `rviz/project1.rviz` | RViz view of the map, lidar and robot pose |
| `project1_reactive/behaviors.py` | The six behaviors (halt, teleop, escape, avoid, random turn, drive) |
| `project1_reactive/arbiter.py` | Priority arbiter (pseudo-subsumption) |
| `project1_reactive/perception.py` | Turns a lidar scan into front-left / front-right clearances |
| `project1_reactive/geometry.py` | Lidar mounting pose, robot pose tracker, angle helpers |
| `project1_reactive/reactive_controller.py` | ROS node: sensors -> arbiter -> `/cmd_vel` |
| `project1_reactive/occupancy_grid.py` | Log-odds occupancy grid |
| `project1_reactive/occupancy_mapper.py` | ROS node: `/scan` + `/odom` -> `/map` |
| `test/` | Style checks and unit tests |

## Build and run (CSN Linux boxes)

1. Open a terminal and load ROS 2:

       source /opt/ros/jazzy/setup.bash

2. Put the package in a workspace and build it:

       mkdir -p ~/p1_ws/src && cd ~/p1_ws/src
       git clone git@github.com:ColinMathGeek987/project1_reactive.git
       cd ~/p1_ws
       colcon build --symlink-install
       source install/setup.bash

3. Launch the simulation:

       ros2 launch project1_reactive project1.launch.py

   Gazebo and RViz open. The robot starts on its dock in the middle of the
   room, undocks by itself (about 10 s), then starts wandering.

   Optional arguments: `x:=`, `y:=`, `yaw:=` (start pose, meters / radians),
   `world:=` (another world file in `worlds/`, name without `.sdf`),
   `use_rviz:=false`.

4. Keyboard control: open a second terminal and run

       source /opt/ros/jazzy/setup.bash
       ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/key_vel -p stamped:=true

   Keep that terminal focused and use `u i o / j k l / m , .` to drive.
   Autonomous behavior resumes 1 s after the last key press.

5. Useful checks:

       ros2 topic echo /active_behavior     # which behavior is in control
       ros2 topic hz /map                   # map updates (~1 Hz)

Stop everything with Ctrl+C in each terminal.

## Physical robot (Project 2)

The robot's own computer runs the lidar, Create 3 and TF drivers; the lab
machine only runs our node and the keyboard. Both must use the robot's
`ROS_DOMAIN_ID` (and discovery-server settings, if the lab uses them).

    source /opt/ros/jazzy/setup.bash
    cd ~/p1_ws && colcon build --symlink-install && source install/setup.bash
    ros2 topic list                       # must show the robot's scan, odom, hazard_detection
    ros2 launch project1_reactive robot.launch.py            # add namespace:=/<ns> if topics are namespaced

Keyboard, in a second terminal (prefix the topic with the namespace if any):

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=key_vel -p stamped:=true

At start-up the node switches off the Create 3 bump reflexes so that our
halt behavior decides what happens on contact. See `TUNING.md` for the lab
tuning procedure.

## Behaviors (highest priority first)

1. **Halt**: stop while a bumper reports a collision, and for 1 s after.
2. **Teleop**: obey `/key_vel` keyboard commands (for 1 s after each key).
3. **Escape**: obstacles within 1 ft on both front sides at nearly equal
   distance -> turn to 180 +/- 30 deg away, finishing the turn even if the
   obstacle disappears (fixed action pattern).
4. **Avoid**: obstacle within 1 ft in front but not symmetric -> turn away
   from the closer side, only while it lasts (reflex).
5. **Random turn**: every 1 ft of forward travel, turn by a random angle in
   [-15, 15] deg.
6. **Drive forward** at 0.2 m/s.

Mapping runs in parallel in its own node and never moves the robot.

## Sources

* Gazebo set-up in `launch/project1.launch.py` is adapted from
  `turtlebot4_gz_bringup/launch/sim.launch.py` (TurtleBot 4 simulator,
  Clearpath Robotics, Apache 2.0); the robot is spawned with that package's
  unmodified `turtlebot4_spawn.launch.py`.
* Physics and plugin blocks in `worlds/project1.sdf` follow the TurtleBot 4
  sample worlds `warehouse.sdf` and `maze.sdf`; the walls are original.
* `test/test_flake8.py` and `test/test_pep257.py` are the standard files
  generated by `ros2 pkg create` (Open Source Robotics Foundation, Apache 2.0).
* Keyboard control uses the `teleop_twist_keyboard` package, unmodified.
* The remaining code was written for this project with the help of an AI
  assistant (Claude, Anthropic); state this per your course's AI policy.
