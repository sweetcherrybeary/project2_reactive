# Tuning on the physical TurtleBot 4 (Project 2)

All tuning is done in `config/robot_params.yaml`; no code changes are needed.
Because the package is built with `--symlink-install`, just edit the file and
relaunch. A parameter can also be changed live, for example
`ros2 param set /reactive_controller forward_speed 0.12`. Note that most
settings are only read at start-up, so relaunch to be sure.

Two people: one watches the robot with a hand near it, one at the keyboard.
Keep `teleop_twist_keyboard` open the whole time. Pressing `k` stops the
robot within one control cycle, because teleop outranks everything except halt.

## 0. Before the first run (once per robot)

| Check | Command | Expected / what to change |
| --- | --- | --- |
| Robot is reachable | `ros2 topic list` | `scan`, `odom`, `hazard_detection`, `cmd_vel`, `tf` appear. If they appear as `/<ns>/scan`, launch with `namespace:=/<ns>`. If nothing appears, set `ROS_DOMAIN_ID` (and the discovery server, if the lab uses one) to the robot's values. |
| `cmd_vel` message type | `ros2 topic info -v cmd_vel` | `geometry_msgs/msg/TwistStamped` → `cmd_vel_stamped: true`; `Twist` → `false`. A wrong type means the robot ignores us silently. |
| Bumper topic | `ros2 topic echo hazard_detection` and press the bumper by hand | A detection with `type: 1` (BUMP). Note whether messages stop after release. The code works either way (`bump_timeout`). |
| Lidar orientation | `log_clearances: true`, hold a box 25 cm in front of the bumper, then front-left, then front-right | Front: left ≈ right ≈ 0.25 m (clearance is from the body). Front-left: left is smaller. If left/right are swapped or the front box is not seen, TF is missing: set `lidar_yaw_offset_deg` (+90 for the standard TB4). |
| Reflexes are off | launch log shows `Create 3 bump reflexes disabled.` | If it warns instead, run `ros2 param set /motion_control reflexes_enabled false` (with the namespace prefix if any) before the demo. |
| Teleop reaches us | Teleop terminal: `... --ros-args -r cmd_vel:=key_vel -p stamped:=true` (add `-r __ns:=/<ns>` if namespaced) | Log shows `Active behavior: teleop` on a key press. |

## 1. Tuning order and symptoms

Tune from the top of this list down, because later items depend on earlier ones.

| Symptom on the real robot | Parameter(s) | Direction |
| --- | --- | --- |
| Escape or avoid fires in open space ("ghost" obstacles) | `outlier_beams`, `scan_filter_window` | Raise `outlier_beams` to 3–4; keep window at 3. Check that the ghost is not part of the robot: raise `self_filter_margin` slightly *negative* (e.g. -0.02) to filter a little further out. |
| Robot touches walls before reacting | `forward_speed`, `scan_filter_window` | Lower speed (0.12); lower the window to 1 if the delay is the problem. Do **not** raise `obstacle_distance`; 1 ft is the spec. |
| Head-on walls cause avoid (wiggle) instead of escape | `symmetry_tolerance` | Raise in 0.02 steps (max ≈ 0.12). |
| Angled walls cause escape instead of avoid | `symmetry_tolerance` | Lower in 0.02 steps. |
| Avoid flickers on/off at 1 ft | `avoid_hysteresis` | Raise to 0.08. |
| Side objects (doorframes) trigger avoid | `front_half_angle_deg` | Lower to 45. |
| Turns overshoot and wobble back and forth at the end | `turn_tolerance_deg`, `turn_speed` | Raise tolerance to 5; lower `turn_speed`. |
| Turns stall short of the target (wheels hum, no motion) | `min_turn_speed` | Raise to 0.4. |
| Escape is visibly not ~180° | Check `odom` (`ros2 topic echo odom` while turning by hand) | Odometry yaw must change ~3.14 rad for a half turn. |
| Random turn not about every foot | `random_turn_step` | Measure 1 ft on the floor with tape; odometry should agree to ±3 cm. If not, it is wheel slip, so report rather than retune. |

Escape angle (180° ± 30°), random-turn range (±15°) and the 1 ft distances are
fixed by the assignment; leave them alone.

## 2. Arbitration checks (record pass/fail for the demo)

1. **Bumper beats teleop:** hold `i` (forward) and press the bumper by hand.
   The robot stops and the log shows `halt`. It resumes 1 s after release.
2. **Teleop beats escape:** drive at a wall with `i` until within 1 ft.
   The robot keeps obeying the keyboard. Release the keys, and 1 s later it escapes.
3. **Escape:** robot drives straight at a flat wall and turns ~150–210°.
4. **Avoid:** a box ahead-left (about 30°) makes it turn right, and ahead-right makes it turn left.
5. **Random turn:** in open space, small turns about every foot (watch
   `ros2 topic echo active_behavior`).
6. **Low obstacles:** the lidar is about 20 cm up, so anything lower is
   only caught by the bumper. Use test boxes taller than the lidar.
