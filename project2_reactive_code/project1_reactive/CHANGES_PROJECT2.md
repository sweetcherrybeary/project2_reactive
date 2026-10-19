# Code change log: Project 1 → Project 2 (Person A, for deliverable 6.5)

Baseline: the Project 1 submission, tagged `project1` in git.
`git diff project1..HEAD` shows every line. Launch-file changes are listed
for completeness; Person B owns the final launch file and its write-up.

The architecture (pseudo-subsumption, six behaviors, priority arbiter) and
the behavior priorities are **unchanged**. Every change addresses a
difference between the Gazebo robot and the physical TurtleBot 4.

## Perception (`project1_reactive/perception.py`)

| # | Change | Reason |
| --- | --- | --- |
| P1 | **Self-filter.** Lidar returns closer to the robot's center than its radius (`robot_radius - self_filter_margin`) are ignored. | The simulated lidar never sees the robot. On the real robot, any return from the robot's own structure (standoffs, cables) gives a *negative* clearance, which reads as "symmetric obstacle" and makes the robot spin in escape forever. In our noisy-room test the Project 1 code spent 49% of its time escaping and drove only 1.5 m in 15 min. |
| P2 | **Outlier rejection** (`outlier_beams`, new `robust_minimum`). The N smallest clearances per sector are discarded, and a sector with N or fewer returns counts as clear. | Real lidars produce occasional short "ghost" readings (dust, reflections, edges). Project 1 used the single minimum, so one bad beam was enough to trigger escape or avoid. A real obstacle within 1 ft spans many beams (0.56° apart), so it is still detected. |
| P3 | **Temporal median filter** (`ClearanceFilter`, `scan_filter_window`, default 3). | Removes glitches that last one scan. It costs about one scan period of delay (~2 cm at 0.15 m/s). It uses the lower median, which errs on the side of seeing an obstacle. |
| P4 | Comments clarify that inf/NaN/0/out-of-range readings are discarded. | Real sensors return all of these; the filtering already existed, but now it is documented why. |

## Behaviors (`project1_reactive/behaviors.py`)

| # | Change | Reason |
| --- | --- | --- |
| B1 | **Turn time limit** in `TurnToHeading` (`turn_timeout_factor`, default 3× the nominal turn time + 1 s). A stalled turn ends and control drops to lower behaviors. | On hardware a turn can stall (a wheel on a cable, or the robot wedged against something below the lidar). Project 1 would then hold the escape or random turn forever. |
| B2 | **Avoid: direction lock + hysteresis** (`avoid_hysteresis`, default 0.05 m). Once avoid picks a direction it keeps it until the nearest clearance exceeds 1 ft + 5 cm; suppression clears the lock. | Real clearances jitter by a few centimeters, so with Project 1's memoryless reflex the robot would dither left-right when the two sides are similar, and flicker on/off right at 1 ft. It is still a reflex: no plan, and the memory lasts only as long as the stimulus. |
| B3 | Avoid has its own turn speed (`avoid_turn_speed`). | It can be tuned separately from the escape and random-turn speed. |
| B4 | Escape: when a turn finishes (or times out) in the same cycle that the obstacle is still symmetric, a new escape starts immediately instead of passing one cycle to lower behaviors. | Prevents a 50 ms lurch forward toward the wall between two escape turns. |
| B5 | Turn speed floor and stopping tolerance are now parameters (`min_turn_speed`, `turn_tolerance_deg`). | Real motors have a dead band and overshoot. These were hard-coded (0.3 rad/s, 3°) in Project 1. |

## ROS node (`project1_reactive/reactive_controller.py`)

| # | Change | Reason |
| --- | --- | --- |
| N1 | **Relative topic names** (`scan`, `odom`, `cmd_vel`, `hazard_detection`, `key_vel`, `dock_status`, `undock` instead of `/scan` …). | Lab robots are often namespaced (e.g. `/tb4_3/scan`). Relative names follow the launch file's `namespace`, while an empty namespace gives the same topics as before, so the simulation is unaffected. |
| N2 | **Bumper from report age** (`bump_timeout`, 0.25 s): pressed = a BUMP was reported in the last 0.25 s. | Project 1 cleared the bumper only when a message *without* BUMP arrived. If the robot does not send one after release, Project 1 stays halted forever. Our test reproduced this: after one bump the Project 1 node halted for the rest of the run. |
| N3 | **Disable Create 3 bump reflexes** at start-up (`disable_reflexes`, calls `motion_control/set_parameters` with `reflexes_enabled=false`). | The real Create 3 backs away on its own when bumped, which overrides our halt behavior (the spec requires a halt). The simulator does not model these reflexes. |
| N4 | **Scan pipeline** uses the self-filter, outlier rejection and median filter (P1–P3), and clears the filter after a gap of more than 1 s in the scans. | Wires the perception changes in; stale history must not be mixed with new scans. |
| N5 | **Final stop on Ctrl+C**: `rclpy.init(..., signal_handler_options=SignalHandlerOptions.NO)`. | With rclpy's default handler the ROS context is already shut down when Python sees Ctrl+C, so Project 1's final stop command was never sent. In Gazebo this did not matter; a real robot would keep rolling until the base's command timeout. |
| N6 | **`log_clearances`** option prints left/right clearances at 2 Hz. | Needed to tune the thresholds in the lab. |
| N7 | New parameters declared for B1–B5 and N2–N6. | Everything stays tunable from YAML; no numbers are buried in code. |

## Other files

| # | File | Change | Reason |
| --- | --- | --- | --- |
| O1 | `config/robot_params.yaml` (new) | Physical-robot settings: `forward_speed` 0.2 → 0.15 m/s, `turn_speed` 1.0 → 0.8 rad/s, `symmetry_tolerance` 0.05 → 0.08 m, plus the new robustness settings. The keys use `/**/` so they match the node in any namespace. | Slower motion gives more stopping margin for sensor delay. A wider symmetry tolerance allows for real noise. `params.yaml` is kept unchanged for the simulation. |
| O2 | `launch/robot.launch.py` (new) | Starts our node(s) only, with no Gazebo, spawn or clock bridge. It has a `namespace` argument, remaps TF for namespaced robots, and uses `use_sim_time` false. | The robot's own computer runs its drivers. **Person B: replace or extend as needed for 6.1/6.2.** |
| O3 | `project1_reactive/occupancy_mapper.py` | Relative topic names. | Same as N1. |
| O4 | `package.xml`, `setup.py` | Version 2.0.0, `rcl_interfaces` dependency, updated description. | N3 uses `rcl_interfaces`. |
| O5 | `test/test_behaviors.py` | 7 new tests: self-filter, ghost-beam rejection, invalid readings, median filter, avoid direction lock, avoid hysteresis, stalled-turn timeout (16 tests total, all pass). | Covers each robustness change. |
| O6 | `README.md`, `TUNING.md` (new) | Physical-robot run notes and the lab tuning procedure. | For the team and the lab sessions. |

## Still to fill in after the lab sessions

Record the final values of each **TUNE** parameter in `robot_params.yaml` and
why it was changed, e.g. "`outlier_beams` 2 → 3: phantom escapes near the
lab's glass door". These are code-relevant changes for 6.5 as well.

| Parameter | Start value | Final value | Reason (observed on the robot) |
| --- | --- | --- | --- |
| forward_speed | 0.15 | | |
| turn_speed | 0.8 | | |
| symmetry_tolerance | 0.08 | | |
| outlier_beams | 2 | | |
| scan_filter_window | 3 | | |
| avoid_hysteresis | 0.05 | | |
| front_half_angle_deg | 60 | | |
| turn_tolerance_deg / min_turn_speed | 3 / 0.3 | | |
