"""Unit tests for the behaviors, perception, and occupancy grid (no simulator needed)."""

import math
import random

import numpy as np
from project1_reactive.arbiter import PriorityArbiter
from project1_reactive.behaviors import build_behaviors, SensorState, VelocityCommand
from project1_reactive.geometry import FEET_TO_METERS, LidarMount
from project1_reactive.occupancy_grid import OccupancyGrid
from project1_reactive.perception import ClearanceFilter, FrontObstacles

N_BEAMS = 640
ANGLE_MIN = -math.pi
ANGLE_INC = 2.0 * math.pi / N_BEAMS


def make_scan(points):
    """Build a TurtleBot 4 style scan (lidar rotated +90 deg) from robot-frame bearings."""
    ranges = [math.inf] * N_BEAMS
    for bearing_deg, distance in points:
        scan_angle = math.radians(bearing_deg) - math.pi / 2.0     # undo the mount yaw
        i = int(round((scan_angle - ANGLE_MIN) / ANGLE_INC)) % N_BEAMS
        ranges[i] = distance
    return ranges


def setup(seed=1):
    """Return (arbiter, detector, state) with a lidar mounted at the robot center."""
    mount = LidarMount(0.0, 0.0, math.pi / 2.0)
    behaviors, detector = build_behaviors({'mount': mount}, random.Random(seed))
    state = SensorState(now=100.0, scan_time=100.0)
    return PriorityArbiter(behaviors), detector, state


def test_open_space_drives_forward():
    arbiter, _, state = setup()
    assert arbiter.step(state)[0] == 'drive_forward'


def test_spec_example_turns_right():
    """0.7 ft ahead-left and 0.9 ft straight ahead -> turn right (avoid)."""
    arbiter, detector, state = setup()
    r = 0.17
    state.obstacles = detector.detect(
        ANGLE_MIN, ANGLE_INC,
        make_scan([(30, r + 0.7 * FEET_TO_METERS), (0, r + 0.9 * FEET_TO_METERS)]),
        0.164, 12.0)
    winner, command = arbiter.step(state)
    assert winner == 'avoid'
    assert command.angular < 0.0


def test_wall_head_on_escapes_and_keeps_turning():
    arbiter, detector, state = setup()
    state.obstacles = detector.detect(ANGLE_MIN, ANGLE_INC,
                                      make_scan([(0, 0.17 + 0.2)]), 0.164, 12.0)
    assert arbiter.step(state)[0] == 'escape'
    # The obstacle disappears but the fixed action pattern keeps turning.
    state.obstacles = FrontObstacles()
    assert arbiter.step(state)[0] == 'escape'
    target = arbiter.behaviors[2].turner.target
    turn = abs(math.degrees(target))
    assert 150.0 - 1e-6 <= turn <= 180.0 + 1e-6
    # Once the heading is reached, control passes back down.
    state.pose.yaw = target
    assert arbiter.step(state)[0] == 'drive_forward'


def test_bumper_beats_teleop():
    arbiter, _, state = setup()
    state.teleop_command = VelocityCommand(0.2, 0.0)
    state.teleop_time = state.now
    state.bumper_pressed = True
    state.last_bump_time = state.now
    assert arbiter.step(state) == ('halt', VelocityCommand(0.0, 0.0))


def test_teleop_beats_escape_and_times_out():
    arbiter, detector, state = setup()
    state.obstacles = detector.detect(ANGLE_MIN, ANGLE_INC,
                                      make_scan([(0, 0.25)]), 0.164, 12.0)
    state.teleop_command = VelocityCommand(0.0, 0.5)
    state.teleop_time = state.now
    assert arbiter.step(state)[0] == 'teleop'
    state.now += 2.0
    state.scan_time = state.now
    assert arbiter.step(state)[0] == 'escape'


def test_random_turn_every_foot():
    arbiter, _, state = setup()
    assert arbiter.step(state)[0] == 'drive_forward'
    state.pose.odometer = FEET_TO_METERS + 0.01
    winner, command = arbiter.step(state)
    target = arbiter.behaviors[4].turner.target
    if target is None:                     # angle was within tolerance
        assert winner == 'drive_forward'
    else:
        assert winner == 'random_turn'
        assert abs(target) <= math.radians(15.0) + 1e-9


def test_escape_angle_distribution():
    """Escape headings fall in 180 +/- 30 degrees."""
    for seed in range(50):
        arbiter, detector, state = setup(seed)
        state.obstacles = FrontObstacles(0.1, 0.1)
        arbiter.step(state)
        turned = math.degrees(arbiter.behaviors[2].turner.target) % 360.0
        assert 150.0 - 1e-6 <= turned <= 210.0 + 1e-6


def test_no_scan_means_stop():
    arbiter, _, state = setup()
    state.scan_time = state.now - 5.0
    assert arbiter.step(state) == ('none', VelocityCommand(0.0, 0.0))


def test_grid_marks_wall():
    grid = OccupancyGrid(resolution=0.05, width_m=4.0, height_m=4.0,
                         origin_x=-2.0, origin_y=-2.0)
    angles = np.array([0.0])
    grid.integrate_scan(0.0, 0.0, 0.0, angles, np.array([1.0]), 0.1, 12.0)
    data = grid.to_ros_data().reshape(grid.height, grid.width)
    row = int(2.0 / 0.05)
    assert data[row, int(3.0 / 0.05)] > 50          # hit at x = 1.0
    assert 0 <= data[row, int(2.5 / 0.05)] < 50     # free at x = 0.5
    assert data[row + 10, int(2.5 / 0.05)] == -1    # unknown off the beam


# ----------------------------------------------------------------------
# Project 2: robustness to real sensor data
# ----------------------------------------------------------------------

def detector_with(**settings):
    """Return an ObstacleDetector built with the given extra settings."""
    params = {'mount': LidarMount(0.0, 0.0, math.pi / 2.0)}
    params.update(settings)
    return build_behaviors(params, random.Random(0))[1]


def test_self_filter_ignores_robot_body():
    """A return inside the robot's outline (a standoff) is not an obstacle."""
    detector = detector_with()
    obstacles = detector.detect(ANGLE_MIN, ANGLE_INC, make_scan([(20, 0.12)]), 0.05, 12.0)
    assert obstacles == FrontObstacles()


def test_outlier_beams_ignore_single_ghost_reading():
    """One spurious short beam is ignored; a real object (many beams) is not."""
    detector = detector_with(outlier_beams=2)
    ghost = detector.detect(ANGLE_MIN, ANGLE_INC, make_scan([(20, 0.3)]), 0.164, 12.0)
    assert ghost == FrontObstacles()
    box = make_scan([(angle, 0.3) for angle in range(15, 26)])
    real = detector.detect(ANGLE_MIN, ANGLE_INC, box, 0.164, 12.0)
    assert math.isclose(real.left, 0.3 - 0.17)


def test_invalid_readings_are_skipped():
    """NaN, inf, zero and below-minimum ranges are all ignored."""
    detector = detector_with()
    ranges = make_scan([])
    ranges[0:4] = [math.nan, 0.0, 0.1, math.inf]
    assert detector.detect(ANGLE_MIN, ANGLE_INC, ranges, 0.164, 12.0) == FrontObstacles()


def test_clearance_filter_removes_one_bad_scan():
    """A median over 3 scans hides a glitch that lasts one scan."""
    scan_filter = ClearanceFilter(3)
    clear = FrontObstacles()
    scan_filter.add(clear)
    scan_filter.add(clear)
    assert scan_filter.add(FrontObstacles(0.1, 0.1)) == clear
    scan_filter.add(FrontObstacles(0.1, 0.1))           # 2 of 3 bad: now real
    assert scan_filter.add(clear).left == 0.1


def test_avoid_keeps_direction_despite_noise():
    """Once avoid starts turning left it does not flip right on jitter."""
    arbiter, _, state = setup()
    avoid = arbiter.behaviors[3]
    avoid.hysteresis = 0.05
    state.obstacles = FrontObstacles(left=0.25, right=0.15)    # closer right: turn left
    winner, command = arbiter.step(state)
    assert winner == 'avoid' and command.angular > 0.0
    state.obstacles = FrontObstacles(left=0.15, right=0.25)    # noise swaps the sides
    assert arbiter.step(state)[1].angular > 0.0


def test_avoid_hysteresis_release():
    """Avoid keeps turning just past 1 ft and lets go beyond the margin."""
    arbiter, _, state = setup()
    arbiter.behaviors[3].hysteresis = 0.05
    state.obstacles = FrontObstacles(left=0.2, right=math.inf)
    assert arbiter.step(state)[0] == 'avoid'
    state.obstacles = FrontObstacles(left=FEET_TO_METERS + 0.03, right=math.inf)
    assert arbiter.step(state)[0] == 'avoid'
    state.obstacles = FrontObstacles(left=FEET_TO_METERS + 0.06, right=math.inf)
    assert arbiter.step(state)[0] == 'drive_forward'


def test_stalled_escape_turn_times_out():
    """If the heading never changes, escape gives up instead of spinning forever."""
    arbiter, _, state = setup()
    state.obstacles = FrontObstacles(0.1, 0.1)
    assert arbiter.step(state)[0] == 'escape'
    state.obstacles = FrontObstacles()
    state.now += 3.0
    state.scan_time = state.now
    assert arbiter.step(state)[0] == 'escape'             # still within its time limit
    state.now += 60.0
    state.scan_time = state.now
    assert arbiter.step(state)[0] == 'drive_forward'
    assert arbiter.behaviors[2].turner.timed_out
