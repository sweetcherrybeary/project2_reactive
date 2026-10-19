"""
The six reactive behaviors, in priority order (1 = highest).

1. ``HaltBehavior``        -- stop when a bumper reports a collision.
2. ``TeleopBehavior``      -- obey keyboard commands from a human.
3. ``EscapeBehavior``      -- turn around (180 +/- 30 deg) from symmetric
                              obstacles within 1 ft; a fixed action pattern.
4. ``AvoidBehavior``       -- turn away from asymmetric obstacles within
                              1 ft; a reflex.
5. ``RandomTurnBehavior``  -- turn by a random angle in [-15, 15] deg after
                              every 1 ft of forward travel.
6. ``DriveForwardBehavior`` -- drive straight ahead.

Every behavior has the same interface (see ``Behavior``): ``update`` looks at
the shared ``SensorState`` and either returns a ``VelocityCommand`` (it wants
control) or ``None`` (it does not). The ``PriorityArbiter`` in ``arbiter.py``
decides which request is sent to the wheels.

This module contains no ROS code, so the behaviors can be unit tested alone.
"""

from dataclasses import dataclass, field
import math
import random

from project1_reactive.geometry import FEET_TO_METERS, PoseTracker, wrap_angle
from project1_reactive.perception import FrontObstacles, ObstacleDetector


@dataclass
class VelocityCommand:
    """A motor command: forward speed (m/s) and turn rate (rad/s, + = left)."""

    linear: float = 0.0
    angular: float = 0.0


STOP = VelocityCommand(0.0, 0.0)


@dataclass
class SensorState:
    """
    Latest sensor readings, shared by all behaviors.

    The ROS node fills this in from its subscriptions; behaviors only read it.
    """

    now: float = 0.0                                   # current time (s)
    pose: PoseTracker = field(default_factory=PoseTracker)
    obstacles: FrontObstacles = field(default_factory=FrontObstacles)
    scan_time: float = -math.inf                       # when the last scan arrived
    bumper_pressed: bool = False
    last_bump_time: float = -math.inf
    teleop_command: VelocityCommand = field(default_factory=VelocityCommand)
    teleop_time: float = -math.inf                     # when the last key command arrived

    def scan_is_fresh(self, max_age=1.0):
        """Return True if a laser scan arrived within ``max_age`` seconds."""
        return self.now - self.scan_time <= max_age


class Behavior:
    """
    Base class for all behaviors.

    Subclasses override ``update``. ``suppress`` is called by the arbiter when
    a higher-priority behavior has control, so a behavior that is in the
    middle of a multi-step action can abandon it.
    """

    name = 'behavior'

    def update(self, state):
        """Return a VelocityCommand to request control, or None to pass."""
        raise NotImplementedError

    def suppress(self):
        """Tell the behavior a higher-priority behavior has taken control."""


class TurnToHeading:
    """
    Proportional controller that turns in place to a target heading.

    On the physical robot a turn can stall (a wheel caught on a cable, the
    robot wedged against something the lidar cannot see), so every turn
    also has a time limit. A turn that runs out of time simply ends, and
    control drops back to the lower-priority behaviors.
    """

    def __init__(self, max_speed=1.0, min_speed=0.3, gain=2.0,
                 tolerance=math.radians(3.0), timeout_factor=3.0):
        """
        Set the turn speeds (rad/s) and the stopping tolerance (rad).

        ``timeout_factor``: a turn is abandoned after this many times the
        time it would take at ``max_speed``, plus one second (0 = no limit).
        """
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.gain = gain
        self.tolerance = tolerance
        self.timeout_factor = timeout_factor
        self.target = None
        self.deadline = math.inf
        self.timed_out = False

    def start(self, target_yaw, current_yaw=None, now=None):
        """Begin turning toward ``target_yaw`` (rad, odometry frame)."""
        self.target = wrap_angle(target_yaw)
        self.timed_out = False
        self.deadline = math.inf
        if self.timeout_factor > 0.0 and current_yaw is not None and now is not None:
            angle = abs(wrap_angle(self.target - current_yaw))
            self.deadline = now + self.timeout_factor * angle / self.max_speed + 1.0

    def cancel(self):
        """Stop turning."""
        self.target = None

    @property
    def active(self):
        """Return True while a turn is in progress."""
        return self.target is not None

    def step(self, current_yaw, now=None):
        """Return the command for this cycle, or None once the target is reached."""
        if self.target is None:
            return None
        if now is not None and now > self.deadline:
            self.target = None
            self.timed_out = True
            return None
        error = wrap_angle(self.target - current_yaw)
        if abs(error) <= self.tolerance:
            self.target = None
            return None
        speed = min(self.max_speed, max(self.min_speed, self.gain * abs(error)))
        return VelocityCommand(0.0, math.copysign(speed, error))


class HaltBehavior(Behavior):
    """Priority 1: stop while a bumper is pressed, and briefly afterwards."""

    name = 'halt'

    def __init__(self, hold_time=1.0):
        """``hold_time``: seconds to stay stopped after the bumper releases."""
        self.hold_time = hold_time

    def update(self, state):
        """Request a full stop if a collision was detected recently."""
        if state.bumper_pressed or state.now - state.last_bump_time < self.hold_time:
            return STOP
        return None


class TeleopBehavior(Behavior):
    """Priority 2: pass keyboard commands through while they keep arriving."""

    name = 'teleop'

    def __init__(self, timeout=1.0):
        """``timeout``: seconds a key command stays in force."""
        self.timeout = timeout

    def update(self, state):
        """Request the latest keyboard command if it is recent."""
        if state.now - state.teleop_time <= self.timeout:
            return state.teleop_command
        return None


class EscapeBehavior(Behavior):
    """
    Priority 3: escape symmetric obstacles (fixed action pattern).

    When obstacles are within 1 ft on both front sectors at roughly equal
    distance, pick a new heading 180 deg +/- up to 30 deg away and keep
    turning until it is reached, even if the obstacles stop being seen.
    """

    name = 'escape'

    def __init__(self, detector, turner, rng=None,
                 turn_angle=math.pi, jitter=math.radians(30.0)):
        """Configure the escape turn: nominal angle and random jitter (rad)."""
        self.detector = detector
        self.turner = turner
        self.rng = rng if rng is not None else random.Random()
        self.turn_angle = turn_angle
        self.jitter = jitter

    def update(self, state):
        """Keep turning if already escaping; otherwise check the trigger."""
        yaw = state.pose.yaw
        if self.turner.active:
            command = self.turner.step(yaw, state.now)
            if command is not None:
                return command
        if state.scan_is_fresh() and self.detector.is_symmetric(state.obstacles):
            offset = self.rng.uniform(-self.jitter, self.jitter)
            self.turner.start(yaw + self.turn_angle + offset, yaw, state.now)
            return self.turner.step(yaw, state.now)
        return None

    def suppress(self):
        """Abandon the escape turn if halt or teleop takes over."""
        self.turner.cancel()


class AvoidBehavior(Behavior):
    """
    Priority 4: turn away from asymmetric obstacles (reflex).

    Turns away from the side with the closer obstacle, and only for as long
    as the obstacle is within 1 ft in front.

    Two additions for the physical robot stop it from dithering. Real
    clearances jitter by a few centimeters from scan to scan:

    * Direction lock: once a turn direction is chosen it is kept while the
      stimulus lasts, so left/right noise cannot reverse the turn.
    * Hysteresis: the stimulus ends only when the nearest clearance rises
      above ``threshold + hysteresis``, not as soon as it crosses 1 ft.

    The behavior is still a reflex: no plan and no memory beyond the
    current stimulus, and the lock is cleared as soon as it ends.
    """

    name = 'avoid'

    def __init__(self, detector, turn_speed=1.0, hysteresis=0.0):
        """``turn_speed``: turn rate (rad/s); ``hysteresis``: release margin (m)."""
        self.detector = detector
        self.turn_speed = turn_speed
        self.hysteresis = hysteresis
        self.direction = 0                  # +1 = turning left, -1 = right, 0 = idle

    def update(self, state):
        """Turn away from the closer side while the stimulus lasts."""
        obstacles = state.obstacles
        if not state.scan_is_fresh():
            self.direction = 0
            return None
        if self.direction != 0:
            if obstacles.nearest() > self.detector.threshold + self.hysteresis:
                self.direction = 0          # clear again: release the reflex
                return None
            return VelocityCommand(0.0, self.direction * self.turn_speed)
        if not self.detector.is_asymmetric(obstacles):
            return None
        # Closer on the left: turn right (-1); closer on the right: turn left (+1).
        self.direction = -1 if obstacles.left < obstacles.right else 1
        return VelocityCommand(0.0, self.direction * self.turn_speed)

    def suppress(self):
        """Forget the locked direction when a higher-priority behavior takes over."""
        self.direction = 0


class RandomTurnBehavior(Behavior):
    """Priority 5: after each 1 ft of forward travel, turn by U(-15, 15) deg."""

    name = 'random_turn'

    def __init__(self, turner, rng=None, step_distance=FEET_TO_METERS,
                 max_angle=math.radians(15.0)):
        """Configure the travel distance (m) and the largest turn angle (rad)."""
        self.turner = turner
        self.rng = rng if rng is not None else random.Random()
        self.step_distance = step_distance
        self.max_angle = max_angle
        self.odometer_at_last_turn = None

    def update(self, state):
        """Start a turn every ``step_distance`` meters, then finish it."""
        odometer = state.pose.odometer
        yaw = state.pose.yaw
        if self.odometer_at_last_turn is None:
            self.odometer_at_last_turn = odometer
        if self.turner.active:
            return self.turner.step(yaw, state.now)
        if odometer - self.odometer_at_last_turn >= self.step_distance:
            self.odometer_at_last_turn = odometer
            angle = self.rng.uniform(-self.max_angle, self.max_angle)
            self.turner.start(yaw + angle, yaw, state.now)
            return self.turner.step(yaw, state.now)
        return None

    def suppress(self):
        """Abandon a turn in progress if a higher-priority behavior takes over."""
        self.turner.cancel()


class DriveForwardBehavior(Behavior):
    """Priority 6: drive straight ahead (only while the lidar is working)."""

    name = 'drive_forward'

    def __init__(self, speed=0.2):
        """``speed``: forward speed in m/s."""
        self.speed = speed

    def update(self, state):
        """Request forward motion, unless the lidar has gone quiet."""
        if not state.scan_is_fresh():
            return None
        return VelocityCommand(self.speed, 0.0)


def build_behaviors(params, rng=None):
    """
    Create the six behaviors in priority order.

    ``params`` is a dict of settings (see ``DEFAULT_PARAMS``); missing keys
    use the defaults. Returns (behaviors, detector).
    """
    p = dict(DEFAULT_PARAMS)
    p.update(params)
    rng = rng if rng is not None else random.Random()
    detector = ObstacleDetector(
        mount=p['mount'],
        robot_radius=p['robot_radius'],
        front_half_angle=math.radians(p['front_half_angle_deg']),
        center_band=math.radians(p['center_band_deg']),
        threshold=p['obstacle_distance'],
        symmetry_tolerance=p['symmetry_tolerance'],
        self_filter_margin=p['self_filter_margin'],
        outlier_beams=p['outlier_beams'])

    def make_turner():
        return TurnToHeading(max_speed=p['turn_speed'], min_speed=p['min_turn_speed'],
                             tolerance=math.radians(p['turn_tolerance_deg']),
                             timeout_factor=p['turn_timeout_factor'])

    behaviors = [
        HaltBehavior(hold_time=p['halt_hold_time']),
        TeleopBehavior(timeout=p['teleop_timeout']),
        EscapeBehavior(detector, make_turner(), rng,
                       turn_angle=math.radians(p['escape_turn_deg']),
                       jitter=math.radians(p['escape_jitter_deg'])),
        AvoidBehavior(detector, turn_speed=p['avoid_turn_speed'],
                      hysteresis=p['avoid_hysteresis']),
        RandomTurnBehavior(make_turner(), rng,
                           step_distance=p['random_turn_step'],
                           max_angle=math.radians(p['random_turn_max_deg'])),
        DriveForwardBehavior(speed=p['forward_speed']),
    ]
    return behaviors, detector


DEFAULT_PARAMS = {
    'mount': None,                      # LidarMount; None = TurtleBot 4 default
    'robot_radius': 0.17,               # m
    'front_half_angle_deg': 60.0,       # width of each front sector
    'center_band_deg': 5.0,             # dead-ahead band counted on both sides
    'obstacle_distance': FEET_TO_METERS,
    'symmetry_tolerance': 0.05,         # m
    'halt_hold_time': 1.0,              # s
    'teleop_timeout': 1.0,              # s
    'escape_turn_deg': 180.0,
    'escape_jitter_deg': 30.0,
    'random_turn_step': FEET_TO_METERS,
    'random_turn_max_deg': 15.0,
    'turn_speed': 1.0,                  # rad/s, fastest escape / random turn
    'forward_speed': 0.2,               # m/s
    # Added for the physical robot (Project 2).
    'self_filter_margin': 0.0,          # m; ignore returns inside the robot outline
    'outlier_beams': 0,                 # smallest readings ignored per sector
    'avoid_turn_speed': 1.0,            # rad/s
    'avoid_hysteresis': 0.0,            # m; avoid releases at threshold + this
    'min_turn_speed': 0.3,              # rad/s; slowest end-of-turn speed
    'turn_tolerance_deg': 3.0,          # a turn ends within this many degrees
    'turn_timeout_factor': 3.0,         # x nominal turn time; 0 = no limit
}
