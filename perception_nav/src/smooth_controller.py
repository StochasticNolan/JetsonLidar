"""
Tesla-Style Smooth Controller

High-performance path following with:
- 100Hz control loop (separate thread)
- Cubic spline path fitting
- Dynamic look-ahead based on speed
- Smooth velocity profiling (jerk-limited)
- Predictive MPC-lite control
- Path interpolation between perception updates

Usage:
    from smooth_controller import SmoothController

    controller = SmoothController()
    controller.start()  # Starts 100Hz control thread

    # From perception loop (10-30Hz):
    controller.update_path(poles)  # Updates path buffer

    # Controller automatically sends smooth commands to guidance
"""

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

try:
    from scipy.interpolate import CubicSpline
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("Warning: scipy not available, using linear interpolation")


# =============================================================================
# Configuration
# =============================================================================

@dataclass
class SmoothControllerConfig:
    """Configuration for Tesla-style smooth controller."""

    # Control loop
    control_rate_hz: float = 100.0      # High-frequency control loop
    perception_timeout: float = 0.5     # Max age of perception data before hold

    # Path following
    lateral_offset: float = 10.0        # Desired offset from pole line (m)
    fly_on_left: bool = True            # Which side to fly

    # Dynamic look-ahead (look_ahead = base + speed * gain)
    look_ahead_base: float = 3.0        # Minimum look-ahead (m)
    look_ahead_gain: float = 1.5        # Seconds ahead to look
    look_ahead_max: float = 30.0        # Maximum look-ahead (m)

    # Velocity limits
    max_speed: float = 5.0              # Max forward speed (m/s)
    cruise_speed: float = 3.0           # Normal cruise speed (m/s)
    min_speed: float = 0.5              # Minimum speed (m/s)

    # Acceleration limits (for smooth motion)
    max_accel: float = 1.0              # Max acceleration (m/s^2)
    max_decel: float = 1.5              # Max deceleration (m/s^2)
    max_jerk: float = 2.0               # Max jerk for smoothness (m/s^3)

    # Lateral control
    max_lateral_accel: float = 1.0      # Max lateral acceleration (m/s^2)
    max_yaw_rate: float = 0.5           # Max yaw rate (rad/s)
    max_yaw_accel: float = 0.5          # Max yaw acceleration (rad/s^2)

    # Controller gains
    lateral_p: float = 0.8              # Proportional gain
    lateral_d: float = 0.3              # Derivative gain
    heading_p: float = 1.2              # Heading proportional
    heading_d: float = 0.4              # Heading derivative

    # Predictive control (MPC-lite)
    prediction_horizon: float = 1.0     # How far ahead to predict (seconds)
    prediction_steps: int = 10          # Steps in prediction

    # Path smoothing
    spline_smoothing: float = 0.5       # Spline smoothing factor
    path_history_size: int = 20         # Poles to keep in history

    # Curvature-based speed
    curvature_speed_gain: float = 2.0   # Slow down in curves
    min_curve_radius: float = 5.0       # Minimum curve radius for full speed

    # Obstacle avoidance
    obstacle_stop_distance: float = 3.0      # Full stop distance (m)
    obstacle_slow_distance: float = 10.0     # Start slowing distance (m)
    obstacle_min_speed: float = 0.3          # Min speed when obstacle detected


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class PathPoint:
    """A point on the planned path."""
    x: float              # Forward position (m)
    y: float              # Lateral position (m)
    heading: float        # Path heading (rad)
    curvature: float      # Path curvature (1/m)
    speed_limit: float    # Recommended speed at this point


@dataclass
class ControlState:
    """Current control state."""
    # Position (estimated)
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0

    # Velocity
    speed: float = 0.0
    lateral_vel: float = 0.0
    yaw_rate: float = 0.0

    # Acceleration (for jerk limiting)
    accel: float = 0.0
    lateral_accel: float = 0.0
    yaw_accel: float = 0.0

    # Errors
    lateral_error: float = 0.0
    heading_error: float = 0.0

    # Path info
    path_valid: bool = False
    distance_to_next_pole: float = float('inf')


@dataclass
class VelocityCommand:
    """Smooth velocity command."""
    vx: float = 0.0           # Forward (m/s)
    vy: float = 0.0           # Lateral (m/s)
    vz: float = 0.0           # Vertical (m/s)
    yaw_rate: float = 0.0     # Yaw rate (rad/s)


# =============================================================================
# Spline Path Planner
# =============================================================================

class SplinePathPlanner:
    """Fits smooth cubic splines through pole positions."""

    def __init__(self, config: SmoothControllerConfig):
        self.config = config
        self._pole_history: deque = deque(maxlen=config.path_history_size)
        self._spline_x: Optional[CubicSpline] = None
        self._spline_y: Optional[CubicSpline] = None
        self._path_length: float = 0.0
        self._last_update = 0.0

    def update(self, poles: List) -> bool:
        """Update path with new pole detections.

        Args:
            poles: List of FusedPole or similar with x, y attributes

        Returns:
            True if path was updated successfully
        """
        if not poles:
            return False

        # Extract pole positions and sort by X (forward distance)
        pole_positions = [(p.x, p.y) for p in poles if p.x > 0]
        if len(pole_positions) < 2:
            return False

        pole_positions.sort(key=lambda p: p[0])

        # Add to history (merge with existing, remove duplicates)
        for pos in pole_positions:
            # Check if this pole is already in history (within 2m)
            is_new = True
            for existing in self._pole_history:
                dist = math.sqrt((pos[0] - existing[0])**2 + (pos[1] - existing[1])**2)
                if dist < 2.0:
                    is_new = False
                    break
            if is_new:
                self._pole_history.append(pos)

        # Remove poles behind us (negative X)
        self._pole_history = deque(
            [p for p in self._pole_history if p[0] > -5.0],
            maxlen=self.config.path_history_size
        )

        # Need at least 3 points for spline
        if len(self._pole_history) < 3:
            return self._fit_line(pole_positions)

        return self._fit_spline()

    def _fit_line(self, poles: List[Tuple[float, float]]) -> bool:
        """Fall back to linear fit when not enough points for spline."""
        if len(poles) < 2:
            return False

        # Simple line fit
        xs = np.array([p[0] for p in poles])
        ys = np.array([p[1] for p in poles])

        # Offset for desired lateral position
        offset = self.config.lateral_offset
        if not self.config.fly_on_left:
            offset = -offset

        # Adjust Y to be our desired path (offset from poles)
        ys_path = ys + offset

        self._path_points = list(zip(xs, ys_path))
        self._spline_x = None
        self._spline_y = None
        self._path_length = max(xs) if len(xs) > 0 else 0
        self._last_update = time.time()

        return True

    def _fit_spline(self) -> bool:
        """Fit cubic spline through pole history."""
        if not SCIPY_AVAILABLE:
            return self._fit_line(list(self._pole_history))

        try:
            # Sort by X
            poles = sorted(self._pole_history, key=lambda p: p[0])

            # Parameterize by arc length (approximate with cumulative distance)
            xs = np.array([p[0] for p in poles])
            ys = np.array([p[1] for p in poles])

            # Apply lateral offset for desired flight path
            offset = self.config.lateral_offset
            if not self.config.fly_on_left:
                offset = -offset
            ys_path = ys + offset

            # Cumulative arc length parameter
            dx = np.diff(xs)
            dy = np.diff(ys_path)
            ds = np.sqrt(dx**2 + dy**2)
            s = np.concatenate([[0], np.cumsum(ds)])

            self._path_length = s[-1]

            # Fit splines
            self._spline_x = CubicSpline(s, xs, bc_type='natural')
            self._spline_y = CubicSpline(s, ys_path, bc_type='natural')
            self._s_param = s
            self._last_update = time.time()

            return True

        except Exception as e:
            print(f"Spline fit error: {e}")
            return False

    def get_path_point(self, distance: float) -> Optional[PathPoint]:
        """Get path point at given distance along path.

        Args:
            distance: Distance along path from start (m)

        Returns:
            PathPoint at that distance, or None if invalid
        """
        if self._spline_x is None or self._spline_y is None:
            return None

        # Clamp distance to path length
        s = max(0, min(distance, self._path_length))

        try:
            # Position
            x = float(self._spline_x(s))
            y = float(self._spline_y(s))

            # Derivatives for heading and curvature
            dx = float(self._spline_x(s, 1))  # First derivative
            dy = float(self._spline_y(s, 1))
            ddx = float(self._spline_x(s, 2))  # Second derivative
            ddy = float(self._spline_y(s, 2))

            # Heading
            heading = math.atan2(dy, dx)

            # Curvature: kappa = (dx * ddy - dy * ddx) / (dx^2 + dy^2)^(3/2)
            denom = (dx**2 + dy**2)**1.5
            if denom > 1e-6:
                curvature = (dx * ddy - dy * ddx) / denom
            else:
                curvature = 0.0

            # Speed limit based on curvature
            if abs(curvature) > 1e-6:
                # v = sqrt(a_lat / kappa) - max lateral accel limits speed in curves
                curve_radius = 1.0 / abs(curvature)
                speed_from_curve = math.sqrt(self.config.max_lateral_accel * curve_radius)
                speed_limit = min(self.config.max_speed, speed_from_curve)
            else:
                speed_limit = self.config.max_speed

            return PathPoint(
                x=x, y=y,
                heading=heading,
                curvature=curvature,
                speed_limit=speed_limit
            )

        except Exception:
            return None

    def get_closest_point_on_path(self, x: float, y: float,
                                    hint_s: float = None) -> Tuple[float, float, float]:
        """Find closest point on path to given position.

        OPTIMIZED: Uses hint_s from previous call to search locally (5x faster).

        Args:
            x, y: Current position
            hint_s: Previous s parameter (search locally around this value)

        Returns:
            (distance_along_path, lateral_error, heading_at_point)
        """
        if self._spline_x is None:
            return 0.0, 0.0, 0.0

        # OPTIMIZED: If we have a hint, search locally instead of full path
        if hint_s is not None and self._path_length > 0:
            # Search +/- 3m around previous position (10 samples vs 50)
            search_radius = 3.0
            s_min = max(0, hint_s - search_radius)
            s_max = min(self._path_length, hint_s + search_radius)
            n_samples = 10
        else:
            s_min = 0
            s_max = self._path_length
            n_samples = 50

        s_samples = np.linspace(s_min, s_max, n_samples)

        min_dist = float('inf')
        best_s = hint_s if hint_s is not None else 0.0

        for s in s_samples:
            px = float(self._spline_x(s))
            py = float(self._spline_y(s))
            dist = (x - px)**2 + (y - py)**2  # Squared dist (no sqrt needed)
            if dist < min_dist:
                min_dist = dist
                best_s = s

        # Get path point
        point = self.get_path_point(best_s)
        if point is None:
            return best_s, 0.0, 0.0

        # Lateral error (signed)
        dx = x - point.x
        dy = y - point.y
        # Cross product to get signed distance
        lateral_error = -dx * math.sin(point.heading) + dy * math.cos(point.heading)

        return best_s, lateral_error, point.heading

    def is_valid(self) -> bool:
        """Check if path is valid and recent."""
        if self._spline_x is None:
            return False
        age = time.time() - self._last_update
        return age < 2.0  # Path valid for 2 seconds

    def get_path_length(self) -> float:
        """Get total path length."""
        return self._path_length


# =============================================================================
# Velocity Profiler
# =============================================================================

class VelocityProfiler:
    """Generates smooth, jerk-limited velocity profiles."""

    def __init__(self, config: SmoothControllerConfig):
        self.config = config
        self._target_speed = 0.0
        self._current_speed = 0.0
        self._current_accel = 0.0

    def set_target_speed(self, speed: float):
        """Set target speed (will be reached smoothly)."""
        self._target_speed = max(0, min(speed, self.config.max_speed))

    def update(self, dt: float) -> float:
        """Update velocity profile for one time step.

        Args:
            dt: Time step (seconds)

        Returns:
            Current commanded speed
        """
        if dt <= 0:
            return self._current_speed

        # Compute desired acceleration to reach target
        speed_error = self._target_speed - self._current_speed

        if abs(speed_error) < 0.01:
            # Close enough
            self._current_speed = self._target_speed
            self._current_accel = 0.0
            return self._current_speed

        # Desired acceleration
        if speed_error > 0:
            desired_accel = min(self.config.max_accel, speed_error / dt)
        else:
            desired_accel = max(-self.config.max_decel, speed_error / dt)

        # Jerk limiting - limit rate of change of acceleration
        accel_change = desired_accel - self._current_accel
        max_accel_change = self.config.max_jerk * dt

        if accel_change > max_accel_change:
            self._current_accel += max_accel_change
        elif accel_change < -max_accel_change:
            self._current_accel -= max_accel_change
        else:
            self._current_accel = desired_accel

        # Update speed
        self._current_speed += self._current_accel * dt
        self._current_speed = max(0, min(self._current_speed, self.config.max_speed))

        return self._current_speed

    def get_current_speed(self) -> float:
        return self._current_speed

    def get_current_accel(self) -> float:
        return self._current_accel

    def emergency_stop(self):
        """Initiate emergency stop."""
        self._target_speed = 0.0


# =============================================================================
# MPC-Lite Predictive Controller
# =============================================================================

class PredictiveController:
    """Simple predictive controller (MPC-lite).

    Looks ahead and pre-corrects for upcoming path changes.
    """

    def __init__(self, config: SmoothControllerConfig):
        self.config = config
        self._path_planner: Optional[SplinePathPlanner] = None

    def set_path_planner(self, planner: SplinePathPlanner):
        """Set path planner for look-ahead."""
        self._path_planner = planner

    def compute_correction(self, current_s: float, speed: float,
                           lateral_error: float, heading_error: float) -> Tuple[float, float]:
        """Compute predictive correction.

        Looks ahead on the path and anticipates needed corrections.

        Args:
            current_s: Current position along path
            speed: Current forward speed
            lateral_error: Current lateral error
            heading_error: Current heading error

        Returns:
            (lateral_correction, yaw_correction) to add to base control
        """
        if self._path_planner is None or speed < 0.1:
            return 0.0, 0.0

        # Prediction horizon
        horizon = self.config.prediction_horizon
        steps = self.config.prediction_steps
        dt = horizon / steps

        lat_correction = 0.0
        yaw_correction = 0.0

        # Simulate forward
        s = current_s
        predicted_lat = lateral_error
        predicted_head = heading_error

        for i in range(steps):
            # Move along path
            s += speed * dt

            # Get path curvature at future point
            point = self._path_planner.get_path_point(s)
            if point is None:
                break

            # Anticipate lateral error from curvature
            # If path curves left (positive curvature), we'll drift right
            predicted_lat += point.curvature * speed * dt * speed * dt * 0.5

            # Weight by how soon (discount factor)
            weight = 1.0 - (i / steps) * 0.5

            # Accumulate corrections
            lat_correction += predicted_lat * weight * 0.1
            yaw_correction += point.curvature * speed * weight * 0.2

        return lat_correction, yaw_correction


# =============================================================================
# Main Smooth Controller
# =============================================================================

class SmoothController:
    """Tesla-style smooth controller with 100Hz control loop."""

    def __init__(self, config: Optional[SmoothControllerConfig] = None,
                 guidance=None):
        """Initialize smooth controller.

        Args:
            config: SmoothControllerConfig
            guidance: MavlinkGuidance instance for sending commands
        """
        self.config = config or SmoothControllerConfig()
        self.guidance = guidance

        # Components
        self.path_planner = SplinePathPlanner(self.config)
        self.velocity_profiler = VelocityProfiler(self.config)
        self.predictive_controller = PredictiveController(self.config)
        self.predictive_controller.set_path_planner(self.path_planner)

        # State
        self.state = ControlState()
        self._last_perception_time = 0.0
        self._current_s = 0.0  # Position along path

        # Control thread
        self._running = False
        self._control_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Command output (for visualization)
        self._last_command = VelocityCommand()

        # PD state
        self._prev_lateral_error = 0.0
        self._prev_heading_error = 0.0
        self._prev_time = time.time()

        # Obstacle state (from capture_system.ObstacleState)
        self._obstacle_detected = False
        self._obstacle_distance = float('inf')
        self._obstacle_timestamp = 0.0

    def set_guidance(self, guidance):
        """Set guidance module for sending commands."""
        self.guidance = guidance

    def update_obstacle_state(self, detected: bool, distance: float):
        """Update obstacle state from obstacle detector.

        Call this from the perception loop when obstacle detection runs.

        Args:
            detected: Whether obstacle is detected
            distance: Distance to obstacle (m)
        """
        with self._lock:
            self._obstacle_detected = detected
            self._obstacle_distance = distance
            self._obstacle_timestamp = time.time()

    def start(self):
        """Start the 100Hz control thread."""
        if self._running:
            return

        self._running = True
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()
        print(f"Smooth controller started at {self.config.control_rate_hz}Hz")

    def stop(self):
        """Stop the control thread."""
        self._running = False
        if self._control_thread:
            self._control_thread.join(timeout=1.0)
        print("Smooth controller stopped")

    def update_path(self, poles: List) -> bool:
        """Update path from perception (called at 10-30Hz).

        Args:
            poles: List of FusedPole objects

        Returns:
            True if path updated successfully
        """
        with self._lock:
            success = self.path_planner.update(poles)
            if success:
                self._last_perception_time = time.time()

                # Set target speed based on path quality
                if self.path_planner.is_valid():
                    self.velocity_profiler.set_target_speed(self.config.cruise_speed)
                else:
                    self.velocity_profiler.set_target_speed(self.config.min_speed)

            return success

    def _control_loop(self):
        """Main 100Hz control loop (runs in separate thread)."""
        dt = 1.0 / self.config.control_rate_hz
        last_time = time.time()
        error_count = 0
        max_errors = 10  # Stop after too many consecutive errors

        while self._running:
            try:
                loop_start = time.time()

                # Compute actual dt
                actual_dt = loop_start - last_time
                last_time = loop_start

                # Run control
                with self._lock:
                    cmd = self._compute_control(actual_dt)

                # Send to guidance
                if self.guidance is not None:
                    self.guidance.send_velocity(cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate)

                self._last_command = cmd
                error_count = 0  # Reset on success

                # Sleep for remainder of period
                elapsed = time.time() - loop_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

            except Exception as e:
                error_count += 1
                if error_count == 1:
                    print(f"Smooth controller error: {e}")
                if error_count >= max_errors:
                    print(f"Too many errors ({max_errors}), stopping control loop")
                    self._running = False
                    break
                time.sleep(dt)  # Still maintain timing

    def _compute_control(self, dt: float) -> VelocityCommand:
        """Compute smooth velocity command.

        Args:
            dt: Time since last control update

        Returns:
            VelocityCommand
        """
        # Check for obstacles FIRST (highest priority)
        obstacle_speed_limit = self.config.max_speed
        if self._obstacle_detected:
            obstacle_age = time.time() - self._obstacle_timestamp
            if obstacle_age < 0.5:  # Fresh obstacle data
                dist = self._obstacle_distance

                if dist <= self.config.obstacle_stop_distance:
                    # Emergency stop
                    self.velocity_profiler.emergency_stop()
                    return VelocityCommand(vx=0, vy=0, vz=0, yaw_rate=0)

                elif dist < self.config.obstacle_slow_distance:
                    # Proportional slowdown
                    # Speed scales linearly from min_speed at stop_dist to cruise at slow_dist
                    ratio = (dist - self.config.obstacle_stop_distance) / \
                            (self.config.obstacle_slow_distance - self.config.obstacle_stop_distance)
                    obstacle_speed_limit = (
                        self.config.obstacle_min_speed +
                        ratio * (self.config.cruise_speed - self.config.obstacle_min_speed)
                    )
                    self.velocity_profiler.set_target_speed(obstacle_speed_limit)

        # Check perception freshness
        perception_age = time.time() - self._last_perception_time
        if perception_age > self.config.perception_timeout:
            # Perception stale - slow down
            self.velocity_profiler.set_target_speed(0.0)
            self.state.path_valid = False

        # Get current speed
        speed = self.velocity_profiler.update(dt)
        self.state.speed = speed

        # Dynamic look-ahead
        look_ahead = self.config.look_ahead_base + speed * self.config.look_ahead_gain
        look_ahead = min(look_ahead, self.config.look_ahead_max)

        # Get path errors
        if not self.path_planner.is_valid():
            self.state.path_valid = False
            return VelocityCommand(vx=speed, vy=0, vz=0, yaw_rate=0)

        self.state.path_valid = True

        # Find closest point on path and errors
        # OPTIMIZED: Pass hint from previous iteration for 5x faster local search
        s, lateral_error, path_heading = self.path_planner.get_closest_point_on_path(
            self.state.x, self.state.y, hint_s=self._current_s
        )
        self._current_s = s

        # Get look-ahead point for pure pursuit
        look_ahead_point = self.path_planner.get_path_point(s + look_ahead)
        if look_ahead_point is None:
            look_ahead_point = self.path_planner.get_path_point(s)

        if look_ahead_point is None:
            return VelocityCommand(vx=speed, vy=0, vz=0, yaw_rate=0)

        # Heading error
        heading_error = path_heading - self.state.heading
        # Normalize to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        self.state.lateral_error = lateral_error
        self.state.heading_error = heading_error

        # Compute derivatives for PD control
        d_lateral = (lateral_error - self._prev_lateral_error) / dt if dt > 0 else 0
        d_heading = (heading_error - self._prev_heading_error) / dt if dt > 0 else 0
        self._prev_lateral_error = lateral_error
        self._prev_heading_error = heading_error

        # Base PD control
        lateral_cmd = -(self.config.lateral_p * lateral_error +
                        self.config.lateral_d * d_lateral)
        yaw_cmd = -(self.config.heading_p * heading_error +
                    self.config.heading_d * d_heading)

        # Add predictive corrections
        pred_lat, pred_yaw = self.predictive_controller.compute_correction(
            s, speed, lateral_error, heading_error
        )
        lateral_cmd += pred_lat
        yaw_cmd += pred_yaw

        # Pure pursuit curvature (blend with PD)
        if look_ahead_point is not None and speed > 0.1:
            # Curvature = 2 * lateral_error_to_lookahead / look_ahead^2
            look_lat = look_ahead_point.y - self.state.y
            curvature = 2 * look_lat / (look_ahead * look_ahead)
            pure_pursuit_yaw = speed * curvature

            # Blend: more pure pursuit at high speed, more PD at low speed
            blend = min(1.0, speed / self.config.cruise_speed)
            yaw_cmd = (1 - blend * 0.5) * yaw_cmd + blend * 0.5 * pure_pursuit_yaw

        # Speed adjustment for curves
        if look_ahead_point.speed_limit < speed:
            self.velocity_profiler.set_target_speed(look_ahead_point.speed_limit)

        # Apply limits with smoothing
        lateral_cmd = self._smooth_limit(
            lateral_cmd, self.state.lateral_vel,
            self.config.max_lateral_accel * dt,
            self.config.max_lateral_accel
        )
        self.state.lateral_vel = lateral_cmd

        yaw_cmd = self._smooth_limit(
            yaw_cmd, self.state.yaw_rate,
            self.config.max_yaw_accel * dt,
            self.config.max_yaw_rate
        )
        self.state.yaw_rate = yaw_cmd

        return VelocityCommand(
            vx=speed,
            vy=lateral_cmd,
            vz=0.0,
            yaw_rate=yaw_cmd
        )

    def _smooth_limit(self, target: float, current: float,
                      max_change: float, max_value: float) -> float:
        """Apply smooth rate limiting.

        Args:
            target: Desired value
            current: Current value
            max_change: Max change per step
            max_value: Absolute max value

        Returns:
            Smoothly limited value
        """
        # Rate limit
        change = target - current
        if change > max_change:
            target = current + max_change
        elif change < -max_change:
            target = current - max_change

        # Absolute limit
        return max(-max_value, min(max_value, target))

    def get_state(self) -> ControlState:
        """Get current control state."""
        return self.state

    def get_last_command(self) -> VelocityCommand:
        """Get last velocity command sent."""
        return self._last_command

    def emergency_stop(self):
        """Trigger emergency stop."""
        self.velocity_profiler.emergency_stop()

    def is_running(self) -> bool:
        """Check if controller is running."""
        return self._running


# =============================================================================
# Convenience
# =============================================================================

def create_smooth_controller(guidance=None,
                             config_path: str = None) -> SmoothController:
    """Create a SmoothController with optional config."""
    config = SmoothControllerConfig()

    # Load from YAML if provided
    if config_path:
        try:
            import yaml
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)

            if 'smooth_controller' in data:
                sc = data['smooth_controller']
                for key, value in sc.items():
                    if hasattr(config, key):
                        setattr(config, key, value)
        except Exception as e:
            print(f"Warning: Could not load smooth controller config: {e}")

    return SmoothController(config, guidance)


# =============================================================================
# Test
# =============================================================================

if __name__ == '__main__':
    import time
    from dataclasses import dataclass

    @dataclass
    class MockPole:
        x: float
        y: float
        fused_confidence: float = 0.8

    # Create test poles (line of poles going forward)
    test_poles = [
        MockPole(20.0, -2.0),
        MockPole(70.0, -1.5),
        MockPole(120.0, -2.5),
        MockPole(170.0, -1.0),
        MockPole(220.0, -2.0),
    ]

    print("Testing Tesla-Style Smooth Controller")
    print("=" * 60)

    # Create controller
    controller = SmoothController()

    # Update path
    success = controller.update_path(test_poles)
    print(f"Path updated: {success}")
    print(f"Path length: {controller.path_planner.get_path_length():.1f}m")

    # Simulate control loop
    print("\nSimulating 2 seconds of control at 100Hz:")
    controller.start()

    for i in range(20):  # 20 updates at "10Hz"
        time.sleep(0.1)
        state = controller.get_state()
        cmd = controller.get_last_command()
        print(f"  Speed: {state.speed:.2f} m/s | "
              f"Lat err: {state.lateral_error:+.2f}m | "
              f"Cmd: vx={cmd.vx:.2f} vy={cmd.vy:.2f} yaw={cmd.yaw_rate:.3f}")

    controller.stop()
    print("\nDone.")
