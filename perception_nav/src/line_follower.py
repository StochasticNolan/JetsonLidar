"""
Line Follower Module

Follows a single line of utility poles (H-frame, transmission poles, etc).
Fits a path ALONG the poles and maintains a lateral offset.

This is different from corridor_follower.py which assumes two parallel rows.

Features:
- Line fitting through pole positions
- Configurable lateral offset (fly left or right of poles)
- Heading alignment with pole line direction
- Look-ahead for smooth path following
- Next-pole targeting for capture scheduling

Usage:
    from line_follower import LineFollower, LineFollowerConfig
    from guidance import CorridorState  # Reuses same output format

    follower = LineFollower(config)
    state = follower.update(fused_poles)
"""

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class LineState:
    """Output state for guidance system (compatible with CorridorState interface)."""
    lateral_error: float      # Distance from desired offset line (m), + = too far right
    heading_error: float      # Angle difference from line direction (rad), + = pointing right
    look_ahead_x: float       # Look-ahead point X in body frame (m)
    look_ahead_y: float       # Look-ahead point Y in body frame (m)
    confidence: float         # Detection confidence (0-1)
    num_poles: int            # Number of detected poles
    line_heading: float       # Heading of the pole line (rad, 0 = forward)
    distance_to_next: float   # Distance to next pole along line (m)
    next_pole_id: int         # ID of next pole (for capture trigger)
    valid: bool               # Whether state is valid for navigation

    # Aliases for compatibility with CorridorState
    @property
    def num_pairs(self) -> int:
        return 0  # Not applicable for line following

    @property
    def corridor_width(self) -> float:
        return 0.0  # Not applicable

    @property
    def centerline_angle(self) -> float:
        return self.line_heading


@dataclass
class LineFollowerConfig:
    """Configuration for line follower."""

    # Lateral offset - which side to fly on and how far
    lateral_offset: float = 5.0        # Desired offset from pole line (m)
    fly_on_left: bool = True           # True = fly left of poles, False = right

    # Line fitting
    min_poles_for_line: int = 2        # Min poles to fit a line
    max_pole_range: float = 100.0      # Max range to consider poles (m)
    smoothing_alpha: float = 0.3       # EMA smoothing (0=none, 1=max)

    # Look-ahead
    look_ahead_distance: float = 10.0  # Look-ahead distance (m)
    min_look_ahead: float = 3.0        # Minimum look-ahead (m)

    # Pole spacing (typical for utility poles)
    expected_pole_spacing: float = 50.0  # Expected distance between poles (m)
    max_pole_gap: float = 150.0          # Max gap before assuming missing pole

    # Navigation
    approach_distance: float = 10.0    # Start slowing when this close to pole (m)
    capture_distance: float = 5.0      # Distance at which to trigger capture (m)

    # Confidence
    min_confidence: float = 0.3        # Min confidence for valid state


# =============================================================================
# Line Follower
# =============================================================================

class LineFollower:
    """Follows a single line of utility poles."""

    def __init__(self, config: Optional[LineFollowerConfig] = None,
                 config_path: Optional[str] = None):
        """Initialize line follower.

        Args:
            config: LineFollowerConfig object
            config_path: Path to YAML config file
        """
        if config is not None:
            self.config = config
        elif config_path is not None:
            self.config = self._load_config(config_path)
        else:
            self.config = LineFollowerConfig()

        # State history
        self._line_history: deque = deque(maxlen=10)
        self._smoothed_heading: Optional[float] = None
        self._smoothed_offset: Optional[float] = None

        # Pole tracking
        self._last_poles: List = []
        self._visited_poles: set = set()  # Poles we've passed
        self._next_pole_id: int = -1

        # Last valid state
        self._last_valid_state: Optional[LineState] = None
        self._last_update = time.time()

        # Timing
        self._timing = {'fitting': 0.0, 'total': 0.0}

    def _load_config(self, config_path: str) -> LineFollowerConfig:
        """Load configuration from YAML file."""
        config = LineFollowerConfig()

        if not YAML_AVAILABLE:
            return config

        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)

            if 'line_follower' in data:
                lf = data['line_follower']
                config.lateral_offset = lf.get('lateral_offset', config.lateral_offset)
                config.fly_on_left = lf.get('fly_on_left', config.fly_on_left)
                config.look_ahead_distance = lf.get('look_ahead_distance', config.look_ahead_distance)
                config.expected_pole_spacing = lf.get('expected_pole_spacing', config.expected_pole_spacing)
                config.smoothing_alpha = lf.get('smoothing_alpha', config.smoothing_alpha)
                config.approach_distance = lf.get('approach_distance', config.approach_distance)
                config.capture_distance = lf.get('capture_distance', config.capture_distance)

        except Exception as e:
            print(f"Warning: Could not load line follower config: {e}")

        return config

    def _get_pole_confidence(self, pole) -> float:
        """Get confidence from pole (handles both FusedPole and PoleDetection)."""
        if hasattr(pole, 'fused_confidence'):
            return pole.fused_confidence
        elif hasattr(pole, 'confidence'):
            return pole.confidence
        return 0.5  # Default

    def _fit_line(self, poles: List) -> Tuple[float, float, np.ndarray, float]:
        """Fit a line through pole positions.

        Uses least-squares to fit a line, then computes heading and offset.

        Args:
            poles: List of FusedPole or PoleDetection objects

        Returns:
            (heading, offset, direction_vector, fit_confidence)
            - heading: angle of line in body frame (rad, 0=forward)
            - offset: perpendicular distance from drone to line (m)
            - direction: unit vector along line (pointing forward generally)
            - confidence: fit quality (0-1)
        """
        if len(poles) < 2:
            # Single pole - assume line goes forward through it
            p = poles[0]
            heading = math.atan2(-p.y, p.x)
            offset = math.sqrt(p.x**2 + p.y**2) * math.sin(heading)
            return heading, offset, np.array([1.0, 0.0]), self._get_pole_confidence(p)

        # Get pole positions
        xs = np.array([p.x for p in poles])
        ys = np.array([p.y for p in poles])

        # Fit line using PCA (handles vertical lines better than least-squares)
        points = np.column_stack([xs, ys])
        centroid = np.mean(points, axis=0)
        centered = points - centroid

        # SVD to find principal direction
        _, _, Vt = np.linalg.svd(centered)
        direction = Vt[0]  # First principal component = line direction

        # Ensure direction points generally forward (positive X)
        if direction[0] < 0:
            direction = -direction

        # Line heading (angle from forward)
        heading = math.atan2(direction[1], direction[0])

        # Perpendicular distance from origin (drone) to line
        # Using point-to-line formula: |n · (p - centroid)| where n is normal
        normal = np.array([-direction[1], direction[0]])  # Perpendicular to line
        offset = np.dot(normal, -centroid)  # Distance to line (signed)

        # Fit quality - based on residuals
        residuals = np.abs(np.dot(centered, normal))
        rmse = np.sqrt(np.mean(residuals**2))
        confidence = max(0.0, 1.0 - rmse / 5.0)  # Scale: 0 at 5m RMSE
        confidence *= np.mean([self._get_pole_confidence(p) for p in poles])

        return heading, offset, direction, confidence

    def _find_next_pole(self, poles: List, direction: np.ndarray) -> Tuple[Optional[int], float]:
        """Find the next pole along the line direction.

        Args:
            poles: List of FusedPole objects
            direction: Unit vector of line direction

        Returns:
            (pole_id, distance) of next pole, or (None, inf) if none
        """
        if not poles:
            return None, float('inf')

        # Project pole positions onto line direction
        # Positive projection = pole is ahead
        next_pole = None
        min_positive_dist = float('inf')

        for pole in poles:
            if pole.id in self._visited_poles:
                continue

            # Project pole position onto direction
            pos = np.array([pole.x, pole.y])
            proj = np.dot(pos, direction)

            # If ahead of us and closer than current best
            if proj > 0 and proj < min_positive_dist:
                min_positive_dist = proj
                next_pole = pole

        if next_pole:
            return next_pole.id, min_positive_dist
        return None, float('inf')

    def _compute_errors(self, heading: float, offset: float,
                        direction: np.ndarray) -> Tuple[float, float]:
        """Compute lateral and heading errors for guidance.

        Args:
            heading: Line heading in body frame (rad)
            offset: Perpendicular distance to line (m)
            direction: Line direction vector

        Returns:
            (lateral_error, heading_error)
        """
        # Desired offset (positive = left of line)
        desired_offset = self.config.lateral_offset
        if not self.config.fly_on_left:
            desired_offset = -desired_offset

        # Lateral error: difference between actual and desired offset
        # offset > 0 means we're left of the line
        # If fly_on_left and offset < desired_offset, we need to go more left (negative error)
        lateral_error = desired_offset - offset

        # Heading error: align with line direction
        heading_error = -heading  # We want to match line heading

        return lateral_error, heading_error

    def _compute_look_ahead(self, heading: float, offset: float,
                            poles: List) -> Tuple[float, float]:
        """Compute look-ahead point.

        Args:
            heading: Line heading (rad)
            offset: Current offset from line (m)
            poles: Pole list for range reference

        Returns:
            (look_ahead_x, look_ahead_y) in body frame
        """
        # Look-ahead distance
        L = self.config.look_ahead_distance

        # Limit to furthest visible pole
        if poles:
            max_x = max(p.x for p in poles)
            L = min(L, max(self.config.min_look_ahead, max_x * 0.8))

        # Desired offset
        desired_offset = self.config.lateral_offset
        if not self.config.fly_on_left:
            desired_offset = -desired_offset

        # Point along desired path at distance L
        # Path is parallel to pole line at desired offset
        look_x = L * math.cos(heading)
        look_y = L * math.sin(heading) + desired_offset

        return look_x, look_y

    def _smooth(self, heading: float, offset: float,
                alpha: float) -> Tuple[float, float]:
        """Apply EMA smoothing."""
        if self._smoothed_heading is None:
            self._smoothed_heading = heading
            self._smoothed_offset = offset
        else:
            self._smoothed_heading = alpha * self._smoothed_heading + (1 - alpha) * heading
            self._smoothed_offset = alpha * self._smoothed_offset + (1 - alpha) * offset

        return self._smoothed_heading, self._smoothed_offset

    def mark_pole_visited(self, pole_id: int):
        """Mark a pole as visited (passed/captured).

        Call this when you've passed a pole or captured data at it.

        Args:
            pole_id: ID of pole to mark as visited
        """
        self._visited_poles.add(pole_id)

    def reset_visited(self):
        """Reset visited poles for a new run."""
        self._visited_poles.clear()

    def update(self, poles: List) -> LineState:
        """Update line follower with new pole detections.

        Main entry point. Call at perception frame rate (~10-30 Hz).

        Args:
            poles: List of FusedPole objects from sensor fusion

        Returns:
            LineState for guidance system
        """
        t_start = time.time()

        # Default invalid state
        invalid_state = LineState(
            lateral_error=0.0,
            heading_error=0.0,
            look_ahead_x=self.config.look_ahead_distance,
            look_ahead_y=0.0,
            confidence=0.0,
            num_poles=len(poles) if poles else 0,
            line_heading=0.0,
            distance_to_next=float('inf'),
            next_pole_id=-1,
            valid=False
        )

        # Need at least one pole
        if not poles:
            if self._last_valid_state and time.time() - self._last_update < 0.5:
                return self._last_valid_state
            return invalid_state

        # Filter poles by range
        poles = [p for p in poles if p.x > 0 and
                 math.sqrt(p.x**2 + p.y**2) < self.config.max_pole_range]

        if len(poles) < self.config.min_poles_for_line:
            # With single pole, we can still estimate
            if len(poles) == 1:
                pass  # Allow single pole
            else:
                return invalid_state

        self._last_poles = poles

        # Fit line through poles
        t0 = time.time()
        heading, offset, direction, fit_confidence = self._fit_line(poles)
        self._timing['fitting'] = time.time() - t0

        # Smooth estimates
        heading, offset = self._smooth(heading, offset, self.config.smoothing_alpha)

        # Compute navigation errors
        lateral_error, heading_error = self._compute_errors(heading, offset, direction)

        # Find next pole
        next_id, next_dist = self._find_next_pole(poles, direction)
        self._next_pole_id = next_id if next_id else -1

        # Compute look-ahead
        look_x, look_y = self._compute_look_ahead(heading, offset, poles)

        # Overall confidence
        pole_conf = np.mean([self._get_pole_confidence(p) for p in poles])
        confidence = fit_confidence * pole_conf
        if len(poles) >= 3:
            confidence = min(1.0, confidence + 0.2)

        # Build state
        state = LineState(
            lateral_error=lateral_error,
            heading_error=heading_error,
            look_ahead_x=look_x,
            look_ahead_y=look_y,
            confidence=confidence,
            num_poles=len(poles),
            line_heading=heading,
            distance_to_next=next_dist,
            next_pole_id=self._next_pole_id,
            valid=confidence >= self.config.min_confidence
        )

        if state.valid:
            self._last_valid_state = state
            self._last_update = time.time()

        self._timing['total'] = time.time() - t_start
        return state

    def should_capture(self, state: LineState) -> bool:
        """Check if we should trigger capture at current position.

        Args:
            state: Current LineState

        Returns:
            True if capture should be triggered
        """
        if state.next_pole_id < 0:
            return False
        return state.distance_to_next <= self.config.capture_distance

    def get_timing(self) -> dict:
        """Get timing breakdown."""
        return self._timing.copy()

    def reset(self):
        """Reset all state."""
        self._line_history.clear()
        self._smoothed_heading = None
        self._smoothed_offset = None
        self._last_poles.clear()
        self._visited_poles.clear()
        self._next_pole_id = -1
        self._last_valid_state = None


# =============================================================================
# Convenience
# =============================================================================

def create_line_follower(config_path: str = 'config/settings.yaml') -> LineFollower:
    """Create LineFollower with config from file."""
    return LineFollower(config_path=config_path)


# =============================================================================
# Test
# =============================================================================

if __name__ == '__main__':
    from dataclasses import dataclass

    @dataclass
    class MockPole:
        id: int
        x: float
        y: float
        z: float = 10.0
        fused_confidence: float = 0.8

    # Simulate poles in a line going forward, slightly to the right
    # Drone is offset to the left of the pole line
    test_poles = [
        MockPole(1, 20.0, -2.0),   # 20m ahead, 2m to the right
        MockPole(2, 70.0, -1.5),   # 70m ahead, 1.5m to the right
        MockPole(3, 120.0, -2.5),  # 120m ahead, 2.5m to the right
    ]

    print("Test: Poles in a line to the right, drone offset to the left")
    print("=" * 60)

    # Create follower - fly on left side of poles at 5m offset
    config = LineFollowerConfig(
        lateral_offset=5.0,
        fly_on_left=True
    )
    follower = LineFollower(config)

    state = follower.update(test_poles)

    print(f"Valid: {state.valid}")
    print(f"Num poles: {state.num_poles}")
    print(f"Line heading: {math.degrees(state.line_heading):.1f}°")
    print(f"Lateral error: {state.lateral_error:+.2f}m (+ = need to go right)")
    print(f"Heading error: {math.degrees(state.heading_error):+.1f}°")
    print(f"Look-ahead: ({state.look_ahead_x:.1f}, {state.look_ahead_y:.1f})")
    print(f"Next pole: ID={state.next_pole_id}, dist={state.distance_to_next:.1f}m")
    print(f"Confidence: {state.confidence:.2f}")

    print("\n" + "=" * 60)
    print("Test: Check capture trigger")
    print("=" * 60)

    # Simulate getting closer to pole
    close_poles = [
        MockPole(1, 4.0, -2.0),   # 4m ahead - within capture distance
        MockPole(2, 54.0, -1.5),
    ]
    state = follower.update(close_poles)
    print(f"Distance to next: {state.distance_to_next:.1f}m")
    print(f"Should capture: {follower.should_capture(state)}")
