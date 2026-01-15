"""
Corridor Follower Module

Converts fused pole detections into a corridor state for guidance.
Fits a centerline through pole pairs and computes tracking errors.

Features:
- Left/right pole pairing by lateral offset
- Centerline fitting through pole midpoints
- Lateral and heading error computation
- Look-ahead point for pure pursuit
- Temporal smoothing for stable tracking

Usage:
    from corridor_follower import CorridorFollower, CorridorConfig
    from guidance import CorridorState

    follower = CorridorFollower(config)
    corridor_state = follower.update(fused_poles)
"""

import math
import time
from collections import deque
from dataclasses import dataclass, field
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
class PolePair:
    """A matched pair of left/right poles defining corridor edges."""
    left_id: int              # ID of left pole
    right_id: int             # ID of right pole
    left_x: float             # Left pole X (forward)
    left_y: float             # Left pole Y (positive = left)
    right_x: float            # Right pole X
    right_y: float            # Right pole Y (negative = right)
    midpoint_x: float         # Centerline X
    midpoint_y: float         # Centerline Y
    width: float              # Corridor width at this pair
    confidence: float         # Combined confidence


@dataclass
class CorridorState:
    """Output state for guidance system."""
    lateral_error: float      # Signed distance from centerline (m), + = drone right of center
    heading_error: float      # Angle to centerline tangent (rad), + = pointing right of centerline
    look_ahead_x: float       # Look-ahead point X in body frame (m)
    look_ahead_y: float       # Look-ahead point Y in body frame (m)
    confidence: float         # Overall detection confidence (0-1)
    num_poles: int            # Number of detected poles
    num_pairs: int            # Number of matched pole pairs
    corridor_width: float     # Average corridor width (m)
    centerline_angle: float   # Centerline heading in body frame (rad)
    valid: bool               # Whether corridor state is valid for navigation


@dataclass
class CorridorConfig:
    """Configuration for corridor follower."""

    # Pole pairing
    max_pair_distance_x: float = 5.0      # Max X (forward) separation for pairing (m)
    min_corridor_width: float = 1.0       # Min corridor width (m)
    max_corridor_width: float = 10.0      # Max corridor width (m)
    min_poles_for_corridor: int = 2       # Min poles to estimate corridor

    # Centerline fitting
    use_ransac: bool = False              # Use RANSAC for line fitting (robust to outliers)
    ransac_threshold: float = 0.3         # RANSAC inlier threshold (m)
    smoothing_alpha: float = 0.3          # EMA smoothing for centerline (0=none, 1=max)

    # Look-ahead
    look_ahead_distance: float = 3.0      # Look-ahead distance (m)
    min_look_ahead: float = 1.0           # Minimum look-ahead (m)

    # Temporal filtering
    history_size: int = 10                # Number of frames to keep
    min_pair_persistence: int = 2         # Min frames for stable pair

    # Error limits
    max_lateral_error: float = 5.0        # Clamp lateral error (m)
    max_heading_error: float = 1.0        # Clamp heading error (rad, ~57 deg)

    # Confidence
    min_confidence: float = 0.3           # Min confidence to output valid state


# =============================================================================
# Corridor Follower
# =============================================================================

class CorridorFollower:
    """Computes corridor state from fused pole detections."""

    def __init__(self, config: Optional[CorridorConfig] = None,
                 config_path: Optional[str] = None):
        """Initialize corridor follower.

        Args:
            config: CorridorConfig object
            config_path: Path to YAML config file
        """
        if config is not None:
            self.config = config
        elif config_path is not None:
            self.config = self._load_config(config_path)
        else:
            self.config = CorridorConfig()

        # State history for temporal filtering
        self._centerline_history: deque = deque(maxlen=self.config.history_size)
        self._width_history: deque = deque(maxlen=self.config.history_size)
        self._pair_history: dict = {}  # (left_id, right_id) -> frame count

        # Smoothed estimates
        self._smoothed_angle: Optional[float] = None
        self._smoothed_offset: Optional[float] = None
        self._smoothed_width: Optional[float] = None

        # Last valid state for hold
        self._last_valid_state: Optional[CorridorState] = None

        # Timing
        self._last_update = time.time()
        self._timing = {'pairing': 0.0, 'fitting': 0.0, 'total': 0.0}

    def _load_config(self, config_path: str) -> CorridorConfig:
        """Load configuration from YAML file."""
        config = CorridorConfig()

        if not YAML_AVAILABLE:
            return config

        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)

            if 'corridor_follower' in data:
                cf = data['corridor_follower']
                config.max_pair_distance_x = cf.get('max_pair_distance_x', config.max_pair_distance_x)
                config.min_corridor_width = cf.get('min_corridor_width', config.min_corridor_width)
                config.max_corridor_width = cf.get('max_corridor_width', config.max_corridor_width)
                config.look_ahead_distance = cf.get('look_ahead_distance', config.look_ahead_distance)
                config.smoothing_alpha = cf.get('smoothing_alpha', config.smoothing_alpha)

            # Also check guidance section for look-ahead
            if 'guidance' in data:
                g = data['guidance']
                if 'look_ahead_distance' in g:
                    config.look_ahead_distance = g['look_ahead_distance']

        except Exception as e:
            print(f"Warning: Could not load corridor config: {e}")

        return config

    def _pair_poles(self, poles: List) -> List[PolePair]:
        """Match poles into left/right pairs.

        Pairs poles that are at similar X (forward) positions but on
        opposite sides (left Y > 0, right Y < 0).

        Args:
            poles: List of FusedPole objects

        Returns:
            List of PolePair objects
        """
        if len(poles) < 2:
            return []

        # Separate into left (Y > 0) and right (Y < 0) poles
        left_poles = [p for p in poles if p.y > 0]
        right_poles = [p for p in poles if p.y <= 0]

        if not left_poles or not right_poles:
            return []

        pairs = []
        used_right = set()

        # For each left pole, find best matching right pole
        for lp in left_poles:
            best_match = None
            best_score = float('inf')

            for rp in right_poles:
                if rp.id in used_right:
                    continue

                # X distance (should be similar)
                dx = abs(lp.x - rp.x)
                if dx > self.config.max_pair_distance_x:
                    continue

                # Corridor width
                width = lp.y - rp.y  # Left Y is positive, right Y is negative
                if width < self.config.min_corridor_width:
                    continue
                if width > self.config.max_corridor_width:
                    continue

                # Score: prefer small X difference and reasonable width
                score = dx + abs(width - 3.0) * 0.5  # Prefer ~3m width

                if score < best_score:
                    best_score = score
                    best_match = rp

            if best_match is not None:
                width = lp.y - best_match.y
                midpoint_x = (lp.x + best_match.x) / 2
                midpoint_y = (lp.y + best_match.y) / 2
                confidence = (lp.fused_confidence + best_match.fused_confidence) / 2

                pair = PolePair(
                    left_id=lp.id,
                    right_id=best_match.id,
                    left_x=lp.x,
                    left_y=lp.y,
                    right_x=best_match.x,
                    right_y=best_match.y,
                    midpoint_x=midpoint_x,
                    midpoint_y=midpoint_y,
                    width=width,
                    confidence=confidence
                )
                pairs.append(pair)
                used_right.add(best_match.id)

        # Sort by X (nearest first)
        pairs.sort(key=lambda p: p.midpoint_x)

        return pairs

    def _fit_centerline(self, pairs: List[PolePair]) -> Tuple[float, float, float]:
        """Fit a line through pole pair midpoints.

        Returns centerline as (angle, offset, confidence) where:
        - angle: heading of centerline in body frame (rad), 0 = straight ahead
        - offset: lateral offset of centerline from drone (m), + = centerline right of drone
        - confidence: fit quality (0-1)

        Args:
            pairs: List of PolePair objects

        Returns:
            (angle, offset, confidence) tuple
        """
        if not pairs:
            return 0.0, 0.0, 0.0

        if len(pairs) == 1:
            # Single pair: assume corridor is straight ahead
            p = pairs[0]
            angle = math.atan2(-p.midpoint_y, p.midpoint_x)  # Angle to midpoint
            offset = -p.midpoint_y  # Lateral offset (+ = centerline right of us)
            return angle, offset, p.confidence

        # Multiple pairs: fit a line
        # Points are (x, y) = (forward, left)
        xs = np.array([p.midpoint_x for p in pairs])
        ys = np.array([p.midpoint_y for p in pairs])

        # Fit line: y = mx + b
        # Using least squares: minimize sum((y - mx - b)^2)
        n = len(pairs)
        sum_x = np.sum(xs)
        sum_y = np.sum(ys)
        sum_xx = np.sum(xs * xs)
        sum_xy = np.sum(xs * ys)

        denom = n * sum_xx - sum_x * sum_x
        if abs(denom) < 1e-6:
            # Vertical line (all poles at same X)
            m = 0.0
            b = np.mean(ys)
        else:
            m = (n * sum_xy - sum_x * sum_y) / denom
            b = (sum_y - m * sum_x) / n

        # Centerline angle (from positive X axis)
        # m = dy/dx, so angle = atan(m)
        # But we want angle relative to straight ahead (X axis)
        angle = math.atan(m)

        # Lateral offset at X=0 (drone position)
        offset = -b  # Negative because + = centerline to our right

        # Confidence based on fit quality
        residuals = ys - (m * xs + b)
        rmse = np.sqrt(np.mean(residuals ** 2))
        confidence = max(0.0, 1.0 - rmse / 2.0)  # Scale: 0 at 2m error
        confidence *= np.mean([p.confidence for p in pairs])

        return angle, offset, confidence

    def _compute_look_ahead(self, angle: float, offset: float,
                            pairs: List[PolePair]) -> Tuple[float, float]:
        """Compute look-ahead point on centerline.

        Args:
            angle: Centerline angle (rad)
            offset: Lateral offset (m)
            pairs: Pole pairs for distance reference

        Returns:
            (look_ahead_x, look_ahead_y) in body frame
        """
        # Look-ahead distance along centerline
        L = self.config.look_ahead_distance

        # If we have pairs, limit look-ahead to furthest pair
        if pairs:
            max_x = max(p.midpoint_x for p in pairs)
            L = min(L, max(self.config.min_look_ahead, max_x))

        # Point on centerline at distance L
        # Centerline passes through (0, -offset) with slope tan(angle)
        # Parameterize: (t * cos(angle), -offset + t * sin(angle))
        look_x = L * math.cos(angle)
        look_y = -offset + L * math.sin(angle)

        return look_x, look_y

    def _smooth_estimates(self, angle: float, offset: float, width: float,
                          alpha: float) -> Tuple[float, float, float]:
        """Apply exponential moving average smoothing.

        Args:
            angle: New centerline angle
            offset: New lateral offset
            width: New corridor width
            alpha: Smoothing factor (0=no smoothing, 1=max smoothing)

        Returns:
            Smoothed (angle, offset, width)
        """
        if self._smoothed_angle is None:
            self._smoothed_angle = angle
            self._smoothed_offset = offset
            self._smoothed_width = width
        else:
            # EMA: new = alpha * old + (1-alpha) * measurement
            self._smoothed_angle = alpha * self._smoothed_angle + (1 - alpha) * angle
            self._smoothed_offset = alpha * self._smoothed_offset + (1 - alpha) * offset
            self._smoothed_width = alpha * self._smoothed_width + (1 - alpha) * width

        return self._smoothed_angle, self._smoothed_offset, self._smoothed_width

    def update(self, poles: List) -> CorridorState:
        """Update corridor state with new pole detections.

        Main entry point. Call this with fused pole detections
        at the perception frame rate (~10-30 Hz).

        Args:
            poles: List of FusedPole objects from sensor fusion

        Returns:
            CorridorState for guidance system
        """
        t_start = time.time()

        # Default invalid state
        invalid_state = CorridorState(
            lateral_error=0.0,
            heading_error=0.0,
            look_ahead_x=self.config.look_ahead_distance,
            look_ahead_y=0.0,
            confidence=0.0,
            num_poles=len(poles) if poles else 0,
            num_pairs=0,
            corridor_width=0.0,
            centerline_angle=0.0,
            valid=False
        )

        if not poles or len(poles) < self.config.min_poles_for_corridor:
            # Hold last valid state briefly
            if self._last_valid_state is not None:
                age = time.time() - self._last_update
                if age < 0.5:  # Hold for 500ms
                    return self._last_valid_state
            return invalid_state

        # Step 1: Pair poles
        t0 = time.time()
        pairs = self._pair_poles(poles)
        self._timing['pairing'] = time.time() - t0

        # Step 2: Fit centerline
        t0 = time.time()
        if pairs:
            angle, offset, fit_confidence = self._fit_centerline(pairs)
            avg_width = np.mean([p.width for p in pairs])
        else:
            # No pairs - try to estimate from individual poles
            if len(poles) >= 2:
                # Use centroid of all poles as rough centerline estimate
                xs = [p.x for p in poles]
                ys = [p.y for p in poles]
                centroid_x = np.mean(xs)
                centroid_y = np.mean(ys)
                angle = math.atan2(-centroid_y, centroid_x)
                offset = -centroid_y
                fit_confidence = 0.3  # Low confidence
                avg_width = max(abs(p.y) for p in poles) * 2
            else:
                self._timing['fitting'] = time.time() - t0
                return invalid_state

        self._timing['fitting'] = time.time() - t0

        # Step 3: Smooth estimates
        angle, offset, avg_width = self._smooth_estimates(
            angle, offset, avg_width, self.config.smoothing_alpha
        )

        # Step 4: Compute errors
        # Lateral error: + = drone is right of centerline
        # offset is + when centerline is right of drone
        # So lateral_error = offset (we need to move left if centerline is right)
        lateral_error = offset

        # Heading error: + = drone pointing right of centerline direction
        # angle is centerline heading, we want to align with it
        heading_error = -angle  # Negative because we want to turn toward centerline direction

        # Clamp errors
        lateral_error = max(-self.config.max_lateral_error,
                           min(self.config.max_lateral_error, lateral_error))
        heading_error = max(-self.config.max_heading_error,
                           min(self.config.max_heading_error, heading_error))

        # Step 5: Compute look-ahead point
        look_x, look_y = self._compute_look_ahead(angle, offset, pairs)

        # Step 6: Compute overall confidence
        pole_confidence = np.mean([p.fused_confidence for p in poles])
        confidence = fit_confidence * pole_confidence

        if pairs:
            # Boost confidence with pairs
            confidence = min(1.0, confidence + 0.2 * len(pairs) / 3)

        # Build output state
        state = CorridorState(
            lateral_error=lateral_error,
            heading_error=heading_error,
            look_ahead_x=look_x,
            look_ahead_y=look_y,
            confidence=confidence,
            num_poles=len(poles),
            num_pairs=len(pairs),
            corridor_width=avg_width,
            centerline_angle=angle,
            valid=confidence >= self.config.min_confidence
        )

        # Store for hold
        if state.valid:
            self._last_valid_state = state
            self._last_update = time.time()

        self._timing['total'] = time.time() - t_start

        return state

    def get_timing(self) -> dict:
        """Get timing breakdown for last update."""
        return self._timing.copy()

    def reset(self):
        """Reset all state."""
        self._centerline_history.clear()
        self._width_history.clear()
        self._pair_history.clear()
        self._smoothed_angle = None
        self._smoothed_offset = None
        self._smoothed_width = None
        self._last_valid_state = None


# =============================================================================
# Visualization Helper
# =============================================================================

def draw_corridor_state(image: np.ndarray, state: CorridorState,
                        poles: List, pairs: List[PolePair] = None,
                        scale: float = 50.0) -> np.ndarray:
    """Draw corridor visualization overlay on image.

    Args:
        image: BGR image to draw on
        state: CorridorState to visualize
        poles: List of FusedPole objects
        pairs: Optional list of PolePair objects
        scale: Pixels per meter for BEV overlay

    Returns:
        Image with overlay drawn
    """
    try:
        import cv2
    except ImportError:
        return image

    h, w = image.shape[:2]
    overlay = image.copy()

    # Draw info text
    info_y = 30
    line_height = 25

    def put_text(text, color=(255, 255, 255)):
        nonlocal info_y
        cv2.putText(overlay, text, (10, info_y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, color, 2)
        info_y += line_height

    # Status color
    if state.valid:
        status_color = (0, 255, 0)  # Green
        status = "VALID"
    else:
        status_color = (0, 0, 255)  # Red
        status = "INVALID"

    put_text(f"Corridor: {status}", status_color)
    put_text(f"Poles: {state.num_poles}, Pairs: {state.num_pairs}")
    put_text(f"Width: {state.corridor_width:.1f}m")
    put_text(f"Lateral err: {state.lateral_error:+.2f}m")
    put_text(f"Heading err: {math.degrees(state.heading_error):+.1f}deg")
    put_text(f"Confidence: {state.confidence:.0%}")

    # Draw mini BEV in corner
    bev_size = 150
    bev_x = w - bev_size - 10
    bev_y = 10
    bev_center = (bev_x + bev_size // 2, bev_y + bev_size)

    # BEV background
    cv2.rectangle(overlay, (bev_x, bev_y), (bev_x + bev_size, bev_y + bev_size),
                  (40, 40, 40), -1)
    cv2.rectangle(overlay, (bev_x, bev_y), (bev_x + bev_size, bev_y + bev_size),
                  (100, 100, 100), 1)

    # Draw drone (at bottom center of BEV)
    cv2.circle(overlay, bev_center, 5, (0, 255, 255), -1)

    # Draw poles in BEV
    for pole in poles:
        px = int(bev_center[0] - pole.y * scale / 5)  # Y is left, so negate for screen
        py = int(bev_center[1] - pole.x * scale / 5)  # X is forward, so up on screen

        if bev_x < px < bev_x + bev_size and bev_y < py < bev_y + bev_size:
            color = (0, 255, 0) if pole.y > 0 else (255, 0, 0)  # Green=left, Blue=right
            cv2.circle(overlay, (px, py), 4, color, -1)

    # Draw look-ahead point
    lax = int(bev_center[0] - state.look_ahead_y * scale / 5)
    lay = int(bev_center[1] - state.look_ahead_x * scale / 5)
    if bev_x < lax < bev_x + bev_size and bev_y < lay < bev_y + bev_size:
        cv2.circle(overlay, (lax, lay), 6, (0, 255, 255), 2)
        cv2.line(overlay, bev_center, (lax, lay), (0, 255, 255), 1)

    # Draw centerline
    if state.valid:
        # Line from drone toward look-ahead
        cv2.line(overlay, bev_center, (lax, lay), (255, 255, 0), 2)

    return overlay


# =============================================================================
# Convenience Functions
# =============================================================================

def create_corridor_follower(config_path: str = 'config/settings.yaml') -> CorridorFollower:
    """Create a CorridorFollower with config from file."""
    return CorridorFollower(config_path=config_path)


# =============================================================================
# Test
# =============================================================================

if __name__ == '__main__':
    from dataclasses import dataclass

    # Mock FusedPole for testing
    @dataclass
    class MockPole:
        id: int
        x: float
        y: float
        z: float = 1.0
        fused_confidence: float = 0.8

    # Create test poles (corridor going forward)
    test_poles = [
        # Left poles (Y > 0)
        MockPole(1, 3.0, 1.5),   # 3m ahead, 1.5m left
        MockPole(2, 6.0, 1.6),   # 6m ahead, 1.6m left
        MockPole(3, 9.0, 1.4),   # 9m ahead, 1.4m left
        # Right poles (Y < 0)
        MockPole(4, 3.0, -1.5),  # 3m ahead, 1.5m right
        MockPole(5, 6.0, -1.4),  # 6m ahead, 1.4m right
        MockPole(6, 9.0, -1.6),  # 9m ahead, 1.6m right
    ]

    # Create follower
    follower = CorridorFollower()

    # Update with test poles
    state = follower.update(test_poles)

    print("Corridor State:")
    print(f"  Valid: {state.valid}")
    print(f"  Poles: {state.num_poles}, Pairs: {state.num_pairs}")
    print(f"  Corridor width: {state.corridor_width:.2f}m")
    print(f"  Lateral error: {state.lateral_error:+.3f}m")
    print(f"  Heading error: {math.degrees(state.heading_error):+.2f}deg")
    print(f"  Look-ahead: ({state.look_ahead_x:.2f}, {state.look_ahead_y:.2f})")
    print(f"  Confidence: {state.confidence:.2f}")

    # Test with offset drone (simulate being 0.5m right of center)
    print("\nTest with offset poles (drone 0.5m right of center):")
    offset_poles = [
        MockPole(1, 3.0, 2.0),   # Left pole appears further left
        MockPole(2, 6.0, 2.1),
        MockPole(4, 3.0, -1.0),  # Right pole appears closer
        MockPole(5, 6.0, -0.9),
    ]
    state = follower.update(offset_poles)
    print(f"  Lateral error: {state.lateral_error:+.3f}m (should be ~+0.5)")
    print(f"  Heading error: {math.degrees(state.heading_error):+.2f}deg")
