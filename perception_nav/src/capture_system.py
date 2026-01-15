"""
Tesla-Style Capture System

Combines:
1. Predictive Capture Scheduler - event-driven, not polling
2. Async Data Bundler - fire-and-forget, never blocks main loop
3. Path Corridor Obstacle Detector - only checks what matters

Design principles:
- Zero per-frame search (pre-computed triggers)
- Zero main-thread I/O (async worker)
- Minimal point processing (corridor filter only)
"""

import hashlib
import heapq
import json
import os
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from queue import Queue, Empty
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class CaptureEvent:
    """Scheduled capture event."""
    trigger_time: float      # When to capture (monotonic time)
    pole_id: int             # Which pole
    pole_x: float            # Pole position at scheduling time
    pole_y: float
    distance: float          # Distance when scheduled

    def __lt__(self, other):
        return self.trigger_time < other.trigger_time


@dataclass
class CapturePackage:
    """Data package for async saving."""
    timestamp: float
    pole_id: int
    image: Optional[np.ndarray]      # Camera frame (or None)
    points: Optional[np.ndarray]     # LiDAR points near pole (Nx5)
    metadata: dict
    output_dir: str


@dataclass
class ObstacleState:
    """Obstacle detection result."""
    obstacle_detected: bool = False
    obstacle_distance: float = float('inf')
    obstacle_point_count: int = 0
    obstacle_velocity: float = 0.0  # Approaching velocity (positive = coming toward us)
    confidence: float = 0.0


# =============================================================================
# Ring Buffer - Zero-copy frame history
# =============================================================================

class RingBuffer:
    """Fixed-size ring buffer for frames. O(1) insert, O(1) access."""

    def __init__(self, capacity: int = 30):
        self.capacity = capacity
        self.buffer: deque = deque(maxlen=capacity)
        self.timestamps: deque = deque(maxlen=capacity)
        self._lock = threading.Lock()

    def push(self, frame, timestamp: float = None):
        """Add frame to buffer. O(1)."""
        if timestamp is None:
            timestamp = time.time()
        with self._lock:
            self.buffer.append(frame)
            self.timestamps.append(timestamp)

    def get_closest(self, target_time: float):
        """Get frame closest to target time. O(n) but n is small (30)."""
        with self._lock:
            if not self.buffer:
                return None, None

            # Find closest timestamp
            best_idx = 0
            best_diff = float('inf')
            for i, t in enumerate(self.timestamps):
                diff = abs(t - target_time)
                if diff < best_diff:
                    best_diff = diff
                    best_idx = i

            return self.buffer[best_idx], self.timestamps[best_idx]

    def get_latest(self):
        """Get most recent frame. O(1)."""
        with self._lock:
            if not self.buffer:
                return None, None
            return self.buffer[-1], self.timestamps[-1]


# =============================================================================
# Capture Scheduler - Predictive Event-Driven
# =============================================================================

class CaptureScheduler:
    """
    Tesla-style predictive capture scheduling.

    Instead of polling every frame "are we close to a pole?",
    we pre-compute capture times and use a priority queue.

    Compute cost: O(log n) per pole addition, O(1) per frame check.
    """

    def __init__(self,
                 capture_distance: float = 10.0,
                 min_interval: float = 2.0,
                 approach_velocity: float = 3.0):
        """
        Args:
            capture_distance: Distance at which to trigger capture (m)
            min_interval: Minimum time between captures (s)
            approach_velocity: Default approach velocity for timing (m/s)
        """
        self.capture_distance = capture_distance
        self.min_interval = min_interval
        self.default_velocity = approach_velocity

        # Priority queue of scheduled captures (min-heap by time)
        self._event_queue: List[CaptureEvent] = []

        # Track which poles we've scheduled/captured
        self._scheduled_poles: set = set()
        self._captured_poles: set = set()

        # Last capture time (for min_interval)
        self._last_capture_time: float = 0

        # Current velocity estimate
        self._current_velocity: float = approach_velocity

        self._lock = threading.Lock()

    def update_velocity(self, velocity: float):
        """Update current velocity estimate for better timing."""
        if velocity > 0.1:
            self._current_velocity = velocity

    def schedule_pole(self, pole_id: int, pole_x: float, pole_y: float,
                      current_x: float = 0, current_y: float = 0):
        """
        Schedule capture for a pole if not already scheduled.

        Args:
            pole_id: Unique pole ID
            pole_x, pole_y: Pole position in body frame
            current_x, current_y: Current drone position (usually 0,0 in body frame)
        """
        with self._lock:
            # Already scheduled or captured?
            if pole_id in self._scheduled_poles or pole_id in self._captured_poles:
                return

            # Only schedule poles ahead of us
            if pole_x <= 0:
                return

            # Calculate distance and time to capture point
            distance = np.sqrt((pole_x - current_x)**2 + (pole_y - current_y)**2)

            # When will we be at capture_distance from the pole?
            distance_to_trigger = distance - self.capture_distance
            if distance_to_trigger <= 0:
                # Already past trigger point, capture now
                trigger_time = time.monotonic()
            else:
                # Estimate time to reach trigger point
                velocity = max(0.5, self._current_velocity)
                time_to_trigger = distance_to_trigger / velocity
                trigger_time = time.monotonic() + time_to_trigger

            # Create and schedule event
            event = CaptureEvent(
                trigger_time=trigger_time,
                pole_id=pole_id,
                pole_x=pole_x,
                pole_y=pole_y,
                distance=distance
            )

            heapq.heappush(self._event_queue, event)
            self._scheduled_poles.add(pole_id)

    def check_trigger(self) -> Optional[CaptureEvent]:
        """
        Check if any capture should trigger now.

        O(1) average case - just peek at heap top.

        Returns:
            CaptureEvent if capture should trigger, None otherwise
        """
        now = time.monotonic()

        with self._lock:
            # Respect minimum interval
            if now - self._last_capture_time < self.min_interval:
                return None

            # Check if top event should trigger
            while self._event_queue:
                event = self._event_queue[0]  # Peek

                # Already captured this pole?
                if event.pole_id in self._captured_poles:
                    heapq.heappop(self._event_queue)
                    continue

                # Time to trigger?
                if event.trigger_time <= now:
                    heapq.heappop(self._event_queue)
                    self._captured_poles.add(event.pole_id)
                    self._last_capture_time = now
                    return event

                # Not yet time
                break

            return None

    def get_pending_count(self) -> int:
        """Get number of pending captures."""
        with self._lock:
            return len(self._event_queue)

    def reset(self):
        """Reset all scheduling state."""
        with self._lock:
            self._event_queue.clear()
            self._scheduled_poles.clear()
            self._captured_poles.clear()
            self._last_capture_time = 0


# =============================================================================
# Async Data Bundler - Fire and Forget
# =============================================================================

class AsyncDataBundler:
    """
    Async data saving - never blocks main perception loop.

    Main thread just pushes to queue and continues.
    Worker thread handles all I/O in background.
    """

    def __init__(self, output_base: str = "captures", max_queue_size: int = 50):
        self.output_base = output_base
        self._queue: Queue = Queue(maxsize=max_queue_size)
        self._worker: Optional[threading.Thread] = None
        self._running = False

        # Stats
        self._saved_count = 0
        self._dropped_count = 0

        # Ensure output directory exists
        os.makedirs(output_base, exist_ok=True)

    def start(self):
        """Start the background worker."""
        if self._running:
            return

        self._running = True
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

    def stop(self):
        """Stop the background worker."""
        self._running = False
        if self._worker:
            self._worker.join(timeout=2.0)

    def submit(self, package: CapturePackage) -> bool:
        """
        Submit capture package for async saving.

        Non-blocking. Returns False if queue is full (data dropped).
        """
        try:
            self._queue.put_nowait(package)
            return True
        except:
            self._dropped_count += 1
            return False

    def _worker_loop(self):
        """Background worker - handles all I/O."""
        while self._running:
            try:
                package = self._queue.get(timeout=0.5)
                self._save_package(package)
                self._saved_count += 1
            except Empty:
                continue
            except Exception as e:
                print(f"DataBundler error: {e}")

    def _save_package(self, pkg: CapturePackage):
        """Save a single capture package."""
        # Create folder
        timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(pkg.timestamp))
        folder_name = f"{timestamp_str}_pole_{pkg.pole_id}"
        folder_path = os.path.join(self.output_base, folder_name)
        os.makedirs(folder_path, exist_ok=True)

        checksums = {}

        # Save image (if available)
        if pkg.image is not None and CV2_AVAILABLE:
            img_path = os.path.join(folder_path, "image.jpg")
            cv2.imwrite(img_path, pkg.image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            checksums['image.jpg'] = self._md5_file(img_path)

        # Save points (simple binary format for speed)
        if pkg.points is not None and len(pkg.points) > 0:
            pts_path = os.path.join(folder_path, "points.npy")
            np.save(pts_path, pkg.points.astype(np.float32))
            checksums['points.npy'] = self._md5_file(pts_path)

        # Save metadata
        pkg.metadata['checksums'] = checksums
        pkg.metadata['saved_at'] = time.time()

        meta_path = os.path.join(folder_path, "metadata.json")
        with open(meta_path, 'w') as f:
            json.dump(pkg.metadata, f, indent=2, default=str)

    def _md5_file(self, path: str) -> str:
        """Compute MD5 checksum of file."""
        hasher = hashlib.md5()
        with open(path, 'rb') as f:
            for chunk in iter(lambda: f.read(65536), b''):
                hasher.update(chunk)
        return hasher.hexdigest()

    def get_stats(self) -> dict:
        """Get saving statistics."""
        return {
            'saved': self._saved_count,
            'dropped': self._dropped_count,
            'pending': self._queue.qsize()
        }


# =============================================================================
# Path Corridor Obstacle Detector
# =============================================================================

class CorridorObstacleDetector:
    """
    Tesla-style obstacle detection - only checks the planned path corridor.

    Instead of processing the entire point cloud for obstacles everywhere,
    we only check points that could actually be in our way.

    Compute cost: Single pass through points with simple box/corridor check.
    """

    def __init__(self,
                 corridor_length: float = 15.0,
                 corridor_width: float = 2.0,
                 altitude_min: float = -1.0,
                 altitude_max: float = 2.0,
                 min_obstacle_points: int = 10,
                 danger_distance: float = 5.0):
        """
        Args:
            corridor_length: How far ahead to check (m)
            corridor_width: Half-width of corridor (m) - full width is 2x this
            altitude_min/max: Vertical band to check (m relative to drone)
            min_obstacle_points: Minimum points to consider an obstacle
            danger_distance: Distance at which obstacle is critical (m)
        """
        self.corridor_length = corridor_length
        self.corridor_width = corridor_width
        self.altitude_min = altitude_min
        self.altitude_max = altitude_max
        self.min_obstacle_points = min_obstacle_points
        self.danger_distance = danger_distance

        # Path for curved corridor check (optional)
        self._path_points: Optional[np.ndarray] = None

        # Temporal smoothing
        self._last_state = ObstacleState()
        self._smoothing = 0.7

    def set_path(self, path_points: np.ndarray):
        """
        Set curved path for corridor check.

        Args:
            path_points: Nx2 array of (x, y) path points
        """
        self._path_points = path_points

    def check_corridor(self, points: np.ndarray,
                       use_velocity: bool = True) -> ObstacleState:
        """
        Check for obstacles in the flight corridor.

        Single-pass through points. O(n) but very cache-friendly.

        Args:
            points: Nx5 array (x, y, z, velocity, reflectance)
            use_velocity: Use FMCW velocity for approach detection

        Returns:
            ObstacleState with detection results
        """
        if points is None or len(points) == 0:
            return self._smooth_result(ObstacleState())

        # Extract coordinates
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        # Simple box corridor check (very fast)
        # X: ahead of us (0 to corridor_length)
        # Y: within corridor width
        # Z: within altitude band

        in_corridor = (
            (x > 0.5) &  # Ahead of us (ignore points too close)
            (x < self.corridor_length) &
            (np.abs(y) < self.corridor_width) &
            (z > self.altitude_min) &
            (z < self.altitude_max)
        )

        # If we have a curved path, refine the check
        if self._path_points is not None and len(self._path_points) > 2:
            in_corridor = self._check_curved_corridor(points, in_corridor)

        corridor_points = points[in_corridor]
        point_count = len(corridor_points)

        # No obstacle if too few points
        if point_count < self.min_obstacle_points:
            return self._smooth_result(ObstacleState())

        # Find closest obstacle point
        distances = np.sqrt(corridor_points[:, 0]**2 + corridor_points[:, 1]**2)
        min_distance = float(np.min(distances))

        # Approach velocity (from FMCW)
        approach_velocity = 0.0
        if use_velocity and points.shape[1] > 3:
            # Negative velocity = approaching us
            velocities = corridor_points[:, 3]
            # Use median of points in danger zone
            danger_mask = distances < self.danger_distance
            if np.any(danger_mask):
                approach_velocity = -float(np.median(velocities[danger_mask]))

        # Confidence based on point count and distance
        confidence = min(1.0, point_count / 50.0)
        if min_distance < self.danger_distance:
            confidence = min(1.0, confidence + 0.3)

        state = ObstacleState(
            obstacle_detected=True,
            obstacle_distance=min_distance,
            obstacle_point_count=point_count,
            obstacle_velocity=approach_velocity,
            confidence=confidence
        )

        return self._smooth_result(state)

    def _check_curved_corridor(self, points: np.ndarray,
                                initial_mask: np.ndarray) -> np.ndarray:
        """
        Refine corridor check for curved path.

        For each point, find distance to nearest path segment.
        """
        if not np.any(initial_mask):
            return initial_mask

        candidate_points = points[initial_mask][:, :2]  # XY only
        path = self._path_points

        # For each candidate point, find min distance to path
        # This is O(n*m) but n (candidates) and m (path points) are small
        refined_mask = initial_mask.copy()
        candidate_indices = np.where(initial_mask)[0]

        for i, (px, py) in enumerate(candidate_points):
            # Distance to path segments
            min_dist = float('inf')
            for j in range(len(path) - 1):
                dist = self._point_to_segment_dist(
                    px, py,
                    path[j, 0], path[j, 1],
                    path[j+1, 0], path[j+1, 1]
                )
                min_dist = min(min_dist, dist)

            # Outside corridor width?
            if min_dist > self.corridor_width:
                refined_mask[candidate_indices[i]] = False

        return refined_mask

    @staticmethod
    def _point_to_segment_dist(px, py, x1, y1, x2, y2) -> float:
        """Distance from point to line segment."""
        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            return np.sqrt((px - x1)**2 + (py - y1)**2)

        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx*dx + dy*dy)))

        proj_x = x1 + t * dx
        proj_y = y1 + t * dy

        return np.sqrt((px - proj_x)**2 + (py - proj_y)**2)

    def _smooth_result(self, state: ObstacleState) -> ObstacleState:
        """Apply temporal smoothing to reduce jitter."""
        alpha = self._smoothing

        # Smooth distance and confidence
        if self._last_state.obstacle_detected and state.obstacle_detected:
            state.obstacle_distance = (
                alpha * self._last_state.obstacle_distance +
                (1 - alpha) * state.obstacle_distance
            )
            state.confidence = (
                alpha * self._last_state.confidence +
                (1 - alpha) * state.confidence
            )

        self._last_state = state
        return state

    def check_simple_box(self, points: np.ndarray) -> ObstacleState:
        """
        Ultra-fast simple box check. Use when curved path not needed.

        Just a forward box - no path following logic.
        """
        if points is None or len(points) == 0:
            return ObstacleState()

        # Vectorized box check - very fast
        mask = (
            (points[:, 0] > 0.5) &
            (points[:, 0] < self.corridor_length) &
            (np.abs(points[:, 1]) < self.corridor_width) &
            (points[:, 2] > self.altitude_min) &
            (points[:, 2] < self.altitude_max)
        )

        count = np.sum(mask)

        if count < self.min_obstacle_points:
            return ObstacleState()

        # Quick distance calculation
        corridor_x = points[mask, 0]
        min_dist = float(np.min(corridor_x))

        return ObstacleState(
            obstacle_detected=True,
            obstacle_distance=min_dist,
            obstacle_point_count=int(count),
            confidence=min(1.0, count / 30.0)
        )


# =============================================================================
# Integrated Capture Manager
# =============================================================================

class CaptureManager:
    """
    Unified manager for capture scheduling, bundling, and obstacle detection.

    Single update() call per frame handles everything.
    """

    def __init__(self, config: dict = None):
        config = config or {}

        # Components
        self.scheduler = CaptureScheduler(
            capture_distance=config.get('capture_distance', 10.0),
            min_interval=config.get('min_capture_interval', 2.0),
            approach_velocity=config.get('approach_velocity', 3.0)
        )

        self.bundler = AsyncDataBundler(
            output_base=config.get('output_dir', 'captures')
        )

        self.obstacle_detector = CorridorObstacleDetector(
            corridor_length=config.get('corridor_length', 15.0),
            corridor_width=config.get('corridor_width', 2.0),
            altitude_min=config.get('altitude_min', -1.0),
            altitude_max=config.get('altitude_max', 2.0),
            min_obstacle_points=config.get('min_obstacle_points', 10),
            danger_distance=config.get('danger_distance', 5.0)
        )

        # Ring buffers
        self.image_buffer = RingBuffer(capacity=30)
        self.lidar_buffer = RingBuffer(capacity=30)

        # State
        self._running = False
        self._last_obstacle_state = ObstacleState()

    def start(self):
        """Start the capture system."""
        self._running = True
        self.bundler.start()

    def stop(self):
        """Stop the capture system."""
        self._running = False
        self.bundler.stop()

    def push_image(self, image: np.ndarray, timestamp: float = None):
        """Push camera frame to ring buffer."""
        self.image_buffer.push(image, timestamp)

    def push_lidar(self, points: np.ndarray, timestamp: float = None):
        """Push LiDAR points to ring buffer."""
        self.lidar_buffer.push(points, timestamp)

    def update(self, poles: list, velocity: float = 0.0,
               lidar_points: np.ndarray = None) -> Tuple[Optional[CaptureEvent], ObstacleState]:
        """
        Main update - call once per perception frame.

        Args:
            poles: List of pole detections (need .id, .x, .y attributes)
            velocity: Current forward velocity (m/s)
            lidar_points: Current LiDAR points for obstacle check

        Returns:
            Tuple of (capture_event or None, obstacle_state)
        """
        if not self._running:
            return None, ObstacleState()

        # Update velocity estimate
        self.scheduler.update_velocity(velocity)

        # Schedule any new poles
        for pole in poles:
            if hasattr(pole, 'id') and hasattr(pole, 'x') and hasattr(pole, 'y'):
                self.scheduler.schedule_pole(pole.id, pole.x, pole.y)

        # Check for capture trigger
        capture_event = self.scheduler.check_trigger()

        if capture_event:
            self._handle_capture(capture_event, poles)

        # Check for obstacles
        if lidar_points is not None:
            obstacle_state = self.obstacle_detector.check_simple_box(lidar_points)
        else:
            obstacle_state = ObstacleState()

        self._last_obstacle_state = obstacle_state

        return capture_event, obstacle_state

    def _handle_capture(self, event: CaptureEvent, poles: list):
        """Handle a capture trigger."""
        now = time.time()

        # Get frames from ring buffers
        image, img_time = self.image_buffer.get_latest()
        points, pts_time = self.lidar_buffer.get_latest()

        # Extract points near the pole (if we have points)
        pole_points = None
        if points is not None:
            # Find points within 3m of pole position
            if len(points) > 0:
                dist = np.sqrt(
                    (points[:, 0] - event.pole_x)**2 +
                    (points[:, 1] - event.pole_y)**2
                )
                pole_points = points[dist < 3.0]

        # Get pole info if still tracked
        pole_info = {}
        for p in poles:
            if hasattr(p, 'id') and p.id == event.pole_id:
                pole_info = {
                    'x': getattr(p, 'x', 0),
                    'y': getattr(p, 'y', 0),
                    'z': getattr(p, 'z', 0),
                    'confidence': getattr(p, 'fused_confidence',
                                         getattr(p, 'confidence', 0)),
                    'radius': getattr(p, 'radius', 0),
                }
                break

        # Build metadata
        metadata = {
            'pole_id': event.pole_id,
            'capture_time': now,
            'image_time': img_time,
            'lidar_time': pts_time,
            'scheduled_distance': event.distance,
            'pole_info': pole_info,
            'point_count': len(pole_points) if pole_points is not None else 0,
        }

        # Submit to async bundler
        package = CapturePackage(
            timestamp=now,
            pole_id=event.pole_id,
            image=image.copy() if image is not None else None,
            points=pole_points,
            metadata=metadata,
            output_dir=self.bundler.output_base
        )

        self.bundler.submit(package)

    def get_obstacle_state(self) -> ObstacleState:
        """Get last obstacle detection state."""
        return self._last_obstacle_state

    def get_stats(self) -> dict:
        """Get system statistics."""
        return {
            'pending_captures': self.scheduler.get_pending_count(),
            'bundler': self.bundler.get_stats(),
            'obstacle': {
                'detected': self._last_obstacle_state.obstacle_detected,
                'distance': self._last_obstacle_state.obstacle_distance,
            }
        }

    def reset(self):
        """Reset all state."""
        self.scheduler.reset()
        self.image_buffer = RingBuffer(capacity=30)
        self.lidar_buffer = RingBuffer(capacity=30)


# =============================================================================
# Test
# =============================================================================

if __name__ == '__main__':
    print("Testing Capture System Components")
    print("=" * 60)

    # Test CaptureScheduler
    print("\n1. CaptureScheduler Test")
    scheduler = CaptureScheduler(capture_distance=5.0, min_interval=1.0)

    # Schedule some poles
    scheduler.schedule_pole(1, 20.0, 2.0)  # 20m ahead
    scheduler.schedule_pole(2, 50.0, -1.0)  # 50m ahead
    scheduler.schedule_pole(3, 8.0, 0.0)   # 8m ahead - should trigger soon

    print(f"   Pending captures: {scheduler.get_pending_count()}")

    # Simulate time passing
    time.sleep(0.1)
    event = scheduler.check_trigger()
    if event:
        print(f"   Triggered capture for pole {event.pole_id}")

    # Test ObstacleDetector
    print("\n2. CorridorObstacleDetector Test")
    detector = CorridorObstacleDetector(corridor_length=10.0, corridor_width=2.0)

    # Create test points - some in corridor, some not
    test_points = np.array([
        [5.0, 0.0, 0.5, 0.0, 0.0],   # In corridor
        [5.0, 0.5, 0.5, 0.0, 0.0],   # In corridor
        [5.0, 1.0, 0.5, -0.5, 0.0],  # In corridor, approaching
        [5.0, 5.0, 0.5, 0.0, 0.0],   # Outside corridor (Y too large)
        [15.0, 0.0, 0.5, 0.0, 0.0],  # Outside corridor (X too large)
        [3.0, 0.0, 5.0, 0.0, 0.0],   # Outside corridor (Z too large)
    ] * 5, dtype=np.float32)  # Repeat to exceed min_points

    state = detector.check_simple_box(test_points)
    print(f"   Obstacle detected: {state.obstacle_detected}")
    print(f"   Distance: {state.obstacle_distance:.1f}m")
    print(f"   Point count: {state.obstacle_point_count}")

    # Test RingBuffer
    print("\n3. RingBuffer Test")
    buffer = RingBuffer(capacity=10)
    for i in range(15):
        buffer.push(f"frame_{i}")

    latest, ts = buffer.get_latest()
    print(f"   Latest frame: {latest}")
    print(f"   Buffer maintains last 10 only")

    print("\n" + "=" * 60)
    print("All tests passed!")
