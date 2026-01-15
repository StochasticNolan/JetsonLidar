"""
LiDAR-based Pole Detection with Kalman Tracking

This module provides pole detection from Aeva Atlas FMCW LiDAR point clouds.
It implements:
- RANSAC ground plane removal
- DBSCAN clustering
- PCA-based cylinder fitting
- Kalman filter tracking with stable IDs

Usage:
    from pole_detector import PoleDetector

    detector = PoleDetector(config_path='config/settings.yaml')
    poles = detector.detect(points)  # Nx5 numpy array
"""

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    from sklearn.cluster import DBSCAN
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False
    print("Warning: sklearn not available, clustering disabled")

try:
    from scipy.optimize import linear_sum_assignment
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("Warning: scipy not available, using greedy association")

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class PoleDetection:
    """Output format for a detected pole."""
    id: int                         # Stable tracking ID (-1 if untracked)
    x: float                        # X position (forward, meters)
    y: float                        # Y position (left, meters)
    z: float                        # Z position (centroid height, meters)
    radius: float                   # Estimated pole radius (meters)
    confidence: float               # Detection confidence (0.0-1.0)
    velocity: float                 # Radial velocity from FMCW data (m/s)

    # Extended attributes
    height: float = 0.0             # Pole height (z_max - z_min)
    z_min: float = 0.0              # Bottom of pole
    z_max: float = 0.0              # Top of pole
    verticality: float = 0.0        # Alignment with vertical (0-1)
    num_points: int = 0             # Point count in cluster
    range_xy: float = 0.0           # Horizontal distance from sensor


@dataclass
class PoleDetectorConfig:
    """Configuration for pole detection."""
    # Performance tuning
    downsample_factor: int = 10         # Process every Nth point (1 = no downsampling)
    max_points: int = 5000              # Max points after downsampling

    # RANSAC ground removal
    ransac_threshold: float = 0.05      # Inlier distance threshold (m)
    ransac_iterations: int = 20         # Max RANSAC iterations
    ransac_sample_size: int = 300       # Max ground candidates for RANSAC
    ground_height_margin: float = 0.1   # Height above ground to keep (m)

    # DBSCAN clustering
    cluster_eps: float = 0.5            # Neighborhood radius (m) - larger = faster
    cluster_min_samples: int = 5        # Min points per cluster (after downsampling)

    # Cylinder validation
    pole_min_height: float = 0.5        # Min pole height (m)
    pole_max_height: float = 20.0       # Max pole height (m)
    pole_min_radius: float = 0.05       # Min pole radius (m)
    pole_max_radius: float = 0.3        # Max pole radius (m)
    pole_verticality: float = 0.85      # Min cos(angle) with Z axis
    pole_min_aspect: float = 3.0        # Min height/diameter ratio

    # Performance limits
    max_clusters: int = 15              # Max clusters to fit (skip rest)
    max_points_per_cluster: int = 100   # Subsample large clusters

    # Range filtering
    min_range: float = 0.5              # Min detection range (m)
    max_range: float = 50.0             # Max detection range (m)

    # Tracking parameters
    track_max_age: int = 10             # Frames before dropping track
    track_min_hits: int = 3             # Hits before confirmed track
    track_association_threshold: float = 1.0  # Max distance for association (m)

    # Moving object rejection (FMCW velocity filtering)
    velocity_filter_enabled: bool = True
    max_point_velocity: float = 0.5     # Max velocity for individual points (m/s)
    max_cluster_velocity: float = 0.3   # Max median velocity for cluster (m/s)
    velocity_outlier_ratio: float = 0.2 # Max ratio of high-velocity points in cluster

    # Ground plane stability
    ground_plane_smoothing: float = 0.7 # EMA factor (0=none, 1=max smoothing)


# =============================================================================
# Kalman Filter Tracker
# =============================================================================

class KalmanPoleTracker:
    """Kalman filter for tracking a single pole."""

    def __init__(self, initial_pos: np.ndarray, pole_id: int, dt: float = 0.1):
        """Initialize tracker with first observation.

        Args:
            initial_pos: [x, y, z] initial position
            pole_id: Unique ID for this pole
            dt: Time step between frames (default 0.1s = 10Hz)
        """
        self.id = pole_id
        self.hits = 1
        self.age = 0
        self.time_since_update = 0

        # State: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6, dtype=np.float64)
        self.state[:3] = initial_pos

        # State covariance
        self.P = np.eye(6, dtype=np.float64) * 1.0
        self.P[3:, 3:] *= 10.0  # Higher uncertainty on velocity

        # Process noise
        self.Q = np.eye(6, dtype=np.float64) * 0.1
        self.Q[3:, 3:] *= 1.0

        # Measurement noise
        self.R = np.eye(3, dtype=np.float64) * 0.1

        # State transition (constant velocity)
        self.dt = dt
        self.F = np.eye(6, dtype=np.float64)
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt

        # Measurement matrix (observe position only)
        self.H = np.zeros((3, 6), dtype=np.float64)
        self.H[:3, :3] = np.eye(3)

    def predict(self) -> np.ndarray:
        """Predict next state.

        Returns:
            Predicted position [x, y, z]
        """
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.age += 1
        self.time_since_update += 1
        return self.state[:3].copy()

    def update(self, measurement: np.ndarray):
        """Update state with new measurement.

        Args:
            measurement: [x, y, z] observed position
        """
        # Innovation
        y = measurement - self.H @ self.state

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ y

        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(6) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

        self.hits += 1
        self.time_since_update = 0

    def get_position(self) -> np.ndarray:
        """Get current estimated position."""
        return self.state[:3].copy()

    def get_velocity(self) -> np.ndarray:
        """Get current estimated velocity."""
        return self.state[3:].copy()


class PoleTracker:
    """Multi-object tracker for poles using Kalman filters."""

    def __init__(self, config: PoleDetectorConfig):
        """Initialize tracker.

        Args:
            config: PoleDetectorConfig with tracking parameters
        """
        self.config = config
        self.trackers: Dict[int, KalmanPoleTracker] = {}
        self.next_id = 0

    def update(self, detections: List[dict]) -> List[PoleDetection]:
        """Update trackers with new detections.

        Args:
            detections: List of raw detection dicts from cylinder fitting

        Returns:
            List of PoleDetection with stable IDs
        """
        # Step 1: Predict all existing trackers
        for tracker in self.trackers.values():
            tracker.predict()

        # Step 2: Associate detections to trackers
        if len(self.trackers) > 0 and len(detections) > 0:
            matches, unmatched_dets, unmatched_trks = self._associate(detections)
        else:
            matches = []
            unmatched_dets = list(range(len(detections)))
            unmatched_trks = list(self.trackers.keys())

        # Step 3: Update matched trackers
        matched_det_indices = set()
        for det_idx, trk_id in matches:
            det = detections[det_idx]
            pos = np.array([det['x'], det['y'], det['z']])
            self.trackers[trk_id].update(pos)
            matched_det_indices.add(det_idx)

        # Step 4: Create new trackers for unmatched detections
        new_tracker_map = {}  # det_idx -> trk_id for O(1) lookup
        for det_idx in unmatched_dets:
            det = detections[det_idx]
            pos = np.array([det['x'], det['y'], det['z']])
            new_id = self.next_id
            self.trackers[new_id] = KalmanPoleTracker(pos, new_id)
            new_tracker_map[det_idx] = new_id
            self.next_id += 1

        # Step 5: Remove stale trackers
        to_remove = []
        for trk_id, tracker in self.trackers.items():
            if tracker.time_since_update > self.config.track_max_age:
                to_remove.append(trk_id)
        for trk_id in to_remove:
            del self.trackers[trk_id]

        # Step 6: Generate output - only confirmed tracks
        return self._generate_output(detections, matches, new_tracker_map)

    def _associate(self, detections: List[dict]) -> Tuple[List, List, List]:
        """Associate detections to existing trackers.

        Uses Hungarian algorithm if scipy available, else greedy matching.

        Returns:
            Tuple of (matches, unmatched_detections, unmatched_trackers)
        """
        trk_ids = list(self.trackers.keys())
        n_det = len(detections)
        n_trk = len(trk_ids)

        # Build cost matrix (Euclidean distance)
        cost_matrix = np.zeros((n_det, n_trk), dtype=np.float64)

        for i, det in enumerate(detections):
            det_pos = np.array([det['x'], det['y'], det['z']])
            for j, trk_id in enumerate(trk_ids):
                trk_pos = self.trackers[trk_id].get_position()
                cost_matrix[i, j] = np.linalg.norm(det_pos - trk_pos)

        # Find optimal assignment
        if SCIPY_AVAILABLE:
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
        else:
            # Greedy matching fallback
            row_ind, col_ind = self._greedy_assignment(cost_matrix)

        # Filter by threshold and build output
        matches = []
        unmatched_dets = set(range(n_det))
        unmatched_trks = set(trk_ids)

        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] < self.config.track_association_threshold:
                matches.append((r, trk_ids[c]))
                unmatched_dets.discard(r)
                unmatched_trks.discard(trk_ids[c])

        return matches, list(unmatched_dets), list(unmatched_trks)

    def _greedy_assignment(self, cost_matrix: np.ndarray) -> Tuple[List, List]:
        """Greedy assignment fallback when scipy unavailable."""
        n_det, n_trk = cost_matrix.shape
        row_ind = []
        col_ind = []

        used_rows = set()
        used_cols = set()

        # Sort all pairs by cost
        pairs = []
        for i in range(n_det):
            for j in range(n_trk):
                pairs.append((cost_matrix[i, j], i, j))
        pairs.sort()

        for cost, i, j in pairs:
            if i not in used_rows and j not in used_cols:
                row_ind.append(i)
                col_ind.append(j)
                used_rows.add(i)
                used_cols.add(j)

        return row_ind, col_ind

    def _generate_output(self, detections: List[dict],
                         matches: List[Tuple],
                         new_tracker_map: Dict[int, int]) -> List[PoleDetection]:
        """Generate PoleDetection output list.

        Args:
            detections: List of raw detection dicts
            matches: List of (det_idx, trk_id) tuples for matched detections
            new_tracker_map: Dict mapping det_idx -> trk_id for new detections

        Returns:
            List of PoleDetection objects sorted by confidence
        """
        output = []

        # Map detection index to tracker ID for matched detections
        det_to_trk = {det_idx: trk_id for det_idx, trk_id in matches}

        for det_idx, det in enumerate(detections):
            if det_idx in det_to_trk:
                trk_id = det_to_trk[det_idx]
                tracker = self.trackers[trk_id]

                # Only output confirmed tracks (enough hits)
                if tracker.hits >= self.config.track_min_hits:
                    # Use filtered position from Kalman
                    pos = tracker.get_position()
                    output.append(PoleDetection(
                        id=trk_id,
                        x=float(pos[0]),
                        y=float(pos[1]),
                        z=float(pos[2]),
                        radius=det['radius'],
                        confidence=det['confidence'],
                        velocity=det.get('velocity', 0.0),
                        height=det.get('height', 0.0),
                        z_min=det.get('z_min', 0.0),
                        z_max=det.get('z_max', 0.0),
                        verticality=det.get('verticality', 0.0),
                        num_points=det.get('num_points', 0),
                        range_xy=det.get('range', 0.0)
                    ))
            elif det_idx in new_tracker_map:
                # New detection - use direct O(1) lookup instead of scanning
                trk_id = new_tracker_map[det_idx]
                output.append(PoleDetection(
                    id=trk_id,
                    x=float(det['x']),
                    y=float(det['y']),
                    z=float(det['z']),
                    radius=det['radius'],
                    confidence=det['confidence'] * 0.5,  # Lower confidence for unconfirmed
                    velocity=det.get('velocity', 0.0),
                    height=det.get('height', 0.0),
                    z_min=det.get('z_min', 0.0),
                    z_max=det.get('z_max', 0.0),
                    verticality=det.get('verticality', 0.0),
                    num_points=det.get('num_points', 0),
                    range_xy=det.get('range', 0.0)
                ))

        # Sort by confidence
        output.sort(key=lambda p: p.confidence, reverse=True)
        return output

    def reset(self):
        """Clear all tracking state."""
        self.trackers.clear()
        self.next_id = 0


# =============================================================================
# Main Pole Detector
# =============================================================================

class PoleDetector:
    """LiDAR-based pole detector with tracking.

    Provides a clean interface for detecting poles from point cloud data.
    Supports both ROS PointCloud2 messages and direct numpy array input.

    Example:
        detector = PoleDetector(config_path='config/settings.yaml')
        poles = detector.detect(points)  # Nx5 numpy array

        for pole in poles:
            print(f"Pole {pole.id} at ({pole.x:.2f}, {pole.y:.2f})")
    """

    def __init__(self, config: Optional[PoleDetectorConfig] = None,
                 config_path: Optional[str] = None):
        """Initialize detector.

        Args:
            config: PoleDetectorConfig object
            config_path: Path to YAML config file
        """
        if config is not None:
            self.config = config
        elif config_path is not None:
            self.config = self._load_config(config_path)
        else:
            self.config = PoleDetectorConfig()  # Defaults

        self.tracker = PoleTracker(self.config)
        self._last_ground_plane: Optional[Tuple[np.ndarray, float]] = None

        # Performance tracking
        self._timing = {
            'velocity_filter': 0.0,
            'ground_removal': 0.0,
            'clustering': 0.0,
            'fitting': 0.0,
            'tracking': 0.0,
            'total': 0.0
        }

    def _load_config(self, config_path: str) -> PoleDetectorConfig:
        """Load configuration from YAML file."""
        config = PoleDetectorConfig()

        if not YAML_AVAILABLE:
            print(f"Warning: yaml not available, using defaults")
            return config

        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)

            if 'pole_detector' in data:
                pd = data['pole_detector']

                # Performance
                config.downsample_factor = pd.get('downsample_factor', config.downsample_factor)
                config.max_points = pd.get('max_points', config.max_points)

                # RANSAC
                config.ransac_threshold = pd.get('ransac_threshold', config.ransac_threshold)
                config.ransac_iterations = pd.get('ransac_iterations', config.ransac_iterations)
                config.ransac_sample_size = pd.get('ransac_sample_size', config.ransac_sample_size)
                config.ground_height_margin = pd.get('ground_height_margin', config.ground_height_margin)

                # Clustering
                config.cluster_eps = pd.get('cluster_eps', config.cluster_eps)
                config.cluster_min_samples = pd.get('cluster_min_samples', config.cluster_min_samples)

                # Cylinder validation
                config.pole_min_height = pd.get('pole_min_height', config.pole_min_height)
                config.pole_max_height = pd.get('pole_max_height', config.pole_max_height)
                config.pole_min_radius = pd.get('pole_min_radius', config.pole_min_radius)
                config.pole_max_radius = pd.get('pole_max_radius', config.pole_max_radius)
                config.pole_verticality = pd.get('pole_verticality', config.pole_verticality)
                config.pole_min_aspect = pd.get('pole_min_aspect', config.pole_min_aspect)

                # Performance limits
                config.max_clusters = pd.get('max_clusters', config.max_clusters)
                config.max_points_per_cluster = pd.get('max_points_per_cluster', config.max_points_per_cluster)

                # Range
                config.min_range = pd.get('min_range', config.min_range)
                config.max_range = pd.get('max_range', config.max_range)

                # Tracking
                config.track_max_age = pd.get('track_max_age', config.track_max_age)
                config.track_min_hits = pd.get('track_min_hits', config.track_min_hits)
                config.track_association_threshold = pd.get('track_association_threshold',
                                                           config.track_association_threshold)

                # Moving object rejection
                config.velocity_filter_enabled = pd.get('velocity_filter_enabled', config.velocity_filter_enabled)
                config.max_point_velocity = pd.get('max_point_velocity', config.max_point_velocity)
                config.max_cluster_velocity = pd.get('max_cluster_velocity', config.max_cluster_velocity)
                config.velocity_outlier_ratio = pd.get('velocity_outlier_ratio', config.velocity_outlier_ratio)

                # Ground plane stability
                config.ground_plane_smoothing = pd.get('ground_plane_smoothing', config.ground_plane_smoothing)

        except Exception as e:
            print(f"Warning: Could not load config from {config_path}: {e}")

        return config

    def detect(self, points: np.ndarray, track: bool = True) -> List[PoleDetection]:
        """Detect poles in point cloud.

        Main entry point for pole detection.

        Args:
            points: Nx5 numpy array (x, y, z, velocity, reflectance)
                   Coordinate system: X=forward, Y=left, Z=up
            track: Whether to apply Kalman tracking for stable IDs

        Returns:
            List of PoleDetection objects, sorted by confidence
        """
        t_start = time.time()

        # Handle edge cases
        if points is None or len(points) < self.config.cluster_min_samples:
            if track:
                return self.tracker.update([])
            return []

        # Ensure correct shape
        points = np.asarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] < 3:
            return []

        # Pad with zeros for velocity and reflectance if needed
        if points.shape[1] < 5:
            padding = np.zeros((len(points), 5 - points.shape[1]), dtype=np.float32)
            points = np.hstack([points, padding])

        # === PERFORMANCE OPTIMIZATIONS ===

        # Step 0a: Range filtering (remove points outside detection range)
        # OPTIMIZED: np.hypot is faster and more numerically stable
        range_xy = np.hypot(points[:, 0], points[:, 1])
        range_mask = (range_xy >= self.config.min_range) & (range_xy <= self.config.max_range)
        points = points[range_mask]

        if len(points) < self.config.cluster_min_samples:
            if track:
                return self.tracker.update([])
            return []

        # Step 0b: Downsampling (take every Nth point)
        if self.config.downsample_factor > 1 and len(points) > self.config.max_points:
            step = self.config.downsample_factor
            points = points[::step]

        # Step 0c: Cap total points for bounded latency
        if len(points) > self.config.max_points:
            indices = np.random.choice(len(points), self.config.max_points, replace=False)
            points = points[indices]

        # Store processed point count for debug
        self._processed_points = len(points)

        # Step 1: Filter moving points (FMCW velocity)
        t0 = time.time()
        static_points = self.filter_moving_points(points)
        self._timing['velocity_filter'] = time.time() - t0

        if len(static_points) < self.config.cluster_min_samples:
            if track:
                return self.tracker.update([])
            return []

        # Step 2: Ground removal (RANSAC)
        t0 = time.time()
        non_ground = self.remove_ground_ransac(static_points)
        self._timing['ground_removal'] = time.time() - t0

        if len(non_ground) < self.config.cluster_min_samples:
            if track:
                return self.tracker.update([])
            return []

        # Step 3: Clustering (DBSCAN)
        t0 = time.time()
        clusters = self.cluster_points(non_ground)
        self._timing['clustering'] = time.time() - t0

        # Step 4: Cylinder fitting
        t0 = time.time()
        raw_detections = []
        for cluster in clusters:
            result = self.fit_cylinder(cluster)
            if result is not None:
                raw_detections.append(result)
        self._timing['fitting'] = time.time() - t0

        # Step 5: Tracking
        t0 = time.time()
        if track:
            poles = self.tracker.update(raw_detections)
        else:
            poles = [
                PoleDetection(
                    id=-1,
                    x=d['x'], y=d['y'], z=d['z'],
                    radius=d['radius'],
                    confidence=d['confidence'],
                    velocity=d.get('velocity', 0.0),
                    height=d.get('height', 0.0),
                    z_min=d.get('z_min', 0.0),
                    z_max=d.get('z_max', 0.0),
                    verticality=d.get('verticality', 0.0),
                    num_points=d.get('num_points', 0),
                    range_xy=d.get('range', 0.0)
                )
                for d in raw_detections
            ]
        self._timing['tracking'] = time.time() - t0

        self._timing['total'] = time.time() - t_start

        return poles

    def remove_ground_ransac(self, points: np.ndarray) -> np.ndarray:
        """Remove ground points using RANSAC plane fitting.

        The Aeva coordinate system uses Z=up, so ground plane should have
        normal vector close to [0, 0, 1].

        Args:
            points: Nx5 array (x, y, z, velocity, reflectance)

        Returns:
            Points with ground removed (Mx5 array)
        """
        if len(points) < 3:
            return points

        xyz = points[:, :3]

        # Use bottom 30% of points (by Z) as ground candidates for efficiency
        z_threshold = np.percentile(xyz[:, 2], 30)
        ground_candidates_mask = xyz[:, 2] <= z_threshold
        ground_candidates = xyz[ground_candidates_mask]

        if len(ground_candidates) < 3:
            # Fall back to using all points
            ground_candidates = xyz

        # Subsample ground candidates for faster RANSAC
        if len(ground_candidates) > self.config.ransac_sample_size:
            indices = np.random.choice(
                len(ground_candidates),
                self.config.ransac_sample_size,
                replace=False
            )
            ground_candidates = ground_candidates[indices]

        # RANSAC plane fitting
        best_inliers = 0
        best_plane = None
        n_points = len(ground_candidates)

        for _ in range(self.config.ransac_iterations):
            # Sample 3 random points
            if n_points < 3:
                break
            idx = np.random.choice(n_points, 3, replace=False)
            p1, p2, p3 = ground_candidates[idx]

            # Compute plane normal via cross product
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)

            if norm < 1e-6:
                continue  # Degenerate triangle

            normal = normal / norm

            # Ensure normal points up (positive Z)
            if normal[2] < 0:
                normal = -normal

            # Check if this is a horizontal plane (normal ~= [0, 0, 1])
            if abs(normal[2]) < 0.9:
                continue  # Not a ground plane

            # Plane equation: n . (p - p1) = 0
            # => n . p = n . p1 = -d
            d = -np.dot(normal, p1)

            # Count inliers (points close to plane)
            distances = np.abs(xyz @ normal + d)
            inliers = np.sum(distances < self.config.ransac_threshold)

            if inliers > best_inliers:
                best_inliers = inliers
                best_plane = (normal, d)

                # Early termination if we have enough inliers
                if inliers > 0.5 * len(ground_candidates):
                    break

        if best_plane is None:
            # No plane found, fall back to simple height threshold
            z_min = np.percentile(xyz[:, 2], 5)
            mask = xyz[:, 2] > (z_min + self.config.ground_height_margin)
            self._last_ground_plane = None
            return points[mask]

        normal, d = best_plane

        # Apply temporal smoothing (EMA) to reduce jitter
        if self._last_ground_plane is not None and self.config.ground_plane_smoothing > 0:
            prev_normal, prev_d = self._last_ground_plane
            # Only smooth if planes are similar (avoid smoothing across large changes)
            if np.dot(normal, prev_normal) > 0.95:
                alpha = self.config.ground_plane_smoothing
                normal = alpha * prev_normal + (1 - alpha) * normal
                normal = normal / np.linalg.norm(normal)  # Re-normalize
                d = alpha * prev_d + (1 - alpha) * d

        self._last_ground_plane = (normal, d)

        # Remove points within ground_height_margin of the plane
        signed_distances = xyz @ normal + d
        mask = signed_distances > self.config.ground_height_margin

        return points[mask]

    def filter_moving_points(self, points: np.ndarray) -> np.ndarray:
        """Remove points with high velocity (moving objects).

        Uses FMCW velocity data to filter out moving objects like people,
        vehicles, or other dynamic obstacles that should not be detected
        as poles.

        Args:
            points: Nx5 array (x, y, z, velocity, reflectance)

        Returns:
            Filtered points array with moving points removed
        """
        if not self.config.velocity_filter_enabled:
            return points

        if points.shape[1] < 4:
            return points  # No velocity data available

        velocities = np.abs(points[:, 3])
        static_mask = velocities <= self.config.max_point_velocity

        return points[static_mask]

    def cluster_points(self, points: np.ndarray) -> List[np.ndarray]:
        """Cluster non-ground points using DBSCAN.

        Clusters based on XY position (horizontal plane) since poles
        are vertical structures. This improves clustering when poles
        have varying heights.

        Args:
            points: Nx5 array (x, y, z, velocity, reflectance)

        Returns:
            List of point clusters (each Mx5 array)
        """
        if not SKLEARN_AVAILABLE:
            return []

        if len(points) < self.config.cluster_min_samples:
            return []

        # Cluster on XY only (poles are vertical)
        xy = points[:, :2]

        clustering = DBSCAN(
            eps=self.config.cluster_eps,
            min_samples=self.config.cluster_min_samples,
            n_jobs=1  # Single thread for deterministic, low-latency
        ).fit(xy)

        labels = clustering.labels_
        unique_labels = set(labels) - {-1}  # Exclude noise

        clusters = []
        for label in unique_labels:
            mask = labels == label
            cluster = points[mask]
            if len(cluster) >= self.config.cluster_min_samples:
                clusters.append(cluster)

        # Sort by size (largest first) and limit count
        clusters.sort(key=len, reverse=True)
        if len(clusters) > self.config.max_clusters:
            clusters = clusters[:self.config.max_clusters]

        return clusters

    def fit_cylinder(self, cluster: np.ndarray) -> Optional[dict]:
        """Fit cylinder model to point cluster.

        Uses PCA to find principal axis (should be vertical for poles).
        Then estimates radius using least-squares in the horizontal plane.

        Args:
            cluster: Nx5 array of points

        Returns:
            dict with cylinder parameters or None if not a valid pole
        """
        if len(cluster) < self.config.cluster_min_samples:
            return None

        # Subsample large clusters for speed
        if len(cluster) > self.config.max_points_per_cluster:
            indices = np.random.choice(
                len(cluster),
                self.config.max_points_per_cluster,
                replace=False
            )
            cluster = cluster[indices]

        xyz = cluster[:, :3]

        # Compute centroid and bounds
        centroid = np.mean(xyz, axis=0)
        mins = np.min(xyz, axis=0)
        maxs = np.max(xyz, axis=0)
        height = maxs[2] - mins[2]

        # Check range bounds
        range_xy = np.sqrt(centroid[0]**2 + centroid[1]**2)
        if range_xy < self.config.min_range or range_xy > self.config.max_range:
            return None

        # Check height bounds
        if height < self.config.pole_min_height or height > self.config.pole_max_height:
            return None

        # PCA to find principal axis
        centered = xyz - centroid
        try:
            cov = np.cov(centered.T)
            if cov.ndim < 2:
                return None
            eigenvalues, eigenvectors = np.linalg.eigh(cov)

            # Principal axis = eigenvector with largest eigenvalue
            principal_axis = eigenvectors[:, np.argmax(eigenvalues)]

            # Ensure axis points upward
            if principal_axis[2] < 0:
                principal_axis = -principal_axis
        except Exception:
            return None

        # Check verticality (dot product with Z axis)
        verticality = abs(principal_axis[2])
        if verticality < self.config.pole_verticality:
            return None

        # Estimate radius from XY spread (not centroid distance)
        # For side-view LiDAR, use minimum spread as diameter estimate
        # This avoids underestimation caused by viewing only one side of pole
        xy_centered = xyz[:, :2] - np.median(xyz[:, :2], axis=0)

        # Use robust percentile range (2.5% to 97.5%) to handle outliers
        range_x = np.percentile(xy_centered[:, 0], 97.5) - np.percentile(xy_centered[:, 0], 2.5)
        range_y = np.percentile(xy_centered[:, 1], 97.5) - np.percentile(xy_centered[:, 1], 2.5)

        # Use minimum spread (perpendicular to viewing direction approximates diameter)
        diameter = min(range_x, range_y)
        radius = diameter / 2

        # Compute std for consistency score (using horizontal distances for compatibility)
        horizontal_distances = np.sqrt(xy_centered[:, 0]**2 + xy_centered[:, 1]**2)
        radius_std = np.std(horizontal_distances)

        # Check radius bounds
        if radius < self.config.pole_min_radius or radius > self.config.pole_max_radius:
            return None

        # Check aspect ratio (height / diameter)
        aspect_ratio = height / (2 * radius + 1e-6)
        if aspect_ratio < self.config.pole_min_aspect:
            return None

        # Extract velocity from FMCW data and check for moving objects
        if cluster.shape[1] > 3:
            velocities = cluster[:, 3]
            velocity = float(np.median(velocities))

            # Reject clusters that are moving (not static poles)
            if self.config.velocity_filter_enabled:
                if abs(velocity) > self.config.max_cluster_velocity:
                    return None

                # Check ratio of high-velocity points in cluster
                high_vel_ratio = np.mean(np.abs(velocities) > self.config.max_point_velocity)
                if high_vel_ratio > self.config.velocity_outlier_ratio:
                    return None
        else:
            velocity = 0.0

        # Compute confidence score with range normalization
        # Expected points decrease quadratically with range (LiDAR geometry)
        expected_points = max(10, 200 / (1 + (range_xy / 10) ** 2))
        point_score = min(1.0, len(cluster) / expected_points)

        vert_score = verticality  # Already 0-1, higher is more vertical

        # Aspect ratio: poles typically 5:1 to 20:1 height/diameter
        aspect_score = min(1.0, max(0.0, (aspect_ratio - 3) / 7))  # 0 at 3:1, 1 at 10:1

        # Radius consistency (more tolerance at range due to beam divergence)
        range_factor = 1 + range_xy / 20
        radius_consistency = 1.0 / (1.0 + (radius_std / (radius + 1e-6)) / range_factor)

        # Weighted combination (verticality most important for pole detection)
        confidence = (
            0.20 * point_score +
            0.35 * vert_score +
            0.25 * aspect_score +
            0.20 * radius_consistency
        )

        return {
            'x': float(centroid[0]),
            'y': float(centroid[1]),
            'z': float(centroid[2]),
            'z_min': float(mins[2]),
            'z_max': float(maxs[2]),
            'height': float(height),
            'radius': float(radius),
            'confidence': float(confidence),
            'num_points': len(cluster),
            'verticality': float(verticality),
            'range': float(range_xy),
            'velocity': velocity
        }

    def detect_from_ros(self, msg) -> List[PoleDetection]:
        """Process ROS PointCloud2 message.

        Convenience method for ROS integration.

        Args:
            msg: sensor_msgs/PointCloud2 message

        Returns:
            List of PoleDetection objects
        """
        try:
            import sensor_msgs.point_cloud2 as pc2
        except ImportError:
            print("Warning: sensor_msgs not available")
            return []

        points = []
        for point in pc2.read_points(msg, skip_nans=True):
            if len(point) >= 3:
                x, y, z = point[0], point[1], point[2]
                vel = point[3] if len(point) > 3 else 0.0
                refl = point[4] if len(point) > 4 else 0.0
                points.append([x, y, z, vel, refl])

        if not points:
            return []

        points_array = np.array(points, dtype=np.float32)
        return self.detect(points_array)

    def get_timing(self) -> dict:
        """Get timing breakdown for last detection.

        Returns:
            dict with timing for each stage (in seconds)
        """
        return self._timing.copy()

    def get_ground_plane(self) -> Optional[Tuple[np.ndarray, float]]:
        """Get last fitted ground plane parameters.

        Returns:
            Tuple of (normal, d) for plane equation n.p + d = 0,
            or None if no plane was fitted
        """
        return self._last_ground_plane

    def reset_tracker(self):
        """Reset all tracking state."""
        self.tracker.reset()


# =============================================================================
# Convenience Functions
# =============================================================================

def create_detector(config_path: str = 'config/settings.yaml') -> PoleDetector:
    """Create a PoleDetector with default or file-based configuration.

    Args:
        config_path: Path to YAML configuration file

    Returns:
        Configured PoleDetector instance
    """
    return PoleDetector(config_path=config_path)
