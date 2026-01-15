"""
Sensor Fusion Module

Combines LiDAR pole detections with camera (YOLO) detections to produce
high-confidence fused pole estimates.

Features:
- Camera-LiDAR projection using calibrated extrinsics
- Hungarian algorithm for optimal detection association
- Weighted confidence blending
- Temporal filtering (N of M frames)
- False positive blacklisting by location

Usage:
    from fusion import SensorFusion, FusionConfig

    fusion = SensorFusion(config)
    fused_poles = fusion.fuse(lidar_poles, camera_detections, camera_shape)
"""

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    from scipy.optimize import linear_sum_assignment
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class FusedPole:
    """Output format for a fused pole detection."""
    id: int                         # Stable tracking ID from LiDAR tracker
    x: float                        # X position (forward, meters)
    y: float                        # Y position (left, meters)
    z: float                        # Z position (centroid height, meters)
    radius: float                   # Estimated pole radius (meters)

    # Confidence scores
    lidar_confidence: float         # Raw LiDAR detection confidence (0-1)
    camera_confidence: float        # Camera/YOLO confidence (0-1, or 0 if no match)
    fused_confidence: float         # Weighted combined confidence (0-1)

    # Association info
    camera_matched: bool            # Whether this pole matched a camera detection
    camera_class: str = ""          # YOLO class name if matched
    image_u: int = 0                # Projected image X coordinate
    image_v: int = 0                # Projected image Y coordinate

    # From LiDAR
    height: float = 0.0             # Pole height (meters)
    velocity: float = 0.0           # Radial velocity (m/s)
    range_xy: float = 0.0           # Horizontal distance from sensor

    # Temporal info
    frames_seen: int = 1            # Frames this pole has been tracked
    frames_since_camera: int = 0    # Frames since last camera match


@dataclass
class FusionConfig:
    """Configuration for sensor fusion."""

    # Camera intrinsics (default for 1280x720 USB camera)
    camera_fx: float = 800.0        # Focal length X (pixels)
    camera_fy: float = 800.0        # Focal length Y (pixels)
    camera_cx: float = 640.0        # Principal point X (pixels)
    camera_cy: float = 360.0        # Principal point Y (pixels)

    # Camera-LiDAR extrinsics (transform from LiDAR to camera frame)
    # Default: camera ~10cm above and 5cm behind LiDAR, looking forward
    extrinsic_tx: float = 0.0       # Translation X (forward) in meters
    extrinsic_ty: float = 0.0       # Translation Y (left) in meters
    extrinsic_tz: float = -0.1      # Translation Z (up) in meters
    extrinsic_roll: float = 0.0     # Roll (degrees)
    extrinsic_pitch: float = 0.0    # Pitch (degrees)
    extrinsic_yaw: float = 0.0      # Yaw (degrees)

    # Association thresholds
    association_threshold_px: float = 150.0  # Max pixel distance for matching
    min_projection_depth: float = 0.5        # Min depth for valid projection (m)

    # Confidence weighting
    weight_lidar: float = 0.6       # Weight for LiDAR confidence
    weight_camera: float = 0.4      # Weight for camera confidence
    camera_boost: float = 0.2       # Confidence boost when camera matches

    # Temporal filtering
    temporal_window: int = 10       # Frames to track history
    min_frames_seen: int = 2        # Min frames to output pole
    camera_hold_frames: int = 5     # Frames to retain camera match status

    # Output gating
    min_fused_confidence: float = 0.3  # Min confidence to output

    # Blacklist (false positive suppression)
    blacklist_enabled: bool = True
    blacklist_radius: float = 0.5   # Radius for location hash (meters)
    blacklist_threshold: int = 50   # Frames before blacklisting location


# =============================================================================
# Sensor Fusion
# =============================================================================

class SensorFusion:
    """Fuses LiDAR and camera pole detections."""

    def __init__(self, config: Optional[FusionConfig] = None,
                 config_path: Optional[str] = None):
        """Initialize fusion module.

        Args:
            config: FusionConfig object
            config_path: Path to YAML config file
        """
        if config is not None:
            self.config = config
        elif config_path is not None:
            self.config = self._load_config(config_path)
        else:
            self.config = FusionConfig()

        # Build camera projection matrix
        self._update_projection_matrix()

        # Temporal tracking
        self._pole_history: Dict[int, deque] = {}  # pole_id -> deque of (timestamp, camera_matched)
        self._camera_match_age: Dict[int, int] = {}  # pole_id -> frames since camera match

        # Blacklist for false positives
        self._location_counts: Dict[Tuple[int, int], int] = {}  # (grid_x, grid_y) -> count
        self._blacklist: set = set()  # Set of blacklisted grid cells

        # Performance tracking
        self._timing = {
            'projection': 0.0,
            'association': 0.0,
            'fusion': 0.0,
            'total': 0.0
        }

    def _load_config(self, config_path: str) -> FusionConfig:
        """Load configuration from YAML file."""
        config = FusionConfig()

        if not YAML_AVAILABLE:
            return config

        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)

            if 'sensor_fusion' in data:
                sf = data['sensor_fusion']

                # Camera intrinsics
                config.camera_fx = sf.get('camera_fx', config.camera_fx)
                config.camera_fy = sf.get('camera_fy', config.camera_fy)
                config.camera_cx = sf.get('camera_cx', config.camera_cx)
                config.camera_cy = sf.get('camera_cy', config.camera_cy)

                # Extrinsics
                config.extrinsic_tx = sf.get('extrinsic_tx', config.extrinsic_tx)
                config.extrinsic_ty = sf.get('extrinsic_ty', config.extrinsic_ty)
                config.extrinsic_tz = sf.get('extrinsic_tz', config.extrinsic_tz)
                config.extrinsic_roll = sf.get('extrinsic_roll', config.extrinsic_roll)
                config.extrinsic_pitch = sf.get('extrinsic_pitch', config.extrinsic_pitch)
                config.extrinsic_yaw = sf.get('extrinsic_yaw', config.extrinsic_yaw)

                # Association
                config.association_threshold_px = sf.get('association_threshold_px',
                                                         config.association_threshold_px)

                # Weights
                config.weight_lidar = sf.get('weight_lidar', config.weight_lidar)
                config.weight_camera = sf.get('weight_camera', config.weight_camera)
                config.camera_boost = sf.get('camera_boost', config.camera_boost)

                # Temporal
                config.temporal_window = sf.get('temporal_window', config.temporal_window)
                config.min_frames_seen = sf.get('min_frames_seen', config.min_frames_seen)
                config.camera_hold_frames = sf.get('camera_hold_frames', config.camera_hold_frames)

                # Output
                config.min_fused_confidence = sf.get('min_fused_confidence',
                                                     config.min_fused_confidence)

        except Exception as e:
            print(f"Warning: Could not load fusion config: {e}")

        return config

    def _update_projection_matrix(self):
        """Build camera intrinsic matrix and LiDAR->camera transform."""
        # Camera intrinsic matrix
        self.K = np.array([
            [self.config.camera_fx, 0, self.config.camera_cx],
            [0, self.config.camera_fy, self.config.camera_cy],
            [0, 0, 1]
        ], dtype=np.float64)

        # Build rotation matrix from Euler angles
        roll = np.deg2rad(self.config.extrinsic_roll)
        pitch = np.deg2rad(self.config.extrinsic_pitch)
        yaw = np.deg2rad(self.config.extrinsic_yaw)

        # Rotation matrices
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # Combined rotation (ZYX order)
        self.R_lidar_to_cam = Rz @ Ry @ Rx

        # Translation vector
        self.t_lidar_to_cam = np.array([
            self.config.extrinsic_tx,
            self.config.extrinsic_ty,
            self.config.extrinsic_tz
        ])

        # Coordinate transform: Aeva (X=fwd, Y=left, Z=up) -> Camera (X=right, Y=down, Z=fwd)
        # This is a 90-degree rotation
        self.R_aeva_to_cam = np.array([
            [0, -1, 0],   # Camera X = -Aeva Y (right = -left)
            [0, 0, -1],   # Camera Y = -Aeva Z (down = -up)
            [1, 0, 0]     # Camera Z = Aeva X (forward = forward)
        ], dtype=np.float64)

    def project_to_image(self, x: float, y: float, z: float,
                         image_width: int, image_height: int) -> Optional[Tuple[int, int]]:
        """Project a 3D point (in LiDAR frame) to image coordinates.

        Args:
            x, y, z: Point in Aeva LiDAR frame (X=fwd, Y=left, Z=up)
            image_width, image_height: Image dimensions

        Returns:
            (u, v) pixel coordinates, or None if behind camera or outside image
        """
        # Point in LiDAR frame
        p_lidar = np.array([x, y, z])

        # Apply extrinsic transform (LiDAR -> camera mount)
        p_cam_mount = self.R_lidar_to_cam @ p_lidar + self.t_lidar_to_cam

        # Apply coordinate system transform (Aeva -> camera convention)
        p_cam = self.R_aeva_to_cam @ p_cam_mount

        # Check if in front of camera
        if p_cam[2] < self.config.min_projection_depth:
            return None

        # Project to image plane
        p_img = self.K @ p_cam
        u = int(p_img[0] / p_img[2])
        v = int(p_img[1] / p_img[2])

        # Check bounds
        if 0 <= u < image_width and 0 <= v < image_height:
            return (u, v)

        return None

    def _associate_detections(self, lidar_poles: List,
                              camera_dets: List[dict],
                              image_width: int,
                              image_height: int) -> Dict[int, dict]:
        """Associate LiDAR poles with camera detections.

        Uses Hungarian algorithm for optimal matching based on
        pixel distance between projected LiDAR poles and YOLO bbox centers.

        Args:
            lidar_poles: List of PoleDetection from LiDAR
            camera_dets: List of camera detection dicts with 'bbox', 'conf', 'class_name'
            image_width, image_height: Image dimensions

        Returns:
            Dict mapping pole.id -> camera detection dict (or empty if no match)
        """
        if not lidar_poles or not camera_dets:
            return {}

        # Project all LiDAR poles to image
        projections = {}  # pole_id -> (u, v)
        for pole in lidar_poles:
            proj = self.project_to_image(pole.x, pole.y, pole.z,
                                         image_width, image_height)
            if proj is not None:
                projections[pole.id] = proj

        if not projections:
            return {}

        # Get camera detection centers
        cam_centers = []
        for det in camera_dets:
            x1, y1, x2, y2 = det['bbox']
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            cam_centers.append((cx, cy))

        # Build cost matrix
        pole_ids = list(projections.keys())
        n_poles = len(pole_ids)
        n_cams = len(camera_dets)

        cost_matrix = np.full((n_poles, n_cams), 1e6, dtype=np.float64)

        # OPTIMIZED: Use squared distances to avoid n*m sqrt operations
        for i, pole_id in enumerate(pole_ids):
            pu, pv = projections[pole_id]
            for j, (cx, cy) in enumerate(cam_centers):
                dist_sq = (pu - cx)**2 + (pv - cy)**2
                cost_matrix[i, j] = dist_sq

        # Solve assignment (works with squared distances since we only need ordering)
        if SCIPY_AVAILABLE and n_poles > 0 and n_cams > 0:
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
        else:
            # Greedy fallback
            row_ind, col_ind = self._greedy_assignment(cost_matrix)

        # Build result, filtering by threshold (use squared threshold)
        threshold_sq = self.config.association_threshold_px ** 2
        result = {}
        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] < threshold_sq:
                pole_id = pole_ids[r]
                result[pole_id] = camera_dets[c]
                # Store projection for visualization
                result[pole_id]['_proj_u'], result[pole_id]['_proj_v'] = projections[pole_id]

        return result

    def _greedy_assignment(self, cost_matrix: np.ndarray) -> Tuple[List, List]:
        """Greedy assignment fallback."""
        n_rows, n_cols = cost_matrix.shape
        row_ind, col_ind = [], []
        used_rows, used_cols = set(), set()

        pairs = []
        for i in range(n_rows):
            for j in range(n_cols):
                pairs.append((cost_matrix[i, j], i, j))
        pairs.sort()

        for cost, i, j in pairs:
            if i not in used_rows and j not in used_cols:
                row_ind.append(i)
                col_ind.append(j)
                used_rows.add(i)
                used_cols.add(j)

        return row_ind, col_ind

    def _get_grid_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Get grid cell for blacklist lookup."""
        r = self.config.blacklist_radius
        return (int(x / r), int(y / r))

    def _update_blacklist(self, pole):
        """Update false positive blacklist based on low-confidence repeated detections."""
        if not self.config.blacklist_enabled:
            return

        cell = self._get_grid_cell(pole.x, pole.y)

        # Only track if low confidence and no camera match
        if pole.fused_confidence < 0.4 and not pole.camera_matched:
            self._location_counts[cell] = self._location_counts.get(cell, 0) + 1

            if self._location_counts[cell] > self.config.blacklist_threshold:
                self._blacklist.add(cell)

    def _is_blacklisted(self, x: float, y: float) -> bool:
        """Check if location is blacklisted."""
        if not self.config.blacklist_enabled:
            return False
        return self._get_grid_cell(x, y) in self._blacklist

    def fuse(self, lidar_poles: List,
             camera_detections: List[dict],
             image_shape: Tuple[int, int]) -> List[FusedPole]:
        """Fuse LiDAR and camera detections.

        Main entry point for sensor fusion.

        Args:
            lidar_poles: List of PoleDetection from LiDAR detector
            camera_detections: List of dicts with 'bbox', 'conf', 'class_name', 'class_id'
            image_shape: (height, width) of camera image

        Returns:
            List of FusedPole objects, sorted by confidence
        """
        t_start = time.time()
        img_h, img_w = image_shape

        # Step 1: Project and associate
        t0 = time.time()
        associations = self._associate_detections(
            lidar_poles, camera_detections, img_w, img_h
        )
        self._timing['association'] = time.time() - t0

        # Step 2: Fuse each pole
        t0 = time.time()
        fused_poles = []

        for pole in lidar_poles:
            # Check blacklist
            if self._is_blacklisted(pole.x, pole.y):
                continue

            # Get camera match if any
            cam_match = associations.get(pole.id)
            camera_matched = cam_match is not None
            camera_conf = cam_match['conf'] if camera_matched else 0.0
            camera_class = cam_match.get('class_name', '') if camera_matched else ''

            # Get projection coordinates
            proj = self.project_to_image(pole.x, pole.y, pole.z, img_w, img_h)
            image_u = proj[0] if proj else 0
            image_v = proj[1] if proj else 0

            # Update temporal tracking
            if pole.id not in self._pole_history:
                self._pole_history[pole.id] = deque(maxlen=self.config.temporal_window)
                self._camera_match_age[pole.id] = 999

            self._pole_history[pole.id].append((time.time(), camera_matched))

            if camera_matched:
                self._camera_match_age[pole.id] = 0
            else:
                self._camera_match_age[pole.id] += 1

            # Count frames seen
            frames_seen = len(self._pole_history[pole.id])

            # Compute fused confidence
            lidar_conf = pole.confidence

            # Apply camera boost if recently matched
            if self._camera_match_age[pole.id] < self.config.camera_hold_frames:
                effective_camera_conf = camera_conf if camera_matched else 0.5  # Retained confidence
                fused_conf = (
                    self.config.weight_lidar * lidar_conf +
                    self.config.weight_camera * effective_camera_conf +
                    self.config.camera_boost
                )
            else:
                fused_conf = self.config.weight_lidar * lidar_conf

            fused_conf = min(1.0, fused_conf)

            # Apply temporal boost for persistent detections
            if frames_seen >= self.config.min_frames_seen:
                temporal_boost = min(0.1, frames_seen * 0.02)
                fused_conf = min(1.0, fused_conf + temporal_boost)

            # Create fused pole
            fused_pole = FusedPole(
                id=pole.id,
                x=pole.x,
                y=pole.y,
                z=pole.z,
                radius=pole.radius,
                lidar_confidence=lidar_conf,
                camera_confidence=camera_conf,
                fused_confidence=fused_conf,
                camera_matched=camera_matched,
                camera_class=camera_class,
                image_u=image_u,
                image_v=image_v,
                height=pole.height,
                velocity=pole.velocity,
                range_xy=pole.range_xy,
                frames_seen=frames_seen,
                frames_since_camera=self._camera_match_age[pole.id]
            )

            # Update blacklist tracking
            self._update_blacklist(fused_pole)

            # Output gating
            if fused_conf >= self.config.min_fused_confidence:
                if frames_seen >= self.config.min_frames_seen:
                    fused_poles.append(fused_pole)

        self._timing['fusion'] = time.time() - t0

        # Cleanup old pole histories
        active_ids = {p.id for p in lidar_poles}
        stale_ids = [pid for pid in self._pole_history if pid not in active_ids]
        for pid in stale_ids:
            # Keep for a few frames in case it comes back
            if len(self._pole_history[pid]) > 0:
                age = time.time() - self._pole_history[pid][-1][0]
                if age > 2.0:  # 2 second timeout
                    del self._pole_history[pid]
                    if pid in self._camera_match_age:
                        del self._camera_match_age[pid]

        # Sort by confidence
        fused_poles.sort(key=lambda p: p.fused_confidence, reverse=True)

        self._timing['total'] = time.time() - t_start

        return fused_poles

    def get_timing(self) -> dict:
        """Get timing breakdown for last fusion."""
        return self._timing.copy()

    def clear_blacklist(self):
        """Clear the false positive blacklist."""
        self._blacklist.clear()
        self._location_counts.clear()

    def reset(self):
        """Reset all fusion state."""
        self._pole_history.clear()
        self._camera_match_age.clear()
        self.clear_blacklist()


# =============================================================================
# Convenience Functions
# =============================================================================

def create_fusion(config_path: str = 'config/settings.yaml') -> SensorFusion:
    """Create a SensorFusion with default or file-based configuration."""
    return SensorFusion(config_path=config_path)
