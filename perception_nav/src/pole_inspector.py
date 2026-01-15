"""
Pole Inspector Module

Handles pole inspection criteria, capture triggering, and deduplication.
Determines when a pole is "found" and prevents re-inspection of the same pole.

Features:
- Inspection criteria (confidence, tracking, camera match)
- Spatial deduplication via grid hashing
- Capture zone detection
- Inspection result generation with GPS coordinates
- Cross-session persistence (save/load inspected poles)

Usage:
    from pole_inspector import PoleInspector, InspectionConfig
    from gps_reader import JetsonGPS

    inspector = PoleInspector(config, gps)

    for pole in fused_poles:
        if inspector.should_inspect(pole):
            if inspector.is_in_capture_zone(pole):
                result = inspector.declare_found(pole, drone_heading)
                # Trigger photo capture, upload to AeroSync
"""

import json
import math
import time
import uuid
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Tuple, Set

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

# Import local modules (handle import errors gracefully for testing)
try:
    from .gps_reader import JetsonGPS
except ImportError:
    JetsonGPS = None


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class InspectionConfig:
    """Configuration for pole inspection."""
    # Confidence thresholds
    min_fused_confidence: float = 0.6      # Minimum fused confidence to consider
    min_frames_tracked: int = 10           # Minimum frames pole must be tracked

    # Camera requirement
    camera_match_required: bool = True     # Require camera (YOLO) confirmation

    # Capture zone
    capture_distance: float = 10.0         # Ideal distance for capture (m)
    max_approach_distance: float = 5.0     # Don't get closer than this (m)
    min_approach_distance: float = 15.0    # Must be at least this close (m)

    # Capture parameters
    dwell_time: float = 2.0                # Seconds to hover for capture
    photos_per_pole: int = 1               # Number of photos to capture

    # Deduplication
    dedup_grid_resolution: float = 5.0     # Grid cell size for spatial dedup (m)
    dedup_distance_threshold: float = 10.0 # Min distance between unique poles (m)

    # Persistence
    save_on_inspection: bool = True        # Auto-save after each inspection


@dataclass
class InspectionResult:
    """Result of a pole inspection."""
    pole_id: int                           # Internal tracking ID
    global_pole_id: str                    # UUID for AeroSync
    timestamp: float                       # Unix timestamp
    timestamp_iso: str                     # ISO format timestamp

    # Position
    position_local: Tuple[float, float, float]  # (x, y, z) in body/local frame
    position_gps: Tuple[float, float, float]    # (lat, lon, alt_msl)

    # Detection info
    confidence: float                      # Fused confidence
    lidar_confidence: float
    camera_confidence: float
    pole_type: str                         # YOLO class name
    pole_height: float                     # Estimated height (m)
    pole_radius: float                     # Estimated radius (m)

    # Capture info
    photos: List[str] = field(default_factory=list)  # Paths to photos
    lidar_snapshot: str = ""               # Path to PCD/numpy file

    # Metadata
    metadata: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {
            'pole_id': self.pole_id,
            'global_pole_id': self.global_pole_id,
            'timestamp': self.timestamp,
            'timestamp_iso': self.timestamp_iso,
            'position_local': list(self.position_local),
            'position_gps': list(self.position_gps),
            'confidence': self.confidence,
            'lidar_confidence': self.lidar_confidence,
            'camera_confidence': self.camera_confidence,
            'pole_type': self.pole_type,
            'pole_height': self.pole_height,
            'pole_radius': self.pole_radius,
            'photos': self.photos,
            'lidar_snapshot': self.lidar_snapshot,
            'metadata': self.metadata
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'InspectionResult':
        """Create from dictionary."""
        return cls(
            pole_id=data['pole_id'],
            global_pole_id=data['global_pole_id'],
            timestamp=data['timestamp'],
            timestamp_iso=data['timestamp_iso'],
            position_local=tuple(data['position_local']),
            position_gps=tuple(data['position_gps']),
            confidence=data['confidence'],
            lidar_confidence=data['lidar_confidence'],
            camera_confidence=data['camera_confidence'],
            pole_type=data['pole_type'],
            pole_height=data['pole_height'],
            pole_radius=data['pole_radius'],
            photos=data.get('photos', []),
            lidar_snapshot=data.get('lidar_snapshot', ''),
            metadata=data.get('metadata', {})
        )


# =============================================================================
# Pole Inspector
# =============================================================================

class PoleInspector:
    """Handles pole inspection criteria and deduplication."""

    def __init__(self, config: Optional[InspectionConfig] = None,
                 config_path: Optional[str] = None,
                 gps: Optional['JetsonGPS'] = None):
        """Initialize pole inspector.

        Args:
            config: InspectionConfig object
            config_path: Path to YAML config file
            gps: JetsonGPS instance for coordinate conversion
        """
        if config is not None:
            self.config = config
        elif config_path is not None:
            self.config = self._load_config(config_path)
        else:
            self.config = InspectionConfig()

        self.gps = gps

        # Inspected poles storage
        self._inspected_poles: Dict[str, InspectionResult] = {}  # global_id -> result

        # Spatial index for deduplication
        self._spatial_index: Dict[Tuple[int, int], str] = {}     # grid cell -> global_id

        # Tracking IDs we've already inspected (prevents re-inspection of same track)
        self._inspected_track_ids: Set[int] = set()

        # Pending inspections (poles that meet criteria but need capture)
        self._pending_inspections: Dict[int, float] = {}  # track_id -> first_seen_in_zone

        # Statistics
        self._stats = {
            'total_inspected': 0,
            'duplicates_skipped': 0,
            'low_confidence_skipped': 0,
            'no_camera_skipped': 0
        }

    def _load_config(self, config_path: str) -> InspectionConfig:
        """Load configuration from YAML file."""
        config = InspectionConfig()

        if not YAML_AVAILABLE:
            return config

        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)

            if 'inspection' in data:
                insp = data['inspection']
                config.min_fused_confidence = insp.get('min_fused_confidence',
                                                        config.min_fused_confidence)
                config.min_frames_tracked = insp.get('min_frames_tracked',
                                                      config.min_frames_tracked)
                config.camera_match_required = insp.get('camera_match_required',
                                                         config.camera_match_required)
                config.capture_distance = insp.get('capture_distance',
                                                    config.capture_distance)
                config.max_approach_distance = insp.get('max_approach_distance',
                                                         config.max_approach_distance)
                config.dwell_time = insp.get('dwell_time', config.dwell_time)
                config.photos_per_pole = insp.get('photos_per_pole',
                                                   config.photos_per_pole)
                config.dedup_grid_resolution = insp.get('dedup_grid_resolution',
                                                         config.dedup_grid_resolution)

        except Exception as e:
            print(f"Warning: Could not load inspection config: {e}")

        return config

    # =========================================================================
    # Grid Deduplication
    # =========================================================================

    def _get_grid_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Get grid cell for spatial indexing."""
        r = self.config.dedup_grid_resolution
        return (int(x / r), int(y / r))

    def is_already_inspected(self, pole) -> bool:
        """Check if we've already inspected a pole at this location.

        Uses spatial grid hashing with neighbor checks for robust deduplication.

        Args:
            pole: FusedPole object

        Returns:
            True if a pole at this location was already inspected
        """
        # Check by tracking ID first (fast path)
        if pole.id in self._inspected_track_ids:
            return True

        # Check spatial grid
        cell = self._get_grid_cell(pole.x, pole.y)

        # Check this cell and all adjacent cells (9-cell neighborhood)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                check_cell = (cell[0] + dx, cell[1] + dy)
                if check_cell in self._spatial_index:
                    existing_id = self._spatial_index[check_cell]
                    existing = self._inspected_poles.get(existing_id)
                    if existing:
                        # Check actual Euclidean distance
                        dist = math.sqrt(
                            (pole.x - existing.position_local[0])**2 +
                            (pole.y - existing.position_local[1])**2
                        )
                        if dist < self.config.dedup_distance_threshold:
                            return True

        return False

    def _register_inspected(self, pole, result: InspectionResult):
        """Register a pole as inspected in spatial index."""
        cell = self._get_grid_cell(pole.x, pole.y)
        self._spatial_index[cell] = result.global_pole_id
        self._inspected_poles[result.global_pole_id] = result
        self._inspected_track_ids.add(pole.id)

    # =========================================================================
    # Inspection Criteria
    # =========================================================================

    def should_inspect(self, pole) -> bool:
        """Check if pole meets inspection criteria.

        Args:
            pole: FusedPole object from sensor fusion

        Returns:
            True if pole should be inspected
        """
        # Already inspected?
        if self.is_already_inspected(pole):
            self._stats['duplicates_skipped'] += 1
            return False

        # Confidence check
        if pole.fused_confidence < self.config.min_fused_confidence:
            self._stats['low_confidence_skipped'] += 1
            return False

        # Tracking stability check
        if pole.frames_seen < self.config.min_frames_tracked:
            return False

        # Camera match required?
        if self.config.camera_match_required and not pole.camera_matched:
            self._stats['no_camera_skipped'] += 1
            return False

        return True

    def is_in_capture_zone(self, pole) -> bool:
        """Check if drone is in position to capture this pole.

        Args:
            pole: FusedPole object

        Returns:
            True if pole is within capture distance range
        """
        # Check range is within capture zone
        return (self.config.max_approach_distance <= pole.range_xy <=
                self.config.capture_distance)

    def is_approaching(self, pole) -> bool:
        """Check if drone is approaching (within min approach distance).

        Args:
            pole: FusedPole object

        Returns:
            True if pole is within approach distance
        """
        return pole.range_xy <= self.config.min_approach_distance

    # =========================================================================
    # Inspection Execution
    # =========================================================================

    def start_inspection(self, pole) -> bool:
        """Start tracking a pole for inspection (when entering capture zone).

        Args:
            pole: FusedPole object

        Returns:
            True if inspection started (wasn't already pending)
        """
        if pole.id in self._pending_inspections:
            return False

        self._pending_inspections[pole.id] = time.time()
        return True

    def get_dwell_progress(self, pole) -> float:
        """Get dwell progress for a pending inspection.

        Args:
            pole: FusedPole object

        Returns:
            Progress from 0.0 to 1.0 (1.0 = ready to capture)
        """
        if pole.id not in self._pending_inspections:
            return 0.0

        elapsed = time.time() - self._pending_inspections[pole.id]
        return min(1.0, elapsed / self.config.dwell_time)

    def is_dwell_complete(self, pole) -> bool:
        """Check if dwell time is complete for a pending inspection.

        Args:
            pole: FusedPole object

        Returns:
            True if dwell time has elapsed
        """
        return self.get_dwell_progress(pole) >= 1.0

    def cancel_inspection(self, pole_id: int):
        """Cancel a pending inspection (pole left capture zone).

        Args:
            pole_id: Tracking ID of pole
        """
        self._pending_inspections.pop(pole_id, None)

    def declare_found(self, pole, drone_heading: float = 0.0,
                      photo_paths: Optional[List[str]] = None,
                      lidar_path: Optional[str] = None,
                      metadata: Optional[dict] = None) -> InspectionResult:
        """Declare a pole as found/inspected.

        Creates an InspectionResult, registers it to prevent re-inspection,
        and optionally saves to disk.

        Args:
            pole: FusedPole object
            drone_heading: Drone heading in degrees (for GPS conversion)
            photo_paths: List of captured photo file paths
            lidar_path: Path to LiDAR snapshot file
            metadata: Additional metadata dict

        Returns:
            InspectionResult with all data
        """
        # Generate unique ID
        global_id = str(uuid.uuid4())

        # Get current time
        now = time.time()
        timestamp_iso = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(now))

        # Get GPS position
        if self.gps and self.gps.has_fix():
            gps_pos = self.gps.pole_position_to_gps(
                pole.x, pole.y, pole.z, drone_heading
            )
        else:
            gps_pos = (0.0, 0.0, 0.0)

        # Create result
        result = InspectionResult(
            pole_id=pole.id,
            global_pole_id=global_id,
            timestamp=now,
            timestamp_iso=timestamp_iso,
            position_local=(pole.x, pole.y, pole.z),
            position_gps=gps_pos,
            confidence=pole.fused_confidence,
            lidar_confidence=pole.lidar_confidence,
            camera_confidence=pole.camera_confidence,
            pole_type=pole.camera_class if pole.camera_matched else "unknown",
            pole_height=pole.height,
            pole_radius=pole.radius,
            photos=photo_paths or [],
            lidar_snapshot=lidar_path or "",
            metadata=metadata or {}
        )

        # Register as inspected
        self._register_inspected(pole, result)

        # Clear from pending
        self._pending_inspections.pop(pole.id, None)

        # Update stats
        self._stats['total_inspected'] += 1

        return result

    # =========================================================================
    # Persistence
    # =========================================================================

    def save_inspected_poles(self, path: str):
        """Save all inspected poles to JSON file.

        Args:
            path: Output file path
        """
        data = {
            'version': 1,
            'saved_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'poles': [r.to_dict() for r in self._inspected_poles.values()],
            'stats': self._stats
        }

        with open(path, 'w') as f:
            json.dump(data, f, indent=2)

    def load_inspected_poles(self, path: str) -> int:
        """Load previously inspected poles from JSON file.

        Used to resume a mission without re-inspecting poles.

        Args:
            path: Input file path

        Returns:
            Number of poles loaded
        """
        try:
            with open(path, 'r') as f:
                data = json.load(f)

            count = 0
            for pole_data in data.get('poles', []):
                result = InspectionResult.from_dict(pole_data)

                # Add to storage
                self._inspected_poles[result.global_pole_id] = result

                # Add to spatial index
                if result.position_local:
                    cell = self._get_grid_cell(
                        result.position_local[0],
                        result.position_local[1]
                    )
                    self._spatial_index[cell] = result.global_pole_id

                count += 1

            return count

        except Exception as e:
            print(f"Warning: Could not load inspected poles: {e}")
            return 0

    def load_from_aerosync(self, previously_inspected: List[dict]):
        """Load previously inspected poles from AeroSync job data.

        Args:
            previously_inspected: List of dicts with 'id', 'lat', 'lon'
        """
        for pole_data in previously_inspected:
            global_id = pole_data.get('id', str(uuid.uuid4()))
            lat = pole_data.get('lat', 0.0)
            lon = pole_data.get('lon', 0.0)

            # Convert GPS to local if we have GPS reference
            if self.gps:
                local_pos = self.gps.gps_to_local(lat, lon, 0.0)
            else:
                # Approximate conversion (won't work without reference)
                local_pos = (0.0, 0.0, 0.0)

            # Create minimal result for deduplication
            result = InspectionResult(
                pole_id=-1,  # Unknown original ID
                global_pole_id=global_id,
                timestamp=0.0,
                timestamp_iso="",
                position_local=local_pos,
                position_gps=(lat, lon, 0.0),
                confidence=1.0,
                lidar_confidence=1.0,
                camera_confidence=1.0,
                pole_type="unknown",
                pole_height=0.0,
                pole_radius=0.0
            )

            # Register
            self._inspected_poles[global_id] = result
            if local_pos != (0.0, 0.0, 0.0):
                cell = self._get_grid_cell(local_pos[0], local_pos[1])
                self._spatial_index[cell] = global_id

    # =========================================================================
    # Status and Statistics
    # =========================================================================

    def get_inspected_count(self) -> int:
        """Get number of inspected poles."""
        return len(self._inspected_poles)

    def get_pending_count(self) -> int:
        """Get number of pending inspections."""
        return len(self._pending_inspections)

    def get_stats(self) -> dict:
        """Get inspection statistics."""
        return self._stats.copy()

    def get_all_inspected(self) -> List[InspectionResult]:
        """Get all inspection results."""
        return list(self._inspected_poles.values())

    def reset(self):
        """Reset all inspection state."""
        self._inspected_poles.clear()
        self._spatial_index.clear()
        self._inspected_track_ids.clear()
        self._pending_inspections.clear()
        self._stats = {
            'total_inspected': 0,
            'duplicates_skipped': 0,
            'low_confidence_skipped': 0,
            'no_camera_skipped': 0
        }


# =============================================================================
# Convenience Functions
# =============================================================================

def create_inspector(config_path: str = 'config/settings.yaml',
                     gps: Optional['JetsonGPS'] = None) -> PoleInspector:
    """Create a PoleInspector with config from file."""
    return PoleInspector(config_path=config_path, gps=gps)


# =============================================================================
# Test
# =============================================================================

if __name__ == '__main__':
    from dataclasses import dataclass

    @dataclass
    class MockPole:
        """Mock FusedPole for testing."""
        id: int
        x: float
        y: float
        z: float = 10.0
        range_xy: float = 10.0
        fused_confidence: float = 0.8
        lidar_confidence: float = 0.9
        camera_confidence: float = 0.7
        camera_matched: bool = True
        camera_class: str = "h_frame"
        frames_seen: int = 15
        height: float = 12.0
        radius: float = 0.15

    print("Pole Inspector Test")
    print("=" * 50)

    # Create inspector
    config = InspectionConfig(
        min_fused_confidence=0.6,
        min_frames_tracked=10,
        camera_match_required=True,
        capture_distance=15.0,
        max_approach_distance=5.0
    )
    inspector = PoleInspector(config)

    # Test poles
    poles = [
        MockPole(1, 50.0, 2.0, range_xy=50.0),   # Too far
        MockPole(2, 12.0, -1.0, range_xy=12.0),  # In capture zone
        MockPole(3, 12.5, -0.5, range_xy=12.5),  # Near pole 2 (should dedup)
        MockPole(4, 100.0, 5.0, range_xy=100.0, fused_confidence=0.4),  # Low conf
        MockPole(5, 80.0, -3.0, range_xy=80.0, camera_matched=False),   # No camera
    ]

    print("\nTesting inspection criteria:")
    for pole in poles:
        should = inspector.should_inspect(pole)
        in_zone = inspector.is_in_capture_zone(pole)
        print(f"  Pole {pole.id}: should_inspect={should}, in_zone={in_zone}")

        if should and in_zone:
            # Simulate inspection
            result = inspector.declare_found(pole)
            print(f"    -> Inspected! ID: {result.global_pole_id[:8]}...")

    print(f"\nStats: {inspector.get_stats()}")
    print(f"Total inspected: {inspector.get_inspected_count()}")

    # Test persistence
    inspector.save_inspected_poles('/tmp/test_poles.json')
    print("\nSaved to /tmp/test_poles.json")

    # Test reload
    inspector2 = PoleInspector(config)
    loaded = inspector2.load_inspected_poles('/tmp/test_poles.json')
    print(f"Loaded {loaded} poles")

    # Verify dedup works after reload
    test_pole = MockPole(99, 12.0, -1.0, range_xy=12.0)  # Same location as pole 2
    is_dup = inspector2.is_already_inspected(test_pole)
    print(f"Dedup check for pole at same location: is_duplicate={is_dup}")
