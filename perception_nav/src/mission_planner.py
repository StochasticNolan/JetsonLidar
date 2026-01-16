"""
Mission Planner Module

Parses AeroSync job data into Mission objects and generates flight waypoints
for different search patterns (line_follow, lawnmower, spiral).

Features:
- Mission dataclass with full AeroSync job fields
- Waypoint generation for multiple search patterns
- Mission validation
- GeoJSON polygon handling
- Coordinate conversions

Usage:
    from mission_planner import parse_mission, generate_waypoints, validate_mission

    # Parse job from AeroSync
    job_data = await aerosync_client.fetch_job(job_id)
    mission = parse_mission(job_data)

    # Validate before execution
    valid, error = validate_mission(mission)
    if not valid:
        raise ValueError(error)

    # Generate waypoints
    waypoints = generate_waypoints(mission)
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Any
from enum import Enum

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

# Import StopConditions from mission_manager if available
try:
    from .mission_manager import StopConditions
except ImportError:
    # Define locally if import fails
    @dataclass
    class StopConditions:
        max_poles: int = 0
        max_time_minutes: float = 0
        min_battery_percent: float = 20.0
        coverage_threshold: float = 0.95
        max_distance_from_home: float = 0

        @classmethod
        def from_dict(cls, data: dict) -> 'StopConditions':
            return cls(
                max_poles=data.get('max_poles', 0),
                max_time_minutes=data.get('max_time_minutes', 0),
                min_battery_percent=data.get('min_battery_percent', 20.0),
                coverage_threshold=data.get('coverage_threshold', 0.95),
                max_distance_from_home=data.get('max_distance_from_home', 0)
            )


# =============================================================================
# Constants
# =============================================================================

# Earth radius in meters (WGS84 mean radius)
EARTH_RADIUS_M = 6371000.0

# Meters per degree latitude (approximately constant)
METERS_PER_DEG_LAT = 111320.0


# =============================================================================
# Enums
# =============================================================================

class SearchPattern(Enum):
    """Available search patterns."""
    LINE_FOLLOW = "line_follow"
    LAWNMOWER = "lawnmower"
    SPIRAL = "spiral"


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class Waypoint:
    """GPS waypoint for flight navigation."""
    lat: float              # Latitude (degrees)
    lon: float              # Longitude (degrees)
    alt: float              # Altitude AGL (meters)
    speed: float = 3.0      # Target speed (m/s)
    heading: Optional[float] = None  # Optional heading (degrees, 0=North)
    action: str = "fly_through"  # 'fly_through', 'hover', 'capture'

    def to_dict(self) -> dict:
        return {
            'lat': self.lat,
            'lon': self.lon,
            'alt': self.alt,
            'speed': self.speed,
            'heading': self.heading,
            'action': self.action
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'Waypoint':
        return cls(
            lat=data['lat'],
            lon=data['lon'],
            alt=data['alt'],
            speed=data.get('speed', 3.0),
            heading=data.get('heading'),
            action=data.get('action', 'fly_through')
        )


@dataclass
class SearchConfig:
    """Configuration for search pattern execution."""
    sweep_spacing: float = 50.0      # Distance between parallel sweeps (m)
    flight_altitude: float = 30.0    # Flight altitude AGL (m)
    speed: float = 3.0               # Cruise speed (m/s)
    sweep_direction: float = 0.0     # Sweep direction (degrees from North)
    overlap_percent: float = 20.0    # Overlap between sweeps (%)

    @classmethod
    def from_dict(cls, data: dict) -> 'SearchConfig':
        return cls(
            sweep_spacing=data.get('sweep_spacing', 50.0),
            flight_altitude=data.get('flight_altitude', 30.0),
            speed=data.get('speed', 3.0),
            sweep_direction=data.get('sweep_direction', 0.0),
            overlap_percent=data.get('overlap_percent', 20.0)
        )


@dataclass
class InspectionConfig:
    """Configuration for pole inspection."""
    min_confidence: float = 0.5      # Min detection confidence
    camera_required: bool = True     # Require camera confirmation
    capture_distance: float = 10.0   # Distance to capture photos (m)
    photos_per_pole: int = 1         # Number of photos per pole
    dwell_time: float = 2.0          # Hover time for capture (s)

    @classmethod
    def from_dict(cls, data: dict) -> 'InspectionConfig':
        return cls(
            min_confidence=data.get('min_confidence', 0.5),
            camera_required=data.get('camera_required', True),
            capture_distance=data.get('capture_distance', 10.0),
            photos_per_pole=data.get('photos_per_pole', 1),
            dwell_time=data.get('dwell_time', 2.0)
        )


@dataclass
class GeoPolygon:
    """GeoJSON-style polygon for search area."""
    coordinates: List[Tuple[float, float]]  # List of (lon, lat) pairs

    @classmethod
    def from_geojson(cls, geojson: dict) -> 'GeoPolygon':
        """Parse from GeoJSON format.

        GeoJSON format: {
            "type": "Polygon",
            "coordinates": [[[lon, lat], [lon, lat], ...]]
        }
        """
        if geojson.get('type') == 'Polygon':
            # First ring is exterior boundary
            coords = geojson['coordinates'][0]
            return cls(coordinates=[(c[0], c[1]) for c in coords])
        elif isinstance(geojson.get('coordinates'), list):
            # Direct coordinates array
            coords = geojson['coordinates']
            if isinstance(coords[0][0], list):
                # Nested [[lon, lat], ...]
                coords = coords[0]
            return cls(coordinates=[(c[0], c[1]) for c in coords])
        else:
            raise ValueError(f"Invalid GeoJSON format: {geojson}")

    def get_bounds(self) -> Tuple[float, float, float, float]:
        """Get bounding box (min_lon, min_lat, max_lon, max_lat)."""
        lons = [c[0] for c in self.coordinates]
        lats = [c[1] for c in self.coordinates]
        return (min(lons), min(lats), max(lons), max(lats))

    def get_center(self) -> Tuple[float, float]:
        """Get centroid (lon, lat)."""
        min_lon, min_lat, max_lon, max_lat = self.get_bounds()
        return ((min_lon + max_lon) / 2, (min_lat + max_lat) / 2)

    def is_closed(self) -> bool:
        """Check if polygon is properly closed."""
        if len(self.coordinates) < 3:
            return False
        first = self.coordinates[0]
        last = self.coordinates[-1]
        return abs(first[0] - last[0]) < 1e-9 and abs(first[1] - last[1]) < 1e-9

    def contains_point(self, lon: float, lat: float) -> bool:
        """Check if point is inside polygon (ray casting algorithm)."""
        n = len(self.coordinates)
        inside = False

        j = n - 1
        for i in range(n):
            xi, yi = self.coordinates[i]
            xj, yj = self.coordinates[j]

            if ((yi > lat) != (yj > lat)) and \
               (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi):
                inside = not inside
            j = i

        return inside

    def get_corners(self) -> List[Tuple[float, float]]:
        """Get unique corner vertices (excluding closing point if duplicate)."""
        corners = []
        for i, coord in enumerate(self.coordinates):
            # Skip last point if it's same as first (closed polygon)
            if i == len(self.coordinates) - 1:
                if abs(coord[0] - self.coordinates[0][0]) < 1e-9 and \
                   abs(coord[1] - self.coordinates[0][1]) < 1e-9:
                    continue
            corners.append(coord)
        return corners

    def get_closest_corner(self, lat: float, lon: float) -> Tuple[float, float, float]:
        """Find the closest corner to a given position.

        Args:
            lat: Latitude of reference point
            lon: Longitude of reference point

        Returns:
            Tuple of (corner_lon, corner_lat, distance_m)
        """
        corners = self.get_corners()
        closest = None
        min_dist = float('inf')

        for corner_lon, corner_lat in corners:
            # Quick approximation for distance (accurate enough for comparison)
            dlat = (corner_lat - lat) * 111320  # meters
            dlon = (corner_lon - lon) * 111320 * math.cos(math.radians(lat))
            dist = math.sqrt(dlat**2 + dlon**2)

            if dist < min_dist:
                min_dist = dist
                closest = (corner_lon, corner_lat)

        return (closest[0], closest[1], min_dist) if closest else (lon, lat, 0)

    def get_bounding_corners(self) -> List[Tuple[float, float]]:
        """Get the four corners of the bounding box.

        Returns:
            List of (lon, lat) tuples: [SW, SE, NE, NW]
        """
        min_lon, min_lat, max_lon, max_lat = self.get_bounds()
        return [
            (min_lon, min_lat),  # SW
            (max_lon, min_lat),  # SE
            (max_lon, max_lat),  # NE
            (min_lon, max_lat),  # NW
        ]


@dataclass
class PreviouslyInspected:
    """Record of previously inspected pole."""
    lat: float
    lon: float
    pole_id: Optional[str] = None
    inspection_date: Optional[str] = None

    @classmethod
    def from_dict(cls, data: dict) -> 'PreviouslyInspected':
        return cls(
            lat=data.get('lat', data.get('latitude', 0.0)),
            lon=data.get('lon', data.get('longitude', 0.0)),
            pole_id=data.get('pole_id'),
            inspection_date=data.get('inspection_date')
        )


@dataclass
class Mission:
    """Complete mission parsed from AeroSync job data."""
    job_id: str
    name: str
    search_area: GeoPolygon
    search_pattern: SearchPattern
    start_point: Tuple[float, float, float]  # (lat, lon, alt)
    home_point: Tuple[float, float, float]   # (lat, lon, alt)
    search_config: SearchConfig
    stop_conditions: StopConditions
    inspection_config: InspectionConfig
    previously_inspected: List[PreviouslyInspected] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def get_start_waypoint(self) -> Waypoint:
        """Get waypoint for mission start point."""
        return Waypoint(
            lat=self.start_point[0],
            lon=self.start_point[1],
            alt=self.start_point[2],
            speed=self.search_config.speed,
            action='hover'
        )

    def get_home_waypoint(self) -> Waypoint:
        """Get waypoint for home/RTL point."""
        return Waypoint(
            lat=self.home_point[0],
            lon=self.home_point[1],
            alt=self.home_point[2],
            speed=self.search_config.speed,
            action='hover'
        )

    def is_previously_inspected(self, lat: float, lon: float,
                                 threshold_m: float = 20.0) -> bool:
        """Check if a location was previously inspected."""
        for prev in self.previously_inspected:
            dist = haversine_distance(lat, lon, prev.lat, prev.lon)
            if dist < threshold_m:
                return True
        return False


# =============================================================================
# Coordinate Utilities
# =============================================================================

def haversine_distance(lat1: float, lon1: float,
                       lat2: float, lon2: float) -> float:
    """Calculate distance between two GPS coordinates in meters."""
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)

    a = math.sin(dlat/2)**2 + \
        math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    return EARTH_RADIUS_M * c


def meters_to_degrees_lat(meters: float) -> float:
    """Convert meters to degrees latitude."""
    return meters / METERS_PER_DEG_LAT


def meters_to_degrees_lon(meters: float, lat: float) -> float:
    """Convert meters to degrees longitude at given latitude."""
    meters_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(lat))
    if meters_per_deg_lon < 1.0:
        meters_per_deg_lon = 1.0  # Avoid division by zero near poles
    return meters / meters_per_deg_lon


def offset_position(lat: float, lon: float,
                    north_m: float, east_m: float) -> Tuple[float, float]:
    """Offset a GPS position by meters north and east."""
    new_lat = lat + meters_to_degrees_lat(north_m)
    new_lon = lon + meters_to_degrees_lon(east_m, lat)
    return (new_lat, new_lon)


def bearing_between(lat1: float, lon1: float,
                    lat2: float, lon2: float) -> float:
    """Calculate bearing from point 1 to point 2 in degrees (0=North)."""
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)

    x = math.sin(dlon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)

    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360


# =============================================================================
# Mission Parser
# =============================================================================

def parse_mission(job_data: dict) -> Mission:
    """Parse AeroSync job data into Mission object.

    Expected job_data format:
    {
        "job_id": "uuid-string",
        "name": "Survey Name",
        "search_area": {
            "type": "Polygon",
            "coordinates": [[[lon, lat], ...]]
        },
        "search_pattern": "lawnmower",
        "search_config": {
            "sweep_spacing": 50.0,
            "flight_altitude": 30.0,
            "speed": 3.0
        },
        "stop_conditions": {
            "max_poles": 100,
            "max_time_minutes": 60,
            "min_battery_percent": 25.0
        },
        "inspection_config": {
            "min_confidence": 0.5,
            "capture_distance": 10.0
        },
        "start_point": {"lat": 41.0, "lon": -105.0},
        "home_point": {"lat": 41.0, "lon": -105.0},
        "previously_inspected": [
            {"lat": 41.001, "lon": -105.001, "pole_id": "..."}
        ]
    }

    Args:
        job_data: Dictionary from AeroSync API

    Returns:
        Mission object

    Raises:
        ValueError: If required fields are missing or invalid
    """
    # Required fields
    job_id = job_data.get('job_id', job_data.get('id', ''))
    if not job_id:
        raise ValueError("Mission must have job_id")

    name = job_data.get('name', f'Mission {job_id}')

    # Parse search area
    search_area_data = job_data.get('search_area')
    if not search_area_data:
        raise ValueError("Mission must have search_area")

    try:
        search_area = GeoPolygon.from_geojson(search_area_data)
    except Exception as e:
        raise ValueError(f"Invalid search_area: {e}")

    # Parse search pattern
    pattern_str = job_data.get('search_pattern', 'line_follow').lower()
    try:
        search_pattern = SearchPattern(pattern_str)
    except ValueError:
        # Default to line_follow for unknown patterns
        print(f"Warning: Unknown search pattern '{pattern_str}', defaulting to line_follow")
        search_pattern = SearchPattern.LINE_FOLLOW

    # Parse search config
    search_config_data = job_data.get('search_config', {})
    search_config = SearchConfig.from_dict(search_config_data)

    # Parse stop conditions
    stop_conditions_data = job_data.get('stop_conditions', {})
    stop_conditions = StopConditions.from_dict(stop_conditions_data)

    # Parse inspection config
    inspection_config_data = job_data.get('inspection_config', {})
    inspection_config = InspectionConfig.from_dict(inspection_config_data)

    # Parse start point
    start_data = job_data.get('start_point', {})
    if start_data:
        start_lat = start_data.get('lat', start_data.get('latitude', 0.0))
        start_lon = start_data.get('lon', start_data.get('longitude', 0.0))
        start_alt = start_data.get('alt', search_config.flight_altitude)
    else:
        # Default to first polygon vertex
        center = search_area.get_center()
        start_lon, start_lat = center
        start_alt = search_config.flight_altitude
    start_point = (start_lat, start_lon, start_alt)

    # Parse home point
    home_data = job_data.get('home_point', job_data.get('launch_point', {}))
    if home_data:
        home_lat = home_data.get('lat', home_data.get('latitude', start_lat))
        home_lon = home_data.get('lon', home_data.get('longitude', start_lon))
        home_alt = home_data.get('alt', 0.0)  # Home is typically ground level
    else:
        # Default to start point
        home_lat, home_lon = start_lat, start_lon
        home_alt = 0.0
    home_point = (home_lat, home_lon, home_alt)

    # Parse previously inspected
    previously_inspected = []
    for prev_data in job_data.get('previously_inspected', []):
        try:
            prev = PreviouslyInspected.from_dict(prev_data)
            previously_inspected.append(prev)
        except Exception as e:
            print(f"Warning: Could not parse previously_inspected entry: {e}")

    # Collect any extra metadata
    metadata = {k: v for k, v in job_data.items()
                if k not in ['job_id', 'id', 'name', 'search_area', 'search_pattern',
                            'search_config', 'stop_conditions', 'inspection_config',
                            'start_point', 'home_point', 'launch_point',
                            'previously_inspected']}

    return Mission(
        job_id=job_id,
        name=name,
        search_area=search_area,
        search_pattern=search_pattern,
        start_point=start_point,
        home_point=home_point,
        search_config=search_config,
        stop_conditions=stop_conditions,
        inspection_config=inspection_config,
        previously_inspected=previously_inspected,
        metadata=metadata
    )


# =============================================================================
# Waypoint Generators
# =============================================================================

def generate_waypoints(mission: Mission) -> List[Waypoint]:
    """Generate waypoint list based on mission search pattern.

    Args:
        mission: Parsed Mission object

    Returns:
        List of Waypoints for the flight path
    """
    if mission.search_pattern == SearchPattern.LINE_FOLLOW:
        return generate_line_follow_waypoints(mission)

    elif mission.search_pattern == SearchPattern.LAWNMOWER:
        return generate_lawnmower_waypoints(mission)

    elif mission.search_pattern == SearchPattern.SPIRAL:
        return generate_spiral_waypoints(mission)

    else:
        raise ValueError(f"Unknown search pattern: {mission.search_pattern}")


def generate_line_follow_waypoints(mission: Mission) -> List[Waypoint]:
    """Generate waypoints for line follow pattern.

    Line follow is simple: fly to start point and let Jetson
    autonomously follow detected poles.

    Args:
        mission: Mission object

    Returns:
        List with just the start waypoint
    """
    waypoints = []

    # Single waypoint at start - Jetson takes over from there
    waypoints.append(Waypoint(
        lat=mission.start_point[0],
        lon=mission.start_point[1],
        alt=mission.search_config.flight_altitude,
        speed=mission.search_config.speed,
        action='hover'
    ))

    return waypoints


def compute_entry_point(mission: Mission,
                        from_position: Optional[Tuple[float, float]] = None) -> Waypoint:
    """Compute the optimal entry point to start the search pattern.

    Finds the closest corner of the search area bounding box to minimize
    transit distance before starting the search.

    Args:
        mission: Mission object
        from_position: Optional (lat, lon) of current position. Uses home if None.

    Returns:
        Waypoint at the optimal entry point
    """
    polygon = mission.search_area
    config = mission.search_config

    # Use home position if not specified
    if from_position is None:
        ref_lat, ref_lon = mission.home_point[0], mission.home_point[1]
    else:
        ref_lat, ref_lon = from_position

    # Get bounding box corners
    corners = polygon.get_bounding_corners()  # [SW, SE, NE, NW]

    # Find closest corner
    min_dist = float('inf')
    closest_corner = corners[0]
    for corner_lon, corner_lat in corners:
        dist = haversine_distance(ref_lat, ref_lon, corner_lat, corner_lon)
        if dist < min_dist:
            min_dist = dist
            closest_corner = (corner_lon, corner_lat)

    return Waypoint(
        lat=closest_corner[1],
        lon=closest_corner[0],
        alt=config.flight_altitude,
        speed=config.speed,
        action='hover'
    )


def generate_lawnmower_waypoints(mission: Mission,
                                  from_position: Optional[Tuple[float, float]] = None) -> List[Waypoint]:
    """Generate parallel sweep waypoints covering polygon (lawnmower pattern).

    Creates a back-and-forth pattern of parallel lines across the search area.
    Starts from the corner closest to the drone's current/home position for
    efficient flight path.

    Args:
        mission: Mission object
        from_position: Optional (lat, lon) to start from. If None, uses home_point.

    Returns:
        List of waypoints for lawnmower pattern
    """
    polygon = mission.search_area
    config = mission.search_config

    # Get bounds
    min_lon, min_lat, max_lon, max_lat = polygon.get_bounds()

    # Determine starting position (use home if not specified)
    if from_position is None:
        start_lat, start_lon = mission.home_point[0], mission.home_point[1]
    else:
        start_lat, start_lon = from_position

    # Find closest bounding box corner to determine optimal entry point
    corners = polygon.get_bounding_corners()  # [SW, SE, NE, NW]
    corner_names = ['SW', 'SE', 'NE', 'NW']

    min_dist = float('inf')
    closest_corner_idx = 0
    for i, (corner_lon, corner_lat) in enumerate(corners):
        dist = haversine_distance(start_lat, start_lon, corner_lat, corner_lon)
        if dist < min_dist:
            min_dist = dist
            closest_corner_idx = i

    # Determine sweep strategy based on closest corner
    # SW (0): start west, go south to north
    # SE (1): start east, go south to north
    # NE (2): start east, go north to south
    # NW (3): start west, go north to south
    start_west = closest_corner_idx in [0, 3]  # SW or NW
    start_south = closest_corner_idx in [0, 1]  # SW or SE

    # Convert spacing from meters to degrees
    center_lat = (min_lat + max_lat) / 2
    spacing_deg_lon = meters_to_degrees_lon(config.sweep_spacing, center_lat)

    waypoints = []

    # First waypoint: fly to the closest corner of the search area
    entry_corner = corners[closest_corner_idx]
    waypoints.append(Waypoint(
        lat=entry_corner[1],  # lat
        lon=entry_corner[0],  # lon
        alt=config.flight_altitude,
        speed=config.speed,
        action='fly_through'
    ))

    # Generate sweep lines based on entry corner
    if start_west:
        lon_start = min_lon
        lon_end = max_lon
        lon_step = spacing_deg_lon
    else:
        lon_start = max_lon
        lon_end = min_lon
        lon_step = -spacing_deg_lon

    # Initial sweep direction based on entry corner
    if start_south:
        sweep_direction = 1   # south to north
    else:
        sweep_direction = -1  # north to south

    current_lon = lon_start
    sweep_count = 0

    while (start_west and current_lon <= lon_end) or \
          (not start_west and current_lon >= lon_end):

        # Sample along the sweep line to find entry/exit points inside polygon
        num_samples = 30
        inside_points = []

        for i in range(num_samples + 1):
            sample_lat = min_lat + (max_lat - min_lat) * i / num_samples
            if polygon.contains_point(current_lon, sample_lat):
                inside_points.append(sample_lat)

        if inside_points:
            # Get the entry and exit points
            if sweep_direction == 1:
                # South to north
                entry_lat = min(inside_points)
                exit_lat = max(inside_points)
            else:
                # North to south
                entry_lat = max(inside_points)
                exit_lat = min(inside_points)

            # Add waypoint at entry (if not first sweep, we're already near here)
            if sweep_count > 0 or abs(entry_lat - entry_corner[1]) > 0.0001:
                waypoints.append(Waypoint(
                    lat=entry_lat,
                    lon=current_lon,
                    alt=config.flight_altitude,
                    speed=config.speed,
                    heading=0 if sweep_direction == 1 else 180
                ))

            # Add waypoint at exit
            waypoints.append(Waypoint(
                lat=exit_lat,
                lon=current_lon,
                alt=config.flight_altitude,
                speed=config.speed,
                heading=0 if sweep_direction == 1 else 180
            ))

            sweep_count += 1

        # Move to next sweep line
        current_lon += lon_step
        sweep_direction *= -1  # Alternate direction

    # If no waypoints generated, fall back to start point
    if not waypoints:
        waypoints.append(mission.get_start_waypoint())

    return waypoints


def generate_spiral_waypoints(mission: Mission) -> List[Waypoint]:
    """Generate outward spiral waypoints from center of polygon.

    Creates an expanding spiral pattern starting from the center of the
    search area, spiraling outward until the area is covered.

    Args:
        mission: Mission object

    Returns:
        List of waypoints for spiral pattern
    """
    polygon = mission.search_area
    config = mission.search_config

    # Get center and bounds
    center_lon, center_lat = polygon.get_center()
    min_lon, min_lat, max_lon, max_lat = polygon.get_bounds()

    # Calculate max radius needed
    corner_distances = [
        haversine_distance(center_lat, center_lon, min_lat, min_lon),
        haversine_distance(center_lat, center_lon, min_lat, max_lon),
        haversine_distance(center_lat, center_lon, max_lat, min_lon),
        haversine_distance(center_lat, center_lon, max_lat, max_lon),
    ]
    max_radius = max(corner_distances) * 1.1  # Add margin

    # Spiral parameters - use sweep_spacing as distance between spiral arms
    spacing = config.sweep_spacing

    # Points per revolution - reasonable number for smooth path
    # More points for larger spirals, but capped for performance
    points_per_rev = min(36, max(8, int(max_radius / spacing)))

    waypoints = []

    # Generate spiral points using Archimedean spiral
    # r = a + b*theta where b = spacing / (2*pi)
    revolutions = max_radius / spacing
    total_points = int(revolutions * points_per_rev)

    # Limit total waypoints to reasonable number
    max_waypoints = 500
    if total_points > max_waypoints:
        # Skip points to stay under limit
        step = total_points // max_waypoints
    else:
        step = 1

    for i in range(0, total_points, step):
        angle = 2 * math.pi * i / points_per_rev
        radius = spacing * angle / (2 * math.pi)

        if radius < spacing / 2:
            continue  # Skip center

        # Calculate position on spiral
        north = radius * math.cos(angle)
        east = radius * math.sin(angle)

        lat, lon = offset_position(center_lat, center_lon, north, east)

        # Only add if inside polygon
        if polygon.contains_point(lon, lat):
            # Calculate heading (tangent to spiral, pointing outward)
            heading = math.degrees(angle + math.pi / 2) % 360

            waypoints.append(Waypoint(
                lat=lat,
                lon=lon,
                alt=config.flight_altitude,
                speed=config.speed,
                heading=heading
            ))

    # If no waypoints generated, fall back to start point
    if not waypoints:
        waypoints.append(mission.get_start_waypoint())

    return waypoints


def _point_near_polygon(lat: float, lon: float,
                        polygon: GeoPolygon, threshold_m: float) -> bool:
    """Check if point is within threshold distance of polygon boundary."""
    for coord in polygon.coordinates:
        dist = haversine_distance(lat, lon, coord[1], coord[0])
        if dist < threshold_m:
            return True
    return False


# =============================================================================
# Mission Validation
# =============================================================================

def validate_mission(mission: Mission) -> Tuple[bool, str]:
    """Validate mission parameters before execution.

    Checks for:
    - Valid altitude
    - Valid speed
    - Closed polygon
    - Reasonable search area size
    - Valid stop conditions

    Args:
        mission: Mission object to validate

    Returns:
        Tuple of (is_valid, error_message)
    """
    errors = []

    # Check altitude
    alt = mission.search_config.flight_altitude
    if alt <= 0:
        errors.append(f"Flight altitude must be > 0 (got {alt})")
    if alt > 120:
        errors.append(f"Flight altitude too high: {alt}m (max 120m for safety)")

    # Check speed
    speed = mission.search_config.speed
    if speed <= 0:
        errors.append(f"Speed must be > 0 (got {speed})")
    if speed > 15:
        errors.append(f"Speed too high: {speed} m/s (max 15 m/s for safety)")

    # Check polygon
    if not mission.search_area.is_closed():
        errors.append("Search area polygon is not closed")

    if len(mission.search_area.coordinates) < 3:
        errors.append("Search area must have at least 3 vertices")

    # Check polygon size
    min_lon, min_lat, max_lon, max_lat = mission.search_area.get_bounds()
    width_m = haversine_distance((min_lat + max_lat)/2, min_lon,
                                  (min_lat + max_lat)/2, max_lon)
    height_m = haversine_distance(min_lat, (min_lon + max_lon)/2,
                                   max_lat, (min_lon + max_lon)/2)

    area_km2 = (width_m * height_m) / 1e6
    if area_km2 > 100:
        errors.append(f"Search area too large: {area_km2:.1f} km² (max 100 km²)")

    if width_m < 10 or height_m < 10:
        errors.append(f"Search area too small: {width_m:.0f}x{height_m:.0f}m")

    # Check start point is valid
    start_lat, start_lon, start_alt = mission.start_point
    if not (-90 <= start_lat <= 90):
        errors.append(f"Invalid start latitude: {start_lat}")
    if not (-180 <= start_lon <= 180):
        errors.append(f"Invalid start longitude: {start_lon}")

    # Check stop conditions are reasonable
    sc = mission.stop_conditions
    if sc.min_battery_percent < 10:
        errors.append(f"Battery threshold too low: {sc.min_battery_percent}% (min 10%)")
    if sc.min_battery_percent > 50:
        errors.append(f"Battery threshold too high: {sc.min_battery_percent}% (max 50%)")

    # Estimate flight time (rough check)
    if mission.search_pattern == SearchPattern.LAWNMOWER:
        estimated_distance_m = width_m * height_m / mission.search_config.sweep_spacing
        estimated_time_min = estimated_distance_m / (speed * 60)

        if sc.max_time_minutes > 0 and estimated_time_min > sc.max_time_minutes * 2:
            errors.append(
                f"Estimated flight time ({estimated_time_min:.0f} min) may exceed "
                f"time limit ({sc.max_time_minutes} min) significantly"
            )

    if errors:
        return False, "; ".join(errors)

    return True, "Valid"


def estimate_mission_stats(mission: Mission) -> dict:
    """Estimate mission statistics before execution.

    Args:
        mission: Mission object

    Returns:
        Dictionary with estimates:
        - total_distance_m: Estimated total flight distance
        - estimated_time_min: Estimated flight time
        - num_waypoints: Number of waypoints
        - coverage_area_km2: Area to cover
    """
    # Generate waypoints to estimate distance
    waypoints = generate_waypoints(mission)

    # Calculate total distance
    total_distance = 0.0
    for i in range(1, len(waypoints)):
        dist = haversine_distance(
            waypoints[i-1].lat, waypoints[i-1].lon,
            waypoints[i].lat, waypoints[i].lon
        )
        total_distance += dist

    # Add distance from home to start and back
    start_to_first = haversine_distance(
        mission.home_point[0], mission.home_point[1],
        waypoints[0].lat, waypoints[0].lon
    ) if waypoints else 0

    last_to_home = haversine_distance(
        waypoints[-1].lat, waypoints[-1].lon,
        mission.home_point[0], mission.home_point[1]
    ) if waypoints else 0

    total_distance += start_to_first + last_to_home

    # Estimate time
    avg_speed = mission.search_config.speed
    estimated_time_min = (total_distance / avg_speed) / 60 if avg_speed > 0 else 0

    # Calculate coverage area
    min_lon, min_lat, max_lon, max_lat = mission.search_area.get_bounds()
    width_m = haversine_distance((min_lat + max_lat)/2, min_lon,
                                  (min_lat + max_lat)/2, max_lon)
    height_m = haversine_distance(min_lat, (min_lon + max_lon)/2,
                                   max_lat, (min_lon + max_lon)/2)
    coverage_area_km2 = (width_m * height_m) / 1e6

    return {
        'total_distance_m': total_distance,
        'estimated_time_min': estimated_time_min,
        'num_waypoints': len(waypoints),
        'coverage_area_km2': coverage_area_km2,
        'avg_speed_ms': avg_speed
    }


# =============================================================================
# Convenience Functions
# =============================================================================

def load_mission_from_file(filepath: str) -> Mission:
    """Load mission from JSON or YAML file.

    Args:
        filepath: Path to mission file

    Returns:
        Parsed Mission object
    """
    import json

    with open(filepath, 'r') as f:
        if filepath.endswith('.yaml') or filepath.endswith('.yml'):
            if not YAML_AVAILABLE:
                raise ImportError("PyYAML not installed")
            data = yaml.safe_load(f)
        else:
            data = json.load(f)

    return parse_mission(data)


def save_waypoints_to_file(waypoints: List[Waypoint], filepath: str):
    """Save waypoints to JSON file.

    Args:
        waypoints: List of Waypoint objects
        filepath: Output file path
    """
    import json

    data = {
        'waypoints': [wp.to_dict() for wp in waypoints],
        'count': len(waypoints)
    }

    with open(filepath, 'w') as f:
        json.dump(data, f, indent=2)


# =============================================================================
# Test
# =============================================================================

if __name__ == '__main__':
    print("Mission Planner Test")
    print("=" * 60)

    # Test job data (simulating AeroSync response)
    test_job = {
        "job_id": "test-mission-001",
        "name": "Wyoming Pole Survey",
        "search_area": {
            "type": "Polygon",
            "coordinates": [[
                [-105.5, 41.0],
                [-105.4, 41.0],
                [-105.4, 41.1],
                [-105.5, 41.1],
                [-105.5, 41.0]  # Closed
            ]]
        },
        "search_pattern": "lawnmower",
        "search_config": {
            "sweep_spacing": 75.0,
            "flight_altitude": 30.0,
            "speed": 5.0,
            "sweep_direction": 45.0
        },
        "stop_conditions": {
            "max_poles": 50,
            "max_time_minutes": 60,
            "min_battery_percent": 25.0
        },
        "inspection_config": {
            "min_confidence": 0.6,
            "capture_distance": 10.0
        },
        "start_point": {
            "lat": 41.05,
            "lon": -105.45
        },
        "home_point": {
            "lat": 41.05,
            "lon": -105.45
        },
        "previously_inspected": [
            {"lat": 41.02, "lon": -105.48, "pole_id": "pole-001"}
        ]
    }

    # Parse mission
    print("\n1. Parsing mission...")
    try:
        mission = parse_mission(test_job)
        print(f"   Job ID: {mission.job_id}")
        print(f"   Name: {mission.name}")
        print(f"   Pattern: {mission.search_pattern.value}")
        print(f"   Start: ({mission.start_point[0]:.4f}, {mission.start_point[1]:.4f})")
        print(f"   Altitude: {mission.search_config.flight_altitude}m")
        print(f"   Speed: {mission.search_config.speed} m/s")
        print(f"   Previously inspected: {len(mission.previously_inspected)} poles")
    except Exception as e:
        print(f"   ERROR: {e}")

    # Validate mission
    print("\n2. Validating mission...")
    valid, error = validate_mission(mission)
    if valid:
        print("   Mission is VALID")
    else:
        print(f"   INVALID: {error}")

    # Estimate stats
    print("\n3. Estimating mission stats...")
    stats = estimate_mission_stats(mission)
    print(f"   Total distance: {stats['total_distance_m']/1000:.1f} km")
    print(f"   Estimated time: {stats['estimated_time_min']:.0f} minutes")
    print(f"   Waypoints: {stats['num_waypoints']}")
    print(f"   Coverage area: {stats['coverage_area_km2']:.2f} km²")

    # Generate waypoints
    print("\n4. Generating waypoints...")
    waypoints = generate_waypoints(mission)
    print(f"   Generated {len(waypoints)} waypoints")

    if waypoints:
        print("\n   First 5 waypoints:")
        for i, wp in enumerate(waypoints[:5]):
            print(f"   {i+1}. ({wp.lat:.5f}, {wp.lon:.5f}) @ {wp.alt}m, "
                  f"{wp.speed} m/s, heading={wp.heading}")

    # Test line follow pattern
    print("\n5. Testing LINE_FOLLOW pattern...")
    test_job['search_pattern'] = 'line_follow'
    mission_lf = parse_mission(test_job)
    waypoints_lf = generate_waypoints(mission_lf)
    print(f"   Generated {len(waypoints_lf)} waypoints (should be 1 for line_follow)")

    # Test spiral pattern
    print("\n6. Testing SPIRAL pattern...")
    test_job['search_pattern'] = 'spiral'
    mission_sp = parse_mission(test_job)
    waypoints_sp = generate_waypoints(mission_sp)
    print(f"   Generated {len(waypoints_sp)} waypoints")

    # Test validation edge cases
    print("\n7. Testing validation edge cases...")

    # Invalid altitude
    bad_job = test_job.copy()
    bad_job['search_config'] = {'flight_altitude': -10, 'speed': 5}
    bad_mission = parse_mission(bad_job)
    valid, error = validate_mission(bad_mission)
    print(f"   Negative altitude: {error}")

    # Invalid speed
    bad_job['search_config'] = {'flight_altitude': 30, 'speed': 50}
    bad_mission = parse_mission(bad_job)
    valid, error = validate_mission(bad_mission)
    print(f"   High speed: {error}")

    print("\n" + "=" * 60)
    print("Tests complete!")
