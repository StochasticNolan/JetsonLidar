"""
Jetson GPS Reader Module

Reads GPS data from a USB GPS module (e.g., u-blox NEO-M8N) connected to the Jetson.
Provides position data for geo-tagging detected poles.

Features:
- NMEA sentence parsing (GGA, RMC)
- Position smoothing (EMA filter)
- Local coordinate conversion (NED offset to GPS)
- Fix quality monitoring

Usage:
    from gps_reader import JetsonGPS

    gps = JetsonGPS(port='/dev/ttyUSB1')

    # In main loop:
    if gps.update():
        lat, lon, alt = gps.get_position()
        if gps.has_fix():
            print(f"Position: {lat:.6f}, {lon:.6f}")
"""

import math
import time
import threading
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("Warning: pyserial not installed. Run: pip install pyserial")

try:
    import pynmea2
    PYNMEA_AVAILABLE = True
except ImportError:
    PYNMEA_AVAILABLE = False
    print("Warning: pynmea2 not installed. Run: pip install pynmea2")

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


# =============================================================================
# Constants
# =============================================================================

# WGS84 ellipsoid parameters
WGS84_A = 6378137.0  # Semi-major axis (meters)
WGS84_B = 6356752.314245  # Semi-minor axis (meters)
WGS84_F = 1 / 298.257223563  # Flattening


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class GPSConfig:
    """Configuration for GPS reader."""
    port: str = '/dev/ttyUSB1'
    baud: int = 9600
    timeout: float = 1.0
    min_satellites: int = 4
    update_rate_hz: float = 5.0
    smoothing_alpha: float = 0.3  # EMA smoothing (0=none, 1=max)
    max_age: float = 2.0  # Max age of position data (seconds)


@dataclass
class GPSPosition:
    """GPS position data."""
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0  # MSL
    fix_quality: int = 0  # 0=invalid, 1=GPS, 2=DGPS, 4=RTK, 5=Float RTK
    num_satellites: int = 0
    hdop: float = 99.9
    speed_knots: float = 0.0
    heading: float = 0.0  # True heading in degrees
    timestamp: float = 0.0


# =============================================================================
# GPS Reader
# =============================================================================

class JetsonGPS:
    """Reads GPS data from USB GPS module on Jetson."""

    def __init__(self, config: Optional[GPSConfig] = None,
                 config_path: Optional[str] = None):
        """Initialize GPS reader.

        Args:
            config: GPSConfig object
            config_path: Path to YAML config file
        """
        if config is not None:
            self.config = config
        elif config_path is not None:
            self.config = self._load_config(config_path)
        else:
            self.config = GPSConfig()

        # Serial connection
        self._serial: Optional[serial.Serial] = None
        self._connected = False

        # Position data
        self._position = GPSPosition()
        self._smoothed_lat: Optional[float] = None
        self._smoothed_lon: Optional[float] = None
        self._smoothed_alt: Optional[float] = None

        # Reference position for local coordinate conversion
        self._ref_lat: Optional[float] = None
        self._ref_lon: Optional[float] = None
        self._ref_alt: Optional[float] = None

        # Threading for background updates
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    def _load_config(self, config_path: str) -> GPSConfig:
        """Load configuration from YAML file."""
        config = GPSConfig()

        if not YAML_AVAILABLE:
            return config

        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)

            if 'gps' in data:
                g = data['gps']
                config.port = g.get('port', config.port)
                config.baud = g.get('baud', config.baud)
                config.min_satellites = g.get('min_satellites', config.min_satellites)
                config.update_rate_hz = g.get('update_rate_hz', config.update_rate_hz)

        except Exception as e:
            print(f"Warning: Could not load GPS config: {e}")

        return config

    def connect(self, port: Optional[str] = None, baud: Optional[int] = None) -> bool:
        """Connect to GPS module.

        Args:
            port: Serial port (overrides config)
            baud: Baud rate (overrides config)

        Returns:
            True if connection successful
        """
        if not SERIAL_AVAILABLE:
            print("Error: pyserial not available")
            return False

        if not PYNMEA_AVAILABLE:
            print("Error: pynmea2 not available")
            return False

        port = port or self.config.port
        baud = baud or self.config.baud

        try:
            print(f"Connecting to GPS on {port} at {baud} baud...")
            self._serial = serial.Serial(
                port,
                baud,
                timeout=self.config.timeout
            )
            self._connected = True
            print("GPS connected")
            return True

        except serial.SerialException as e:
            print(f"GPS connection failed: {e}")
            self._connected = False
            return False

    def disconnect(self):
        """Disconnect from GPS module."""
        self.stop_background()

        if self._serial:
            self._serial.close()
            self._serial = None

        self._connected = False

    def start_background(self):
        """Start background thread for continuous GPS updates."""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._background_loop, daemon=True)
        self._thread.start()

    def stop_background(self):
        """Stop background update thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _background_loop(self):
        """Background thread for reading GPS data."""
        while self._running:
            try:
                self.update()
                time.sleep(1.0 / self.config.update_rate_hz)
            except Exception as e:
                print(f"GPS background error: {e}")
                time.sleep(0.5)

    def update(self) -> bool:
        """Read and parse GPS data.

        Call this in your main loop if not using background thread.

        Returns:
            True if new valid position was received
        """
        if not self._connected or not self._serial:
            return False

        try:
            # Read available lines
            while self._serial.in_waiting > 0:
                line = self._serial.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$'):
                    self._parse_nmea(line)

            return self._position.fix_quality > 0

        except Exception as e:
            print(f"GPS read error: {e}")
            return False

    def _parse_nmea(self, sentence: str):
        """Parse NMEA sentence."""
        try:
            msg = pynmea2.parse(sentence)

            with self._lock:
                if isinstance(msg, pynmea2.GGA):
                    # GGA - Global Positioning System Fix Data
                    if msg.latitude and msg.longitude:
                        lat = msg.latitude
                        lon = msg.longitude
                        alt = float(msg.altitude) if msg.altitude else 0.0

                        # Apply smoothing
                        lat, lon, alt = self._smooth_position(lat, lon, alt)

                        self._position.latitude = lat
                        self._position.longitude = lon
                        self._position.altitude = alt
                        self._position.fix_quality = int(msg.gps_qual) if msg.gps_qual else 0
                        self._position.num_satellites = int(msg.num_sats) if msg.num_sats else 0
                        self._position.hdop = float(msg.horizontal_dil) if msg.horizontal_dil else 99.9
                        self._position.timestamp = time.time()

                elif isinstance(msg, pynmea2.RMC):
                    # RMC - Recommended Minimum Navigation Information
                    if msg.spd_over_grnd:
                        self._position.speed_knots = float(msg.spd_over_grnd)
                    if msg.true_course:
                        self._position.heading = float(msg.true_course)

        except pynmea2.ParseError:
            pass  # Ignore malformed sentences

    def _smooth_position(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """Apply EMA smoothing to position."""
        alpha = self.config.smoothing_alpha

        if self._smoothed_lat is None:
            self._smoothed_lat = lat
            self._smoothed_lon = lon
            self._smoothed_alt = alt
        else:
            self._smoothed_lat = alpha * self._smoothed_lat + (1 - alpha) * lat
            self._smoothed_lon = alpha * self._smoothed_lon + (1 - alpha) * lon
            self._smoothed_alt = alpha * self._smoothed_alt + (1 - alpha) * alt

        return self._smoothed_lat, self._smoothed_lon, self._smoothed_alt

    # =========================================================================
    # Position Access
    # =========================================================================

    def get_position(self) -> Tuple[float, float, float]:
        """Get current GPS position.

        Returns:
            (latitude, longitude, altitude_msl) in degrees and meters
        """
        with self._lock:
            return (
                self._position.latitude,
                self._position.longitude,
                self._position.altitude
            )

    def get_position_data(self) -> GPSPosition:
        """Get full position data including quality info.

        Returns:
            GPSPosition dataclass with all fields
        """
        with self._lock:
            return GPSPosition(
                latitude=self._position.latitude,
                longitude=self._position.longitude,
                altitude=self._position.altitude,
                fix_quality=self._position.fix_quality,
                num_satellites=self._position.num_satellites,
                hdop=self._position.hdop,
                speed_knots=self._position.speed_knots,
                heading=self._position.heading,
                timestamp=self._position.timestamp
            )

    def has_fix(self) -> bool:
        """Check if GPS has a valid fix.

        Returns:
            True if fix is valid and recent
        """
        with self._lock:
            if self._position.fix_quality == 0:
                return False
            if self._position.num_satellites < self.config.min_satellites:
                return False

            age = time.time() - self._position.timestamp
            if age > self.config.max_age:
                return False

            return True

    def get_fix_quality_str(self) -> str:
        """Get human-readable fix quality string."""
        quality_names = {
            0: "No Fix",
            1: "GPS",
            2: "DGPS",
            4: "RTK Fixed",
            5: "RTK Float"
        }
        with self._lock:
            return quality_names.get(self._position.fix_quality, "Unknown")

    # =========================================================================
    # Coordinate Conversion
    # =========================================================================

    def set_reference(self, lat: Optional[float] = None,
                      lon: Optional[float] = None,
                      alt: Optional[float] = None):
        """Set reference position for local coordinate conversion.

        If no arguments provided, uses current position as reference.

        Args:
            lat: Reference latitude (degrees)
            lon: Reference longitude (degrees)
            alt: Reference altitude (meters MSL)
        """
        if lat is None or lon is None:
            with self._lock:
                self._ref_lat = self._position.latitude
                self._ref_lon = self._position.longitude
                self._ref_alt = self._position.altitude if alt is None else alt
        else:
            self._ref_lat = lat
            self._ref_lon = lon
            self._ref_alt = alt if alt is not None else 0.0

    def local_to_gps(self, x: float, y: float, z: float = 0.0) -> Tuple[float, float, float]:
        """Convert local NED coordinates to GPS.

        Local frame: X=forward (North), Y=right (East), Z=down
        This matches the drone body frame when heading north.

        Args:
            x: North offset in meters
            y: East offset in meters
            z: Down offset in meters (positive = below reference)

        Returns:
            (latitude, longitude, altitude_msl) in degrees and meters
        """
        if self._ref_lat is None:
            # Use current position as reference
            self.set_reference()

        if self._ref_lat is None:
            return (0.0, 0.0, 0.0)

        # Convert lat/lon to radians
        ref_lat_rad = math.radians(self._ref_lat)
        ref_lon_rad = math.radians(self._ref_lon)

        # Earth radius at reference latitude (approximate)
        # Using local radius of curvature
        sin_lat = math.sin(ref_lat_rad)
        cos_lat = math.cos(ref_lat_rad)

        # Radius of curvature in the meridian (north-south)
        R_N = WGS84_A / math.sqrt(1 - (2 * WGS84_F - WGS84_F**2) * sin_lat**2)

        # Radius of curvature in the prime vertical (east-west)
        R_M = R_N * (1 - (2 * WGS84_F - WGS84_F**2)) / (1 - (2 * WGS84_F - WGS84_F**2) * sin_lat**2)

        # Convert NED offsets to lat/lon changes
        # d_lat = x / R_M (north offset affects latitude)
        # d_lon = y / (R_N * cos(lat)) (east offset affects longitude)
        d_lat = x / R_M
        d_lon = y / (R_N * cos_lat)

        # New position
        new_lat = self._ref_lat + math.degrees(d_lat)
        new_lon = self._ref_lon + math.degrees(d_lon)
        new_alt = self._ref_alt - z  # Z is down, so subtract for altitude

        return (new_lat, new_lon, new_alt)

    def gps_to_local(self, lat: float, lon: float, alt: float = 0.0) -> Tuple[float, float, float]:
        """Convert GPS coordinates to local NED.

        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude MSL in meters

        Returns:
            (x, y, z) in meters, NED frame relative to reference
        """
        if self._ref_lat is None:
            self.set_reference()

        if self._ref_lat is None:
            return (0.0, 0.0, 0.0)

        # Convert to radians
        ref_lat_rad = math.radians(self._ref_lat)
        ref_lon_rad = math.radians(self._ref_lon)
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)

        # Earth radius at reference
        sin_lat = math.sin(ref_lat_rad)
        cos_lat = math.cos(ref_lat_rad)

        R_N = WGS84_A / math.sqrt(1 - (2 * WGS84_F - WGS84_F**2) * sin_lat**2)
        R_M = R_N * (1 - (2 * WGS84_F - WGS84_F**2)) / (1 - (2 * WGS84_F - WGS84_F**2) * sin_lat**2)

        # Convert lat/lon differences to NED
        d_lat = lat_rad - ref_lat_rad
        d_lon = lon_rad - ref_lon_rad

        x = d_lat * R_M  # North
        y = d_lon * R_N * cos_lat  # East
        z = self._ref_alt - alt  # Down

        return (x, y, z)

    def pole_position_to_gps(self, pole_x: float, pole_y: float, pole_z: float,
                              drone_heading: float = 0.0) -> Tuple[float, float, float]:
        """Convert pole position (in drone body frame) to GPS.

        Pole positions from the perception system are in body frame:
        - X = forward
        - Y = left (positive)
        - Z = up

        This function converts to GPS coordinates.

        Args:
            pole_x: Pole X position (forward, meters)
            pole_y: Pole Y position (left, meters)
            pole_z: Pole Z position (up, meters) - usually ignored for GPS
            drone_heading: Drone heading in degrees (0 = North, 90 = East)

        Returns:
            (latitude, longitude, altitude_msl)
        """
        # Convert body frame to NED
        # Body: X=fwd, Y=left, Z=up
        # NED: X=north, Y=east, Z=down

        heading_rad = math.radians(drone_heading)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)

        # Rotate body XY to NED XY
        # Body Y is left (negative of right/east when heading north)
        north = pole_x * cos_h - (-pole_y) * sin_h
        east = pole_x * sin_h + (-pole_y) * cos_h
        down = -pole_z  # Body Z up -> NED Z down

        return self.local_to_gps(north, east, down)


# =============================================================================
# Convenience Functions
# =============================================================================

def create_gps(config_path: str = 'config/settings.yaml') -> JetsonGPS:
    """Create a JetsonGPS with config from file."""
    return JetsonGPS(config_path=config_path)


# =============================================================================
# Test / Demo
# =============================================================================

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Jetson GPS Reader Test')
    parser.add_argument('--port', default='/dev/ttyUSB1', help='Serial port')
    parser.add_argument('--baud', type=int, default=9600, help='Baud rate')
    args = parser.parse_args()

    # Create GPS reader
    config = GPSConfig(port=args.port, baud=args.baud)
    gps = JetsonGPS(config)

    if not gps.connect():
        print("Failed to connect to GPS")
        exit(1)

    print("\nGPS Reader Test")
    print("=" * 50)
    print("Reading GPS data... Press Ctrl+C to exit\n")

    # Set reference position when we get first fix
    ref_set = False

    try:
        while True:
            if gps.update():
                pos = gps.get_position_data()

                # Set reference on first fix
                if not ref_set and gps.has_fix():
                    gps.set_reference()
                    ref_set = True
                    print(f"Reference set: {pos.latitude:.6f}, {pos.longitude:.6f}")
                    print("-" * 50)

                print(f"Lat: {pos.latitude:.6f}  Lon: {pos.longitude:.6f}  "
                      f"Alt: {pos.altitude:.1f}m")
                print(f"Fix: {gps.get_fix_quality_str()}  "
                      f"Sats: {pos.num_satellites}  "
                      f"HDOP: {pos.hdop:.1f}")

                if ref_set:
                    # Test coordinate conversion
                    local = gps.gps_to_local(pos.latitude, pos.longitude, pos.altitude)
                    print(f"Local NED: ({local[0]:.2f}, {local[1]:.2f}, {local[2]:.2f})")

                print()

            time.sleep(0.5)

    except KeyboardInterrupt:
        pass

    finally:
        gps.disconnect()
        print("\nDisconnected")
