"""
MAVLink Guidance Module for Cube Blue Flight Controller

Converts corridor state to velocity commands and sends them to the
Cube Blue via MAVLink protocol.

Features:
- MAVLink connection over serial or UDP
- Body-frame velocity commands (SET_POSITION_TARGET_LOCAL_NED)
- State machine: SEARCH -> FOLLOW -> RESCAN -> ABORT
- Failsafe behaviors (loiter, RTL)
- Rate limiting and acceleration smoothing

Usage:
    from guidance import MavlinkGuidance, GuidanceConfig

    guidance = MavlinkGuidance(config)
    guidance.connect('/dev/ttyTHS1', baud=57600)
    guidance.arm_and_takeoff()

    # In control loop:
    guidance.update(corridor_state)
"""

import time
import threading
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("Warning: pymavlink not installed. Run: pip install pymavlink")

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


# =============================================================================
# Enums and Data Classes
# =============================================================================

class GuidanceState(Enum):
    """High-level guidance state machine states."""
    IDLE = "IDLE"           # Not active, waiting for command
    SEARCH = "SEARCH"       # Looking for initial pole pair
    FOLLOW = "FOLLOW"       # Normal corridor tracking
    RESCAN = "RESCAN"       # Lost poles, searching
    INSPECT_POLE = "INSPECT_POLE"  # Hovering at pole for capture
    OBSTACLE_STOP = "OBSTACLE_STOP"  # Emergency stop - obstacle detected
    ABORT = "ABORT"         # Failsafe triggered
    LANDED = "LANDED"       # On ground


@dataclass
class CorridorState:
    """Input from corridor follower."""
    lateral_error: float      # Signed distance from centerline (m), + = right of center
    heading_error: float      # Angle to centerline tangent (rad), + = pointing right
    look_ahead_x: float       # Look-ahead point X (forward, m)
    look_ahead_y: float       # Look-ahead point Y (left, m)
    confidence: float         # Detection confidence (0-1)
    num_poles: int            # Number of detected poles
    corridor_width: float     # Estimated corridor width (m)


@dataclass
class GuidanceConfig:
    """Configuration for guidance controller."""

    # Connection
    connection_string: str = '/dev/ttyTHS1'  # Serial port or UDP address
    baud_rate: int = 57600

    # Velocity limits (m/s)
    max_forward_vel: float = 2.0      # Max forward speed
    max_lateral_vel: float = 1.0      # Max side-to-side speed
    max_vertical_vel: float = 0.5     # Max up/down speed
    max_yaw_rate: float = 0.5         # Max yaw rate (rad/s)

    # Acceleration limits (m/s^2) for smoothing
    max_forward_accel: float = 1.0
    max_lateral_accel: float = 0.5
    max_yaw_accel: float = 0.3

    # Controller gains
    lateral_p: float = 0.5            # Proportional gain for lateral error
    lateral_d: float = 0.1            # Derivative gain for lateral error
    heading_p: float = 1.0            # Proportional gain for heading error
    heading_d: float = 0.2            # Derivative gain for heading error

    # Pure pursuit
    look_ahead_distance: float = 3.0  # Look-ahead distance (m)
    use_pure_pursuit: bool = True     # Use pure pursuit vs PD controller

    # State machine thresholds
    min_confidence_follow: float = 0.5   # Min confidence to stay in FOLLOW
    min_confidence_search: float = 0.3   # Min confidence to exit SEARCH
    min_poles_follow: int = 2            # Min poles to stay in FOLLOW
    rescan_timeout: float = 5.0          # Seconds in RESCAN before ABORT
    search_timeout: float = 30.0         # Seconds in SEARCH before ABORT

    # Failsafe
    heartbeat_timeout: float = 2.0    # Seconds without heartbeat = failsafe
    command_timeout: float = 0.5      # Seconds without update = hold position

    # Obstacle detection failsafe
    obstacle_stop_distance: float = 3.0     # Full stop distance (m)
    obstacle_slow_distance: float = 8.0     # Start slowing distance (m)
    obstacle_clear_distance: float = 5.0    # Distance to resume after stop (m)
    obstacle_timeout: float = 0.5           # Max age of obstacle data (s)

    # Control rate
    control_rate_hz: float = 20.0     # Velocity command rate


@dataclass
class VelocityCommand:
    """Body-frame velocity command."""
    vx: float = 0.0       # Forward velocity (m/s)
    vy: float = 0.0       # Lateral velocity (m/s), + = left
    vz: float = 0.0       # Vertical velocity (m/s), + = up (will be negated for NED)
    yaw_rate: float = 0.0 # Yaw rate (rad/s), + = CCW


# =============================================================================
# MAVLink Guidance Controller
# =============================================================================

class MavlinkGuidance:
    """MAVLink-based guidance controller for Cube Blue."""

    def __init__(self, config: Optional[GuidanceConfig] = None,
                 config_path: Optional[str] = None):
        """Initialize guidance controller.

        Args:
            config: GuidanceConfig object
            config_path: Path to YAML config file
        """
        if not MAVLINK_AVAILABLE:
            raise RuntimeError("pymavlink not available. Install with: pip install pymavlink")

        if config is not None:
            self.config = config
        elif config_path is not None:
            self.config = self._load_config(config_path)
        else:
            self.config = GuidanceConfig()

        # MAVLink connection
        self.mav: Optional[mavutil.mavlink_connection] = None
        self.connected = False
        self.armed = False
        self.in_offboard = False

        # State machine
        self.state = GuidanceState.IDLE
        self._state_enter_time = time.time()

        # Control state
        self._last_cmd = VelocityCommand()
        self._last_update_time = time.time()
        self._last_corridor_state: Optional[CorridorState] = None

        # Error derivatives for PD control
        self._prev_lateral_error = 0.0
        self._prev_heading_error = 0.0
        self._prev_time = time.time()

        # Heartbeat monitoring
        self._last_heartbeat = 0.0
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._running = False

        # Telemetry
        self._position = (0.0, 0.0, 0.0)  # x, y, z in local frame
        self._velocity = (0.0, 0.0, 0.0)  # vx, vy, vz
        self._attitude = (0.0, 0.0, 0.0)  # roll, pitch, yaw

        # Flight mode tracking (from RC controller)
        self._current_mode = 0            # ArduCopter custom mode number
        self._mode_name = "UNKNOWN"       # Human-readable mode name

        # ArduCopter mode numbers
        self.MODES = {
            0: "STABILIZE",
            1: "ACRO",
            2: "ALT_HOLD",
            3: "AUTO",
            4: "GUIDED",      # ← Jetson controls in this mode
            5: "LOITER",
            6: "RTL",
            7: "CIRCLE",
            9: "LAND",
            16: "POSHOLD",
            17: "BRAKE",
            21: "SMART_RTL",
        }

        # Obstacle detection state
        self._obstacle_detected = False
        self._obstacle_distance = float('inf')
        self._obstacle_timestamp = 0.0
        self._pre_obstacle_state: Optional[GuidanceState] = None  # State before obstacle stop

        # RC channel tracking (for Herelink buttons)
        # Herelink: A=ch13, B=ch14, C=ch15, D=ch16
        self._rc_channels = [0] * 18  # RC channels 1-18
        self._button_c_state = False  # Current state of C button
        self._button_c_pressed = False  # Edge detection: True when just pressed
        self._button_c_threshold = 1500  # PWM threshold (>1500 = pressed)

        # Inspection state tracking
        self._inspecting_pole_id: int = -1  # ID of pole being inspected
        self._inspection_start_time: float = 0.0  # When inspection started
        self._inspection_position: tuple = (0.0, 0.0, 0.0)  # Hold position during inspection
        self._inspection_complete_callback = None  # Callback when inspection done

    def _load_config(self, config_path: str) -> GuidanceConfig:
        """Load configuration from YAML file."""
        config = GuidanceConfig()

        if not YAML_AVAILABLE:
            return config

        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)

            if 'guidance' in data:
                g = data['guidance']
                config.max_lateral_vel = g.get('max_lateral_velocity', config.max_lateral_vel)
                config.max_yaw_rate = g.get('max_angular_velocity', config.max_yaw_rate)
                config.lateral_p = g.get('lateral_gain', config.lateral_p)
                config.heading_p = g.get('heading_gain', config.heading_p)

        except Exception as e:
            print(f"Warning: Could not load guidance config: {e}")

        return config

    # =========================================================================
    # Connection Management
    # =========================================================================

    def connect(self, connection_string: Optional[str] = None,
                baud: Optional[int] = None) -> bool:
        """Connect to flight controller via MAVLink.

        Args:
            connection_string: Serial port or UDP address
                Examples: '/dev/ttyTHS1', '/dev/ttyUSB0', 'udp:127.0.0.1:14550'
            baud: Baud rate for serial connections

        Returns:
            True if connection successful
        """
        conn_str = connection_string or self.config.connection_string
        baud_rate = baud or self.config.baud_rate

        try:
            print(f"Connecting to {conn_str}...")

            if conn_str.startswith('udp') or conn_str.startswith('tcp'):
                self.mav = mavutil.mavlink_connection(conn_str)
            else:
                self.mav = mavutil.mavlink_connection(conn_str, baud=baud_rate)

            # Wait for heartbeat
            print("Waiting for heartbeat...")
            self.mav.wait_heartbeat(timeout=10)
            self._last_heartbeat = time.time()

            print(f"Connected to system {self.mav.target_system}, "
                  f"component {self.mav.target_component}")

            self.connected = True

            # Start heartbeat monitoring thread
            self._running = True
            self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
            self._heartbeat_thread.start()

            # Request data streams
            self._request_data_streams()

            return True

        except Exception as e:
            print(f"Connection failed: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Disconnect from flight controller."""
        self._running = False
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=2.0)

        if self.mav:
            self.mav.close()
            self.mav = None

        self.connected = False
        self.armed = False
        self.in_offboard = False

    def _request_data_streams(self):
        """Request telemetry data streams from FCU."""
        if not self.mav:
            return

        # Request position and attitude at 10Hz
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10, 1
        )

        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # Attitude
            10, 1
        )

        # Request RC channels at 10Hz (for Herelink buttons)
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            10, 1
        )

    def _heartbeat_loop(self):
        """Background thread for heartbeat and telemetry."""
        while self._running:
            if not self.mav:
                time.sleep(0.1)
                continue

            # Send heartbeat to FCU
            self.mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )

            # Receive messages
            try:
                while True:
                    msg = self.mav.recv_match(blocking=False)
                    if msg is None:
                        break

                    msg_type = msg.get_type()

                    if msg_type == 'HEARTBEAT':
                        self._last_heartbeat = time.time()
                        self.armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

                        # Track flight mode from RC controller
                        self._current_mode = msg.custom_mode
                        self._mode_name = self.MODES.get(msg.custom_mode, f"MODE_{msg.custom_mode}")

                    elif msg_type == 'LOCAL_POSITION_NED':
                        self._position = (msg.x, msg.y, msg.z)
                        self._velocity = (msg.vx, msg.vy, msg.vz)

                    elif msg_type == 'ATTITUDE':
                        self._attitude = (msg.roll, msg.pitch, msg.yaw)

                    elif msg_type == 'GLOBAL_POSITION_INT':
                        # GPS position in degrees (lat/lon scaled by 1e7)
                        self._gps_lat = msg.lat / 1e7
                        self._gps_lon = msg.lon / 1e7
                        self._gps_alt = msg.relative_alt / 1000.0  # mm to m (relative to home)
                        self._gps_heading = msg.hdg / 100.0  # cdeg to deg

                    elif msg_type == 'SYS_STATUS':
                        # Battery status
                        if msg.battery_remaining >= 0:
                            self._battery_percent = msg.battery_remaining
                        self._battery_voltage = msg.voltage_battery / 1000.0  # mV to V

                    elif msg_type == 'RC_CHANNELS':
                        # Store all RC channels
                        self._rc_channels = [
                            msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                            msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
                            msg.chan9_raw, msg.chan10_raw, msg.chan11_raw, msg.chan12_raw,
                            msg.chan13_raw, msg.chan14_raw, msg.chan15_raw, msg.chan16_raw,
                            msg.chan17_raw, msg.chan18_raw
                        ]
                        # Check Herelink C button (channel 15, index 14)
                        c_button_pwm = self._rc_channels[14]
                        new_state = c_button_pwm > self._button_c_threshold

                        # Edge detection: detect rising edge (button just pressed)
                        if new_state and not self._button_c_state:
                            self._button_c_pressed = True

                        self._button_c_state = new_state

            except Exception:
                pass

            time.sleep(0.05)  # 20Hz heartbeat loop

    # =========================================================================
    # Arming and Mode Control
    # =========================================================================

    def arm(self) -> bool:
        """Arm the vehicle.

        Returns:
            True if arm command sent successfully
        """
        if not self.mav or not self.connected:
            print("Not connected")
            return False

        print("Arming...")
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0, 0, 0, 0, 0, 0
        )

        # Wait for arm
        start = time.time()
        while time.time() - start < 5.0:
            if self.armed:
                print("Armed")
                return True
            time.sleep(0.1)

        print("Arm timeout")
        return False

    def disarm(self) -> bool:
        """Disarm the vehicle."""
        if not self.mav:
            return False

        print("Disarming...")
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0,  # disarm
            0, 0, 0, 0, 0, 0
        )

        time.sleep(1.0)
        self.armed = False
        return True

    def set_guided_mode(self) -> bool:
        """Set vehicle to GUIDED mode (ArduPilot) for offboard control.

        Returns:
            True if mode change successful
        """
        if not self.mav:
            return False

        # GUIDED mode for ArduCopter
        guided_mode = 4

        print("Setting GUIDED mode...")
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            guided_mode
        )

        time.sleep(0.5)
        self.in_offboard = True
        return True

    def set_loiter_mode(self) -> bool:
        """Set vehicle to LOITER mode (hold position)."""
        if not self.mav:
            return False

        loiter_mode = 5  # LOITER for ArduCopter

        print("Setting LOITER mode...")
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            loiter_mode
        )

        self.in_offboard = False
        return True

    def is_guided_mode(self) -> bool:
        """Check if FCU is in GUIDED mode (accepts Jetson commands).

        Returns:
            True if pilot has selected GUIDED mode on RC controller
        """
        return self._current_mode == 4  # GUIDED = 4

    def get_mode_name(self) -> str:
        """Get current flight mode name.

        Returns:
            Human-readable mode name (e.g., "GUIDED", "LOITER", "STABILIZE")
        """
        return self._mode_name

    def return_to_launch(self) -> bool:
        """Command Return to Launch (RTL)."""
        if not self.mav:
            return False

        rtl_mode = 6  # RTL for ArduCopter

        print("Setting RTL mode...")
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            rtl_mode
        )

        self.in_offboard = False
        self.state = GuidanceState.ABORT
        return True

    # =========================================================================
    # High-Level Flight Commands (Takeoff, Navigate, Land)
    # =========================================================================

    def takeoff(self, altitude: float, timeout: float = 60.0) -> bool:
        """Arm and takeoff to specified altitude.

        Sets GUIDED mode, arms the vehicle, and commands takeoff to the
        specified altitude AGL. Blocks until altitude is reached or timeout.

        Args:
            altitude: Target altitude above ground level (meters)
            timeout: Maximum time to wait for takeoff (seconds)

        Returns:
            True if takeoff successful and altitude reached
        """
        if not self.mav or not self.connected:
            print("Not connected to FCU")
            return False

        print(f"Takeoff sequence to {altitude}m")

        # 1. Set GUIDED mode
        if not self.set_guided_mode():
            print("Failed to set GUIDED mode")
            return False
        time.sleep(0.5)

        # 2. Arm
        if not self.armed:
            if not self.arm():
                print("Failed to arm")
                return False
        time.sleep(0.5)

        # 3. Send takeoff command
        print(f"Commanding takeoff to {altitude}m...")
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,                  # confirmation
            0,                  # param1: pitch (ignored for copter)
            0,                  # param2: empty
            0,                  # param3: empty
            float('nan'),       # param4: yaw (NaN = current heading)
            0,                  # param5: latitude (0 = current)
            0,                  # param6: longitude (0 = current)
            altitude            # param7: altitude (m)
        )

        # 4. Wait for altitude
        start_time = time.time()
        while time.time() - start_time < timeout:
            # Get current altitude from telemetry
            current_alt = self._position[2]  # Z in local frame

            # Check if we've reached target (within 5%)
            if current_alt >= altitude * 0.95:
                print(f"Reached target altitude: {current_alt:.1f}m")
                self.state = GuidanceState.IDLE
                return True

            # Progress update
            if int(time.time() - start_time) % 2 == 0:
                print(f"  Altitude: {current_alt:.1f}m / {altitude}m")

            time.sleep(0.5)

        print(f"Takeoff timeout - current altitude: {self._position[2]:.1f}m")
        return False

    def land(self) -> bool:
        """Command vehicle to land at current location.

        Returns:
            True if land command sent successfully
        """
        if not self.mav:
            return False

        land_mode = 9  # LAND for ArduCopter

        print("Commanding LAND...")
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            land_mode
        )

        self.in_offboard = False
        self.state = GuidanceState.LANDED
        return True

    def navigate_to_waypoint(self, lat: float, lon: float, alt: float,
                              speed: float = 3.0,
                              threshold_m: float = 3.0,
                              timeout: float = 300.0) -> bool:
        """Navigate to GPS waypoint using ArduPilot's internal navigation.

        Sends a GUIDED mode waypoint command and waits for arrival.
        Requires GUIDED mode to be active.

        Args:
            lat: Target latitude (degrees)
            lon: Target longitude (degrees)
            alt: Target altitude AGL (meters)
            speed: Cruise speed (m/s)
            threshold_m: Arrival threshold (meters)
            timeout: Maximum navigation time (seconds)

        Returns:
            True if waypoint reached within timeout
        """
        if not self.mav or not self.connected:
            print("Not connected to FCU")
            return False

        if not self.is_guided_mode():
            print("Not in GUIDED mode - cannot navigate")
            return False

        print(f"Navigating to ({lat:.6f}, {lon:.6f}) @ {alt}m")

        # Set cruise speed
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,                  # confirmation
            0,                  # speed type (0=airspeed, 1=groundspeed)
            speed,              # speed (m/s)
            -1,                 # throttle (-1 = no change)
            0, 0, 0, 0
        )

        # Send position target (global frame)
        # Convert altitude to AMSL if needed (here we assume AGL = AMSL for simplicity)
        self.mav.mav.set_position_target_global_int_send(
            0,                  # time_boot_ms
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # type_mask: use position only
            int(lat * 1e7),     # lat_int
            int(lon * 1e7),     # lon_int
            alt,                # alt (relative to home)
            0, 0, 0,            # velocity (ignored)
            0, 0, 0,            # acceleration (ignored)
            0,                  # yaw (ignored)
            0                   # yaw_rate (ignored)
        )

        # Wait for arrival
        start_time = time.time()
        last_log_time = 0

        while time.time() - start_time < timeout:
            # Check if still in GUIDED mode
            if not self.is_guided_mode():
                print("Exited GUIDED mode - navigation aborted")
                return False

            # Calculate distance to target (simplified)
            distance = self._distance_to_waypoint(lat, lon)

            # Arrival check
            if distance < threshold_m:
                print(f"Arrived at waypoint (distance: {distance:.1f}m)")
                return True

            # Progress logging (every 5 seconds)
            if time.time() - last_log_time > 5.0:
                print(f"  Distance to waypoint: {distance:.0f}m")
                last_log_time = time.time()

            time.sleep(0.5)

        print(f"Navigation timeout - distance: {self._distance_to_waypoint(lat, lon):.0f}m")
        return False

    def _distance_to_waypoint(self, target_lat: float, target_lon: float) -> float:
        """Calculate approximate distance to waypoint in meters.

        Uses local position if available, otherwise estimates from GPS.
        """
        # For now, use a simple approximation
        # In practice, should use GPS telemetry from FCU
        current_lat, current_lon = self.get_gps_position()
        if current_lat == 0 and current_lon == 0:
            return float('inf')  # No GPS fix

        # Haversine approximation
        import math
        dlat = math.radians(target_lat - current_lat)
        dlon = math.radians(target_lon - current_lon)
        a = math.sin(dlat/2)**2 + \
            math.cos(math.radians(current_lat)) * \
            math.cos(math.radians(target_lat)) * \
            math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return 6371000 * c  # Earth radius in meters

    def get_gps_position(self) -> Tuple[float, float]:
        """Get current GPS position.

        Returns:
            Tuple of (latitude, longitude) in degrees, or (0, 0) if unavailable
        """
        # This should be populated from GLOBAL_POSITION_INT messages
        # For now, return placeholder - needs telemetry integration
        return getattr(self, '_gps_lat', 0.0), getattr(self, '_gps_lon', 0.0)

    def get_altitude(self) -> float:
        """Get current altitude AGL in meters."""
        return self._position[2]

    def get_battery_percent(self) -> float:
        """Get current battery percentage.

        Returns:
            Battery percentage (0-100), or -1 if unavailable
        """
        return getattr(self, '_battery_percent', -1.0)

    def hold_position(self):
        """Command vehicle to hold current position.

        Sends zero velocity command to maintain position.
        """
        self.send_velocity(0, 0, 0, 0)

    # =========================================================================
    # Velocity Commands
    # =========================================================================

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """Send body-frame velocity command to FCU.

        IMPORTANT: Only sends commands when FCU is in GUIDED mode.
        Pilot controls mode via RC transmitter - flip to GUIDED to enable
        autonomous control, flip away to disable.

        Args:
            vx: Forward velocity (m/s), + = forward
            vy: Lateral velocity (m/s), + = left
            vz: Vertical velocity (m/s), + = up
            yaw_rate: Yaw rate (rad/s), + = CCW

        Note: Converts from our convention to MAVLink NED body frame.
        """
        if not self.mav or not self.connected:
            return

        # SAFETY: Only send velocity commands when pilot has selected GUIDED mode
        if not self.is_guided_mode():
            return  # Pilot has control - do not send commands

        # Apply velocity limits
        vx = max(-self.config.max_forward_vel, min(self.config.max_forward_vel, vx))
        vy = max(-self.config.max_lateral_vel, min(self.config.max_lateral_vel, vy))
        vz = max(-self.config.max_vertical_vel, min(self.config.max_vertical_vel, vz))
        yaw_rate = max(-self.config.max_yaw_rate, min(self.config.max_yaw_rate, yaw_rate))

        # Convert to NED body frame:
        # Our frame: X=forward, Y=left, Z=up
        # NED body:  X=forward, Y=right, Z=down
        vx_ned = vx
        vy_ned = -vy       # Left -> Right
        vz_ned = -vz       # Up -> Down

        # Type mask: ignore position, acceleration; use velocity and yaw_rate
        # Bits: pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, yaw, yaw_rate
        # We want velocity (bits 3,4,5 = 0) and yaw_rate (bit 10 = 0)
        type_mask = 0b010111000111  # Use velocity and yaw_rate only

        self.mav.mav.set_position_target_local_ned_send(
            0,                              # time_boot_ms (0 = use system time)
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,                        # position (ignored)
            vx_ned, vy_ned, vz_ned,         # velocity
            0, 0, 0,                        # acceleration (ignored)
            0,                              # yaw (ignored)
            yaw_rate                        # yaw_rate
        )

        self._last_cmd = VelocityCommand(vx, vy, vz, yaw_rate)

    def send_velocity_smoothed(self, cmd: VelocityCommand):
        """Send velocity command with acceleration limiting.

        Args:
            cmd: Target velocity command
        """
        dt = time.time() - self._prev_time
        if dt <= 0:
            dt = 0.05

        # Rate limit each axis
        def rate_limit(current, target, max_accel, dt):
            diff = target - current
            max_change = max_accel * dt
            if abs(diff) > max_change:
                return current + max_change * (1 if diff > 0 else -1)
            return target

        vx = rate_limit(self._last_cmd.vx, cmd.vx, self.config.max_forward_accel, dt)
        vy = rate_limit(self._last_cmd.vy, cmd.vy, self.config.max_lateral_accel, dt)
        yaw_rate = rate_limit(self._last_cmd.yaw_rate, cmd.yaw_rate,
                              self.config.max_yaw_accel, dt)

        self.send_velocity(vx, vy, cmd.vz, yaw_rate)

    def stop(self):
        """Command zero velocity (hover in place)."""
        self.send_velocity(0, 0, 0, 0)

    # =========================================================================
    # Obstacle Detection Failsafe
    # =========================================================================

    def update_obstacle(self, detected: bool, distance: float):
        """Update obstacle detection state.

        Call this from the perception loop with obstacle detection results.

        Args:
            detected: Whether an obstacle is detected in the flight corridor
            distance: Distance to the closest obstacle (m)
        """
        self._obstacle_detected = detected
        self._obstacle_distance = distance if detected else float('inf')
        self._obstacle_timestamp = time.time()

        # Check for emergency stop condition
        if detected and distance <= self.config.obstacle_stop_distance:
            self._trigger_obstacle_stop()

    def _trigger_obstacle_stop(self):
        """Trigger emergency stop due to obstacle."""
        if self.state != GuidanceState.OBSTACLE_STOP:
            print(f"OBSTACLE STOP! Distance: {self._obstacle_distance:.1f}m")
            self._pre_obstacle_state = self.state
            self._set_state(GuidanceState.OBSTACLE_STOP)
            self.stop()  # Immediate stop command

    def _check_obstacle_clear(self) -> bool:
        """Check if obstacle has cleared and we can resume.

        Returns:
            True if safe to resume previous state
        """
        # Check if obstacle data is stale
        data_age = time.time() - self._obstacle_timestamp
        if data_age > self.config.obstacle_timeout:
            # No recent obstacle data - assume clear but be cautious
            return True

        # Check if obstacle has moved away
        if not self._obstacle_detected:
            return True

        if self._obstacle_distance >= self.config.obstacle_clear_distance:
            return True

        return False

    def get_obstacle_state(self) -> tuple:
        """Get current obstacle detection state.

        Returns:
            Tuple of (detected, distance, is_emergency)
        """
        is_emergency = (self._obstacle_detected and
                       self._obstacle_distance <= self.config.obstacle_stop_distance)
        return (self._obstacle_detected, self._obstacle_distance, is_emergency)

    # =========================================================================
    # State Machine
    # =========================================================================

    def _set_state(self, new_state: GuidanceState):
        """Transition to new state."""
        if new_state != self.state:
            print(f"Guidance: {self.state.value} -> {new_state.value}")
            self.state = new_state
            self._state_enter_time = time.time()

    def _state_duration(self) -> float:
        """Time in current state (seconds)."""
        return time.time() - self._state_enter_time

    def start_mission(self):
        """Start the guidance mission (enter SEARCH state)."""
        if not self.connected:
            print("Not connected to FCU")
            return

        self._set_state(GuidanceState.SEARCH)

    def stop_mission(self):
        """Stop the mission and loiter."""
        self.stop()
        self.set_loiter_mode()
        self._set_state(GuidanceState.IDLE)

    # =========================================================================
    # Main Control Loop
    # =========================================================================

    def update(self, corridor_state: Optional[CorridorState] = None) -> VelocityCommand:
        """Update guidance with new corridor state.

        This is the main control loop entry point. Call this at ~20Hz
        with the latest corridor state from the perception system.

        Args:
            corridor_state: Current corridor state from corridor follower

        Returns:
            Velocity command being sent to FCU
        """
        now = time.time()
        dt = now - self._prev_time
        self._prev_time = now

        # Check for failsafe conditions
        if self._check_failsafe():
            return self._last_cmd

        # Store corridor state
        if corridor_state is not None:
            self._last_corridor_state = corridor_state
            self._last_update_time = now

        # Check for stale data
        data_age = now - self._last_update_time
        if data_age > self.config.command_timeout and self.state == GuidanceState.FOLLOW:
            print(f"Warning: Stale corridor data ({data_age:.1f}s)")
            self._set_state(GuidanceState.RESCAN)

        # State machine
        cmd = VelocityCommand()

        if self.state == GuidanceState.IDLE:
            cmd = VelocityCommand()  # Zero velocity

        elif self.state == GuidanceState.SEARCH:
            cmd = self._search_behavior()

        elif self.state == GuidanceState.FOLLOW:
            cmd = self._follow_behavior(dt)

        elif self.state == GuidanceState.RESCAN:
            cmd = self._rescan_behavior()

        elif self.state == GuidanceState.OBSTACLE_STOP:
            cmd = self._obstacle_stop_behavior()

        elif self.state == GuidanceState.INSPECT_POLE:
            dwell = getattr(self, '_inspection_dwell_time', 2.0)
            cmd = self._inspect_pole_behavior(dwell)

        elif self.state == GuidanceState.ABORT:
            cmd = VelocityCommand()  # FCU handles RTL/loiter

        # Send command (except when stopped or in failsafe states)
        stopped_states = [GuidanceState.IDLE, GuidanceState.ABORT,
                         GuidanceState.LANDED, GuidanceState.OBSTACLE_STOP]
        if self.state not in stopped_states:
            self.send_velocity_smoothed(cmd)

        return cmd

    def _check_failsafe(self) -> bool:
        """Check failsafe conditions.

        Returns:
            True if failsafe triggered
        """
        # Heartbeat timeout
        if time.time() - self._last_heartbeat > self.config.heartbeat_timeout:
            print("FAILSAFE: Lost heartbeat from FCU")
            self._set_state(GuidanceState.ABORT)
            return True

        return False

    def _search_behavior(self) -> VelocityCommand:
        """Behavior for SEARCH state: look for initial pole pair."""
        cs = self._last_corridor_state

        # Check if we found poles
        if cs and cs.confidence >= self.config.min_confidence_search:
            if cs.num_poles >= self.config.min_poles_follow:
                print(f"Found {cs.num_poles} poles, confidence {cs.confidence:.2f}")
                self._set_state(GuidanceState.FOLLOW)
                return VelocityCommand()

        # Timeout check
        if self._state_duration() > self.config.search_timeout:
            print("SEARCH timeout - aborting")
            self.return_to_launch()
            return VelocityCommand()

        # Search behavior: slow rotation to scan for poles
        return VelocityCommand(
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.2  # Slow CCW rotation
        )

    def _follow_behavior(self, dt: float) -> VelocityCommand:
        """Behavior for FOLLOW state: track corridor centerline."""
        cs = self._last_corridor_state

        if cs is None:
            self._set_state(GuidanceState.RESCAN)
            return VelocityCommand()

        # Check if we should drop to RESCAN
        if cs.confidence < self.config.min_confidence_follow:
            print(f"Low confidence ({cs.confidence:.2f}), entering RESCAN")
            self._set_state(GuidanceState.RESCAN)
            return VelocityCommand()

        if cs.num_poles < self.config.min_poles_follow:
            print(f"Lost poles ({cs.num_poles}), entering RESCAN")
            self._set_state(GuidanceState.RESCAN)
            return VelocityCommand()

        # Compute control
        if self.config.use_pure_pursuit:
            cmd = self._pure_pursuit_control(cs)
        else:
            cmd = self._pd_control(cs, dt)

        # Scale forward velocity by confidence
        cmd.vx *= cs.confidence

        return cmd

    def _pd_control(self, cs: CorridorState, dt: float) -> VelocityCommand:
        """PD controller for corridor following.

        Args:
            cs: Current corridor state
            dt: Time step

        Returns:
            Velocity command
        """
        # Lateral control (correct side-to-side error)
        lateral_error = cs.lateral_error
        lateral_deriv = (lateral_error - self._prev_lateral_error) / dt if dt > 0 else 0
        self._prev_lateral_error = lateral_error

        vy = -(self.config.lateral_p * lateral_error +
               self.config.lateral_d * lateral_deriv)

        # Heading control (correct yaw to align with corridor)
        heading_error = cs.heading_error
        heading_deriv = (heading_error - self._prev_heading_error) / dt if dt > 0 else 0
        self._prev_heading_error = heading_error

        yaw_rate = -(self.config.heading_p * heading_error +
                     self.config.heading_d * heading_deriv)

        # Forward velocity (proportional to confidence, reduced when correcting)
        correction_magnitude = abs(lateral_error) + abs(heading_error)
        vx = self.config.max_forward_vel * max(0.3, 1.0 - correction_magnitude)

        return VelocityCommand(vx=vx, vy=vy, vz=0.0, yaw_rate=yaw_rate)

    def _pure_pursuit_control(self, cs: CorridorState) -> VelocityCommand:
        """Pure pursuit controller for corridor following.

        Args:
            cs: Current corridor state

        Returns:
            Velocity command
        """
        import math

        # Look-ahead point in body frame
        lx = cs.look_ahead_x
        ly = cs.look_ahead_y

        # Distance to look-ahead point
        L = math.sqrt(lx*lx + ly*ly)
        if L < 0.1:
            return VelocityCommand(vx=0.5, vy=0.0, vz=0.0, yaw_rate=0.0)

        # Curvature = 2 * y / L^2
        curvature = 2.0 * ly / (L * L)

        # Velocity
        vx = self.config.max_forward_vel
        yaw_rate = vx * curvature

        # Add lateral correction for immediate error
        vy = -self.config.lateral_p * cs.lateral_error * 0.5

        return VelocityCommand(vx=vx, vy=vy, vz=0.0, yaw_rate=yaw_rate)

    def _rescan_behavior(self) -> VelocityCommand:
        """Behavior for RESCAN state: slow down and search for poles."""
        cs = self._last_corridor_state

        # Check if we reacquired poles
        if cs and cs.confidence >= self.config.min_confidence_follow:
            if cs.num_poles >= self.config.min_poles_follow:
                print("Reacquired poles, resuming FOLLOW")
                self._set_state(GuidanceState.FOLLOW)
                return VelocityCommand()

        # Timeout check
        if self._state_duration() > self.config.rescan_timeout:
            print("RESCAN timeout - aborting")
            self.return_to_launch()
            return VelocityCommand()

        # Slow forward motion while scanning
        return VelocityCommand(
            vx=0.3,   # Slow forward
            vy=0.0,
            vz=0.0,
            yaw_rate=0.1  # Slight yaw to scan
        )

    def _obstacle_stop_behavior(self) -> VelocityCommand:
        """Behavior for OBSTACLE_STOP state: hold position until clear."""
        # Keep sending stop command
        self.stop()

        # Check if obstacle has cleared
        if self._check_obstacle_clear():
            # Resume previous state
            resume_state = self._pre_obstacle_state or GuidanceState.FOLLOW
            print(f"Obstacle cleared! Resuming {resume_state.value}")
            self._set_state(resume_state)
            self._pre_obstacle_state = None

        return VelocityCommand()  # Zero velocity

    def _inspect_pole_behavior(self, dwell_time: float = 2.0) -> VelocityCommand:
        """Behavior for INSPECT_POLE state: hover in place for capture.

        Args:
            dwell_time: How long to hover (seconds)

        Returns:
            Zero velocity command (hover)
        """
        # Check if inspection time complete
        elapsed = time.time() - self._inspection_start_time
        if elapsed >= dwell_time:
            # Inspection complete - trigger callback and resume FOLLOW
            if self._inspection_complete_callback:
                self._inspection_complete_callback(self._inspecting_pole_id)
            print(f"Inspection complete for pole {self._inspecting_pole_id}")
            self._inspecting_pole_id = -1
            self._set_state(GuidanceState.FOLLOW)

        # Hold position (zero velocity)
        return VelocityCommand()

    def start_inspection(self, pole_id: int, callback=None, dwell_time: float = 2.0):
        """Start pole inspection (hover for capture).

        Call this when the drone reaches capture position for a pole.

        Args:
            pole_id: ID of pole being inspected
            callback: Function to call when inspection complete (receives pole_id)
            dwell_time: How long to hover (seconds)
        """
        self._inspecting_pole_id = pole_id
        self._inspection_start_time = time.time()
        self._inspection_complete_callback = callback
        self._inspection_dwell_time = dwell_time
        self._set_state(GuidanceState.INSPECT_POLE)
        print(f"Starting inspection of pole {pole_id} (dwell: {dwell_time}s)")

    def cancel_inspection(self):
        """Cancel current inspection and resume FOLLOW."""
        if self.state == GuidanceState.INSPECT_POLE:
            print(f"Inspection of pole {self._inspecting_pole_id} cancelled")
            self._inspecting_pole_id = -1
            self._set_state(GuidanceState.FOLLOW)

    def is_inspecting(self) -> bool:
        """Check if currently inspecting a pole."""
        return self.state == GuidanceState.INSPECT_POLE

    def get_inspection_progress(self) -> float:
        """Get inspection progress (0.0 to 1.0)."""
        if self.state != GuidanceState.INSPECT_POLE:
            return 0.0
        elapsed = time.time() - self._inspection_start_time
        dwell = getattr(self, '_inspection_dwell_time', 2.0)
        return min(1.0, elapsed / dwell)

    # =========================================================================
    # Telemetry Access
    # =========================================================================

    def get_position(self) -> Tuple[float, float, float]:
        """Get current position in local frame (x, y, z)."""
        return self._position

    def get_velocity(self) -> Tuple[float, float, float]:
        """Get current velocity (vx, vy, vz)."""
        return self._velocity

    def get_attitude(self) -> Tuple[float, float, float]:
        """Get current attitude (roll, pitch, yaw) in radians."""
        return self._attitude

    def get_state(self) -> GuidanceState:
        """Get current guidance state."""
        return self.state

    def is_healthy(self) -> bool:
        """Check if guidance system is healthy."""
        if not self.connected:
            return False
        if time.time() - self._last_heartbeat > self.config.heartbeat_timeout:
            return False
        return True

    # =========================================================================
    # Herelink Button Access
    # =========================================================================

    def check_button_c_pressed(self) -> bool:
        """Check if Herelink C button was just pressed (rising edge).

        Returns True once per button press, then resets.
        Call this in your main loop to detect button presses.

        Returns:
            True if button was just pressed, False otherwise
        """
        if self._button_c_pressed:
            self._button_c_pressed = False  # Clear the flag
            return True
        return False

    def get_button_c_state(self) -> bool:
        """Get current state of Herelink C button.

        Returns:
            True if button is currently held down
        """
        return self._button_c_state

    def get_rc_channel(self, channel: int) -> int:
        """Get raw PWM value for an RC channel.

        Args:
            channel: RC channel number (1-18)

        Returns:
            PWM value (typically 1000-2000, 0 if not available)
        """
        if 1 <= channel <= 18:
            return self._rc_channels[channel - 1]
        return 0


# =============================================================================
# Convenience Functions
# =============================================================================

def create_guidance(config_path: str = 'config/settings.yaml') -> MavlinkGuidance:
    """Create a MavlinkGuidance instance with config from file."""
    return MavlinkGuidance(config_path=config_path)


# =============================================================================
# Test / Demo
# =============================================================================

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='MAVLink Guidance Test')
    parser.add_argument('--port', default='/dev/ttyTHS1', help='Serial port')
    parser.add_argument('--baud', type=int, default=57600, help='Baud rate')
    parser.add_argument('--udp', default=None, help='UDP address (e.g., udp:127.0.0.1:14550)')
    args = parser.parse_args()

    # Create guidance
    guidance = MavlinkGuidance()

    # Connect
    conn_str = args.udp if args.udp else args.port
    if not guidance.connect(conn_str, args.baud):
        print("Failed to connect")
        exit(1)

    print("\nGuidance connected. Commands:")
    print("  a = arm")
    print("  g = set GUIDED mode")
    print("  f = forward 0.5 m/s")
    print("  s = stop")
    print("  l = loiter mode")
    print("  r = RTL")
    print("  q = quit")

    try:
        while True:
            cmd = input("\n> ").strip().lower()

            if cmd == 'a':
                guidance.arm()
            elif cmd == 'g':
                guidance.set_guided_mode()
            elif cmd == 'f':
                print("Forward 0.5 m/s for 2 seconds...")
                for _ in range(40):
                    guidance.send_velocity(0.5, 0, 0, 0)
                    time.sleep(0.05)
                guidance.stop()
            elif cmd == 's':
                guidance.stop()
            elif cmd == 'l':
                guidance.set_loiter_mode()
            elif cmd == 'r':
                guidance.return_to_launch()
            elif cmd == 'q':
                break
            else:
                # Print telemetry
                pos = guidance.get_position()
                att = guidance.get_attitude()
                mode = guidance.get_mode_name()
                in_guided = guidance.is_guided_mode()
                print(f"Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
                print(f"Attitude: (r={att[0]:.2f}, p={att[1]:.2f}, y={att[2]:.2f})")
                print(f"Flight Mode: {mode} {'[JETSON CAN CONTROL]' if in_guided else '[PILOT CONTROL]'}")
                print(f"State: {guidance.get_state().value}")
                print(f"Armed: {guidance.armed}")

    except KeyboardInterrupt:
        pass

    finally:
        guidance.stop()
        guidance.disconnect()
        print("\nDisconnected")
