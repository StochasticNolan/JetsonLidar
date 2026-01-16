"""
Mission Executor Module

Orchestrates the complete autonomous mission flow:
1. Fetch and parse mission from AeroSync
2. Validate mission parameters
3. Takeoff to mission altitude
4. Navigate to search area (closest corner)
5. Execute search pattern (lawnmower/line-follow)
6. Detect and report poles
7. Capture and upload photos
8. Monitor stop conditions
9. Return to launch
10. Land

Usage:
    from mission_executor import MissionExecutor

    executor = MissionExecutor(config_path='config/settings.yaml')
    await executor.run_mission(job_id='uuid-string')
"""

import asyncio
import time
import threading
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Callable, List, Dict, Any

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

# Local imports
try:
    from .mission_planner import (
        parse_mission, generate_waypoints, validate_mission,
        compute_entry_point, Mission, Waypoint, SearchPattern
    )
    from .mission_manager import MissionManager, StopConditions, StopReason
    from .aerosync_client import AeroSyncClient, Job
    from .guidance import MavlinkGuidance, GuidanceState, GuidanceConfig
except ImportError:
    # Fallback for direct execution
    from mission_planner import (
        parse_mission, generate_waypoints, validate_mission,
        compute_entry_point, Mission, Waypoint, SearchPattern
    )
    from mission_manager import MissionManager, StopConditions, StopReason
    from aerosync_client import AeroSyncClient, Job
    from guidance import MavlinkGuidance, GuidanceState, GuidanceConfig


# =============================================================================
# Enums
# =============================================================================

class ExecutionState(Enum):
    """Mission execution states."""
    IDLE = "idle"
    FETCHING_MISSION = "fetching_mission"
    VALIDATING = "validating"
    TAKEOFF = "takeoff"
    TRANSIT_TO_AREA = "transit_to_area"
    SEARCHING = "searching"
    LINE_FOLLOWING = "line_following"
    INSPECTING = "inspecting"
    RETURNING = "returning"
    LANDING = "landing"
    COMPLETED = "completed"
    ABORTED = "aborted"
    ERROR = "error"


# =============================================================================
# Configuration
# =============================================================================

@dataclass
class ExecutorConfig:
    """Configuration for mission executor."""
    # Flight parameters
    takeoff_altitude: float = 30.0      # Default takeoff altitude (m)
    transit_speed: float = 5.0          # Speed to search area (m/s)
    search_speed: float = 3.0           # Speed during search (m/s)
    arrival_threshold: float = 5.0      # Waypoint arrival threshold (m)

    # Search behavior
    search_pattern: str = "lawnmower"   # Default: lawnmower, then line-follow
    switch_to_line_follow: bool = True  # Switch to line-follow when poles found
    min_poles_for_line: int = 2         # Min poles to switch to line-follow

    # Inspection
    capture_enabled: bool = True        # Enable photo capture
    capture_distance: float = 10.0      # Distance to capture (m)
    dwell_time: float = 2.0             # Hover time for capture (s)

    # Timing
    waypoint_timeout: float = 300.0     # Max time per waypoint (s)
    mission_timeout: float = 3600.0     # Max total mission time (s)

    # Status updates
    position_update_rate: float = 1.0   # Position broadcast rate (Hz)
    status_update_rate: float = 0.2     # Status update rate (Hz)


# =============================================================================
# Mission Executor
# =============================================================================

class MissionExecutor:
    """Executes complete autonomous missions."""

    def __init__(self, config: Optional[ExecutorConfig] = None,
                 config_path: Optional[str] = None):
        """Initialize mission executor.

        Args:
            config: ExecutorConfig object
            config_path: Path to YAML config file
        """
        self.config = config or ExecutorConfig()
        self.config_path = config_path or 'config/settings.yaml'

        # Components (initialized later)
        self.guidance: Optional[MavlinkGuidance] = None
        self.aerosync: Optional[AeroSyncClient] = None
        self.mission_manager: Optional[MissionManager] = None

        # Current mission
        self.mission: Optional[Mission] = None
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_idx: int = 0

        # State
        self.state = ExecutionState.IDLE
        self._running = False
        self._abort_requested = False

        # Callbacks
        self._on_state_change: Optional[Callable[[ExecutionState], None]] = None
        self._on_pole_found: Optional[Callable[[dict], None]] = None

        # Statistics
        self._poles_found: List[dict] = []
        self._photos_taken: int = 0
        self._distance_flown: float = 0.0

    # =========================================================================
    # Initialization
    # =========================================================================

    def initialize(self) -> bool:
        """Initialize all components.

        Returns:
            True if initialization successful
        """
        print("Initializing mission executor...")

        try:
            # Initialize guidance (FCU connection)
            self.guidance = MavlinkGuidance(config_path=self.config_path)

            # Initialize AeroSync client
            self.aerosync = AeroSyncClient(config_path=self.config_path)

            print("Mission executor initialized")
            return True

        except Exception as e:
            print(f"Initialization failed: {e}")
            return False

    def connect_fcu(self, port: str = '/dev/ttyACM0', baud: int = 57600) -> bool:
        """Connect to flight controller.

        Args:
            port: Serial port or UDP address
            baud: Baud rate for serial

        Returns:
            True if connected
        """
        if not self.guidance:
            print("Guidance not initialized")
            return False

        return self.guidance.connect(port, baud)

    # =========================================================================
    # Mission Execution
    # =========================================================================

    async def run_mission(self, job_id: str) -> bool:
        """Execute a complete mission.

        This is the main entry point for autonomous operation.

        Args:
            job_id: AeroSync job UUID

        Returns:
            True if mission completed successfully
        """
        self._running = True
        self._abort_requested = False
        mission_start = time.time()

        try:
            # 1. Fetch mission from AeroSync
            self._set_state(ExecutionState.FETCHING_MISSION)
            print(f"Fetching mission {job_id}...")

            job = self.aerosync.fetch_job(job_id)
            if not job:
                print("Failed to fetch mission")
                self._set_state(ExecutionState.ERROR)
                return False

            # Convert Job to dict format for mission planner
            job_data = job.to_mission_dict()

            # 2. Parse and validate mission
            self._set_state(ExecutionState.VALIDATING)
            print("Parsing mission...")

            try:
                self.mission = parse_mission(job_data)
            except Exception as e:
                print(f"Failed to parse mission: {e}")
                self._set_state(ExecutionState.ERROR)
                return False

            valid, error = validate_mission(self.mission)
            if not valid:
                print(f"Mission validation failed: {error}")
                self._set_state(ExecutionState.ERROR)
                return False

            print(f"Mission validated: {self.mission.name}")

            # Initialize mission manager
            self.mission_manager = MissionManager(
                self.mission.stop_conditions,
                job_id=job_id
            )

            # 3. Notify AeroSync of mission start
            self.aerosync.start_job(job_id)

            # 4. Connect WebSocket for real-time updates
            self.aerosync.connect_websocket(job_id)

            # Set abort callback
            self.aerosync.set_on_abort(self._handle_abort_command)
            self.aerosync.set_on_pause(self._handle_pause_command)
            self.aerosync.set_on_resume(self._handle_resume_command)

            # 5. Start mission manager
            self.mission_manager.start()

            # 6. Takeoff
            self._set_state(ExecutionState.TAKEOFF)
            altitude = self.mission.search_config.flight_altitude

            if not self.guidance.takeoff(altitude):
                print("Takeoff failed")
                await self._abort_mission("takeoff_failed")
                return False

            # 7. Navigate to search area entry point
            self._set_state(ExecutionState.TRANSIT_TO_AREA)
            entry_point = compute_entry_point(self.mission)

            print(f"Navigating to search area entry: ({entry_point.lat:.6f}, {entry_point.lon:.6f})")

            if not self.guidance.navigate_to_waypoint(
                entry_point.lat, entry_point.lon, entry_point.alt,
                speed=self.config.transit_speed
            ):
                print("Failed to reach search area")
                await self._abort_mission("transit_failed")
                return False

            # 8. Generate search waypoints
            self.waypoints = generate_waypoints(self.mission)
            self.current_waypoint_idx = 0
            print(f"Generated {len(self.waypoints)} search waypoints")

            # 9. Execute search pattern
            self._set_state(ExecutionState.SEARCHING)
            await self._execute_search_pattern()

            # 10. Check why we stopped
            if self._abort_requested:
                await self._abort_mission("user_requested")
                return False

            # 11. Return to launch
            self._set_state(ExecutionState.RETURNING)
            print("Returning to launch...")
            self.guidance.return_to_launch()

            # Wait for RTL to complete
            await self._wait_for_landing()

            # 12. Complete mission
            self._set_state(ExecutionState.COMPLETED)

            # Get summary and report to AeroSync
            summary = self.mission_manager.get_summary()
            self.aerosync.complete_job(job_id, summary)

            # Disconnect WebSocket
            self.aerosync.disconnect_websocket()

            elapsed = time.time() - mission_start
            print(f"Mission completed in {elapsed/60:.1f} minutes")
            print(f"  Poles found: {len(self._poles_found)}")
            print(f"  Photos taken: {self._photos_taken}")

            return True

        except Exception as e:
            print(f"Mission error: {e}")
            self._set_state(ExecutionState.ERROR)
            await self._abort_mission(f"error: {e}")
            return False

        finally:
            self._running = False

    async def _execute_search_pattern(self):
        """Execute the search pattern (lawnmower or line-follow)."""
        poles_detected = []
        switched_to_line_follow = False

        while self._running and not self._abort_requested:
            # Check stop conditions
            should_stop, reason = self.mission_manager.should_stop(
                battery_percent=self.guidance.get_battery_percent(),
                distance_from_home=self._get_distance_from_home()
            )

            if should_stop:
                print(f"Stop condition met: {reason.value}")
                break

            # Check if we should switch to line-follow mode
            if self.config.switch_to_line_follow and not switched_to_line_follow:
                if len(poles_detected) >= self.config.min_poles_for_line:
                    print(f"Found {len(poles_detected)} poles - switching to line-follow")
                    self._set_state(ExecutionState.LINE_FOLLOWING)
                    switched_to_line_follow = True
                    # Line-follow will use the smooth controller and line follower
                    # Continue flying along detected pole line

            # Navigate to next waypoint if in search mode
            if not switched_to_line_follow:
                if self.current_waypoint_idx >= len(self.waypoints):
                    print("Search pattern complete")
                    break

                wp = self.waypoints[self.current_waypoint_idx]

                # Navigate to waypoint
                arrived = self.guidance.navigate_to_waypoint(
                    wp.lat, wp.lon, wp.alt,
                    speed=self.config.search_speed,
                    timeout=self.config.waypoint_timeout
                )

                if arrived:
                    self.current_waypoint_idx += 1
                else:
                    # Timeout or abort
                    if self._abort_requested:
                        break
                    print(f"Waypoint {self.current_waypoint_idx} timeout, continuing...")
                    self.current_waypoint_idx += 1

            else:
                # In line-follow mode, let the smooth controller handle navigation
                # This would integrate with the existing line_follower and smooth_controller
                await asyncio.sleep(0.1)

            # Send position update to AeroSync
            lat, lon = self.guidance.get_gps_position()
            alt = self.guidance.get_altitude()
            self.aerosync.send_position(lat, lon, alt, 0, 0)

            await asyncio.sleep(0.1)

    async def _wait_for_landing(self, timeout: float = 120.0):
        """Wait for the vehicle to land.

        Args:
            timeout: Maximum wait time (seconds)
        """
        print("Waiting for landing...")
        start = time.time()

        while time.time() - start < timeout:
            if not self.guidance.armed:
                print("Landed and disarmed")
                self._set_state(ExecutionState.LANDING)
                return

            alt = self.guidance.get_altitude()
            print(f"  RTL altitude: {alt:.1f}m")
            await asyncio.sleep(2.0)

        print("Landing timeout")

    async def _abort_mission(self, reason: str):
        """Abort the mission and return to launch.

        Args:
            reason: Abort reason string
        """
        print(f"Aborting mission: {reason}")
        self._set_state(ExecutionState.ABORTED)

        # Command RTL
        self.guidance.return_to_launch()

        # Report to AeroSync
        if self.mission_manager:
            self.mission_manager.abort(StopReason.FAILSAFE)
            summary = self.mission_manager.get_summary()
            summary.stop_reason = reason
            if self.aerosync and self.mission:
                self.aerosync.complete_job(self.mission.job_id, summary)

        # Wait for landing
        await self._wait_for_landing()

    def _get_distance_from_home(self) -> float:
        """Calculate distance from home position."""
        if not self.mission:
            return 0.0

        lat, lon = self.guidance.get_gps_position()
        home_lat, home_lon, _ = self.mission.home_point

        # Simple approximation
        import math
        dlat = (lat - home_lat) * 111320
        dlon = (lon - home_lon) * 111320 * math.cos(math.radians(lat))
        return math.sqrt(dlat**2 + dlon**2)

    # =========================================================================
    # State Management
    # =========================================================================

    def _set_state(self, state: ExecutionState):
        """Update execution state and notify callbacks."""
        old_state = self.state
        self.state = state
        print(f"State: {old_state.value} -> {state.value}")

        if self._on_state_change:
            self._on_state_change(state)

    def get_state(self) -> ExecutionState:
        """Get current execution state."""
        return self.state

    def is_running(self) -> bool:
        """Check if mission is currently running."""
        return self._running

    # =========================================================================
    # Command Handlers
    # =========================================================================

    def _handle_abort_command(self):
        """Handle abort command from AeroSync."""
        print("Abort command received from AeroSync")
        self._abort_requested = True

    def _handle_pause_command(self):
        """Handle pause command from AeroSync."""
        print("Pause command received")
        if self.mission_manager:
            self.mission_manager.pause()
        self.guidance.hold_position()

    def _handle_resume_command(self):
        """Handle resume command from AeroSync."""
        print("Resume command received")
        if self.mission_manager:
            self.mission_manager.resume()

    def request_abort(self):
        """Request mission abort (from local control)."""
        self._abort_requested = True

    # =========================================================================
    # Callbacks
    # =========================================================================

    def set_on_state_change(self, callback: Callable[[ExecutionState], None]):
        """Set callback for state changes."""
        self._on_state_change = callback

    def set_on_pole_found(self, callback: Callable[[dict], None]):
        """Set callback for pole detection."""
        self._on_pole_found = callback

    # =========================================================================
    # Pole Handling
    # =========================================================================

    def report_pole(self, pole_data: dict):
        """Report a detected pole.

        Called by perception system when a pole is confirmed.

        Args:
            pole_data: Pole detection data
        """
        self._poles_found.append(pole_data)

        if self.mission_manager:
            self.mission_manager.record_pole_inspected(
                str(pole_data.get('id', '')),
                pole_data.get('global_id', '')
            )

        if self._on_pole_found:
            self._on_pole_found(pole_data)

        # Report to AeroSync (would need InspectionResult conversion)
        # self.aerosync.report_pole(self.mission.job_id, result)

    # =========================================================================
    # Status
    # =========================================================================

    def get_status(self) -> dict:
        """Get current mission status."""
        return {
            'state': self.state.value,
            'running': self._running,
            'waypoint': f"{self.current_waypoint_idx}/{len(self.waypoints)}",
            'poles_found': len(self._poles_found),
            'photos_taken': self._photos_taken,
            'battery': self.guidance.get_battery_percent() if self.guidance else -1,
            'altitude': self.guidance.get_altitude() if self.guidance else 0,
            'position': self.guidance.get_gps_position() if self.guidance else (0, 0)
        }


# =============================================================================
# Convenience Functions
# =============================================================================

def create_executor(config_path: str = 'config/settings.yaml') -> MissionExecutor:
    """Create and initialize a mission executor."""
    executor = MissionExecutor(config_path=config_path)
    executor.initialize()
    return executor


# =============================================================================
# Test
# =============================================================================

if __name__ == '__main__':
    print("Mission Executor Test")
    print("=" * 60)

    # Create executor
    executor = MissionExecutor()

    # Test state management
    print("\n1. State management test...")
    executor._set_state(ExecutionState.FETCHING_MISSION)
    executor._set_state(ExecutionState.VALIDATING)
    executor._set_state(ExecutionState.TAKEOFF)
    print(f"   Current state: {executor.get_state().value}")

    # Test status
    print("\n2. Status test...")
    status = executor.get_status()
    print(f"   Status: {status}")

    # Test with mock mission
    print("\n3. Mock mission parsing test...")
    mock_job_data = {
        'job_id': 'test-001',
        'name': 'Test Mission',
        'search_area': {
            'type': 'Polygon',
            'coordinates': [[
                [-105.5, 41.0], [-105.4, 41.0],
                [-105.4, 41.1], [-105.5, 41.1],
                [-105.5, 41.0]
            ]]
        },
        'search_pattern': 'lawnmower',
        'search_config': {
            'flight_altitude': 30,
            'speed': 3,
            'sweep_spacing': 100
        },
        'stop_conditions': {
            'max_poles': 50,
            'max_time_minutes': 30
        }
    }

    mission = parse_mission(mock_job_data)
    print(f"   Parsed mission: {mission.name}")

    valid, error = validate_mission(mission)
    print(f"   Valid: {valid}")
    if not valid:
        print(f"   Error: {error}")

    waypoints = generate_waypoints(mission)
    print(f"   Waypoints: {len(waypoints)}")

    entry = compute_entry_point(mission)
    print(f"   Entry point: ({entry.lat:.4f}, {entry.lon:.4f})")

    print("\n" + "=" * 60)
    print("Tests complete!")
    print("\nTo run a real mission:")
    print("  executor = create_executor()")
    print("  executor.connect_fcu('/dev/ttyACM0')")
    print("  await executor.run_mission('job-uuid')")
