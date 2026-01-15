"""
Mission Manager Module

Manages mission state, stop conditions, and overall mission lifecycle.
Tracks progress and determines when to end a mission.

Features:
- Configurable stop conditions (poles, time, battery, coverage, distance)
- Mission state tracking (status, progress, timing)
- Event logging
- Mission summary generation

Usage:
    from mission_manager import MissionManager, StopConditions

    stop_conditions = StopConditions(
        max_poles=50,
        max_time_minutes=30,
        min_battery_percent=25.0
    )
    manager = MissionManager(stop_conditions)

    manager.start()

    # In main loop:
    should_stop, reason = manager.should_stop(
        battery_percent=current_battery,
        distance_from_home=dist
    )
    if should_stop:
        print(f"Mission ending: {reason}")
"""

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


# =============================================================================
# Enums and Data Classes
# =============================================================================

class MissionState(Enum):
    """Mission lifecycle states."""
    NOT_STARTED = "not_started"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    ABORTED = "aborted"


class StopReason(Enum):
    """Reasons for mission stop."""
    NONE = "none"
    MAX_POLES_REACHED = "max_poles_reached"
    TIME_LIMIT = "time_limit"
    LOW_BATTERY = "low_battery"
    COVERAGE_COMPLETE = "coverage_complete"
    MAX_DISTANCE = "max_distance"
    USER_REQUESTED = "user_requested"
    FAILSAFE = "failsafe"
    ERROR = "error"


@dataclass
class StopConditions:
    """Configurable stop conditions for a mission."""
    max_poles: int = 0              # 0 = unlimited
    max_time_minutes: float = 0     # 0 = unlimited
    min_battery_percent: float = 20.0
    coverage_threshold: float = 0.95  # 0-1 (for area search patterns)
    max_distance_from_home: float = 0  # 0 = unlimited, meters

    @classmethod
    def from_dict(cls, data: dict) -> 'StopConditions':
        """Create from dictionary (e.g., from AeroSync job)."""
        return cls(
            max_poles=data.get('max_poles', 0),
            max_time_minutes=data.get('max_time_minutes', 0),
            min_battery_percent=data.get('min_battery_percent', 20.0),
            coverage_threshold=data.get('coverage_threshold', 0.95),
            max_distance_from_home=data.get('max_distance_from_home', 0)
        )


@dataclass
class MissionStatus:
    """Current mission status for reporting."""
    state: MissionState
    poles_inspected: int
    elapsed_minutes: float
    battery_percent: float
    distance_from_home: float
    coverage_percent: float
    stop_reason: StopReason = StopReason.NONE


@dataclass
class MissionSummary:
    """Summary of completed mission."""
    job_id: str
    start_time: float
    end_time: float
    duration_minutes: float
    state: MissionState
    stop_reason: StopReason
    poles_inspected: int
    poles_skipped_duplicate: int
    poles_skipped_low_confidence: int
    final_battery_percent: float
    final_distance_from_home: float
    final_coverage_percent: float
    events: List[dict] = field(default_factory=list)

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON/API."""
        return {
            'job_id': self.job_id,
            'start_time': self.start_time,
            'end_time': self.end_time,
            'duration_minutes': self.duration_minutes,
            'state': self.state.value,
            'stop_reason': self.stop_reason.value,
            'poles_inspected': self.poles_inspected,
            'poles_skipped_duplicate': self.poles_skipped_duplicate,
            'poles_skipped_low_confidence': self.poles_skipped_low_confidence,
            'final_battery_percent': self.final_battery_percent,
            'final_distance_from_home': self.final_distance_from_home,
            'final_coverage_percent': self.final_coverage_percent,
            'events': self.events
        }


@dataclass
class MissionEvent:
    """Event logged during mission."""
    timestamp: float
    event_type: str
    data: dict

    def to_dict(self) -> dict:
        return {
            'timestamp': self.timestamp,
            'type': self.event_type,
            'data': self.data
        }


# =============================================================================
# Mission Manager
# =============================================================================

class MissionManager:
    """Manages mission lifecycle and stop conditions."""

    def __init__(self, stop_conditions: Optional[StopConditions] = None,
                 config_path: Optional[str] = None,
                 job_id: str = ""):
        """Initialize mission manager.

        Args:
            stop_conditions: StopConditions object
            config_path: Path to YAML config file
            job_id: Job identifier (from AeroSync)
        """
        if stop_conditions is not None:
            self.stop_conditions = stop_conditions
        elif config_path is not None:
            self.stop_conditions = self._load_config(config_path)
        else:
            self.stop_conditions = StopConditions()

        self.job_id = job_id

        # Mission state
        self._state = MissionState.NOT_STARTED
        self._stop_reason = StopReason.NONE
        self._start_time: Optional[float] = None
        self._end_time: Optional[float] = None
        self._pause_time: Optional[float] = None
        self._paused_duration: float = 0.0

        # Progress tracking
        self._poles_inspected: int = 0
        self._poles_skipped_duplicate: int = 0
        self._poles_skipped_low_confidence: int = 0
        self._coverage: float = 0.0

        # Current telemetry (updated externally)
        self._battery_percent: float = 100.0
        self._distance_from_home: float = 0.0

        # Event log
        self._events: List[MissionEvent] = []

    def _load_config(self, config_path: str) -> StopConditions:
        """Load stop conditions from YAML config."""
        conditions = StopConditions()

        if not YAML_AVAILABLE:
            return conditions

        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)

            if 'stop_conditions' in data:
                sc = data['stop_conditions']
                conditions.max_poles = sc.get('max_poles', conditions.max_poles)
                conditions.max_time_minutes = sc.get('max_time_minutes',
                                                      conditions.max_time_minutes)
                conditions.min_battery_percent = sc.get('min_battery_percent',
                                                         conditions.min_battery_percent)
                conditions.coverage_threshold = sc.get('coverage_threshold',
                                                        conditions.coverage_threshold)
                conditions.max_distance_from_home = sc.get('max_distance_from_home',
                                                            conditions.max_distance_from_home)

        except Exception as e:
            print(f"Warning: Could not load stop conditions: {e}")

        return conditions

    # =========================================================================
    # Mission Lifecycle
    # =========================================================================

    def start(self):
        """Start the mission."""
        if self._state != MissionState.NOT_STARTED:
            print(f"Warning: Cannot start mission in state {self._state.value}")
            return

        self._state = MissionState.RUNNING
        self._start_time = time.time()
        self._log_event('mission_started', {'job_id': self.job_id})
        print(f"Mission started: {self.job_id}")

    def pause(self):
        """Pause the mission."""
        if self._state != MissionState.RUNNING:
            return

        self._state = MissionState.PAUSED
        self._pause_time = time.time()
        self._log_event('mission_paused', {})

    def resume(self):
        """Resume a paused mission."""
        if self._state != MissionState.PAUSED:
            return

        if self._pause_time:
            self._paused_duration += time.time() - self._pause_time
        self._pause_time = None

        self._state = MissionState.RUNNING
        self._log_event('mission_resumed', {})

    def complete(self, reason: StopReason = StopReason.NONE):
        """Mark mission as completed.

        Args:
            reason: Why the mission completed
        """
        self._state = MissionState.COMPLETED
        self._stop_reason = reason
        self._end_time = time.time()
        self._log_event('mission_completed', {'reason': reason.value})
        print(f"Mission completed: {reason.value}")

    def abort(self, reason: StopReason = StopReason.FAILSAFE):
        """Abort the mission.

        Args:
            reason: Why the mission was aborted
        """
        self._state = MissionState.ABORTED
        self._stop_reason = reason
        self._end_time = time.time()
        self._log_event('mission_aborted', {'reason': reason.value})
        print(f"Mission aborted: {reason.value}")

    # =========================================================================
    # Stop Condition Checking
    # =========================================================================

    def should_stop(self,
                    battery_percent: Optional[float] = None,
                    distance_from_home: Optional[float] = None,
                    coverage: Optional[float] = None) -> Tuple[bool, StopReason]:
        """Check if mission should stop based on current conditions.

        Args:
            battery_percent: Current battery level (0-100)
            distance_from_home: Distance from home/launch point (m)
            coverage: Current coverage percentage (0-1)

        Returns:
            Tuple of (should_stop, reason)
        """
        if self._state != MissionState.RUNNING:
            return False, StopReason.NONE

        # Update telemetry
        if battery_percent is not None:
            self._battery_percent = battery_percent
        if distance_from_home is not None:
            self._distance_from_home = distance_from_home
        if coverage is not None:
            self._coverage = coverage

        sc = self.stop_conditions

        # Check max poles
        if sc.max_poles > 0 and self._poles_inspected >= sc.max_poles:
            return True, StopReason.MAX_POLES_REACHED

        # Check time limit
        if sc.max_time_minutes > 0:
            elapsed = self.get_elapsed_minutes()
            if elapsed >= sc.max_time_minutes:
                return True, StopReason.TIME_LIMIT

        # Check battery
        if self._battery_percent <= sc.min_battery_percent:
            return True, StopReason.LOW_BATTERY

        # Check coverage
        if self._coverage >= sc.coverage_threshold:
            return True, StopReason.COVERAGE_COMPLETE

        # Check distance
        if sc.max_distance_from_home > 0:
            if self._distance_from_home >= sc.max_distance_from_home:
                return True, StopReason.MAX_DISTANCE

        return False, StopReason.NONE

    def check_and_stop(self,
                       battery_percent: Optional[float] = None,
                       distance_from_home: Optional[float] = None,
                       coverage: Optional[float] = None) -> bool:
        """Check stop conditions and automatically complete/abort if met.

        Returns:
            True if mission was stopped
        """
        should_stop, reason = self.should_stop(
            battery_percent, distance_from_home, coverage
        )

        if should_stop:
            if reason == StopReason.LOW_BATTERY or reason == StopReason.FAILSAFE:
                self.abort(reason)
            else:
                self.complete(reason)
            return True

        return False

    # =========================================================================
    # Progress Tracking
    # =========================================================================

    def record_pole_inspected(self, pole_id: str, global_id: str):
        """Record a pole inspection.

        Args:
            pole_id: Local tracking ID
            global_id: Global UUID for AeroSync
        """
        self._poles_inspected += 1
        self._log_event('pole_inspected', {
            'pole_id': pole_id,
            'global_id': global_id,
            'total_inspected': self._poles_inspected
        })

    def record_pole_skipped(self, reason: str = "duplicate"):
        """Record a skipped pole.

        Args:
            reason: Why the pole was skipped
        """
        if reason == "duplicate":
            self._poles_skipped_duplicate += 1
        else:
            self._poles_skipped_low_confidence += 1

    def update_coverage(self, coverage: float):
        """Update coverage percentage.

        Args:
            coverage: Coverage percentage (0-1)
        """
        self._coverage = coverage

    def update_telemetry(self, battery_percent: float, distance_from_home: float):
        """Update telemetry values.

        Args:
            battery_percent: Current battery (0-100)
            distance_from_home: Distance from home (m)
        """
        self._battery_percent = battery_percent
        self._distance_from_home = distance_from_home

    # =========================================================================
    # Status and Reporting
    # =========================================================================

    def get_state(self) -> MissionState:
        """Get current mission state."""
        return self._state

    def is_running(self) -> bool:
        """Check if mission is actively running."""
        return self._state == MissionState.RUNNING

    def get_elapsed_minutes(self) -> float:
        """Get elapsed mission time in minutes (excluding paused time)."""
        if self._start_time is None:
            return 0.0

        end = self._end_time or time.time()
        total = end - self._start_time - self._paused_duration

        # Account for current pause
        if self._state == MissionState.PAUSED and self._pause_time:
            total -= (time.time() - self._pause_time)

        return total / 60.0

    def get_status(self) -> MissionStatus:
        """Get current mission status."""
        return MissionStatus(
            state=self._state,
            poles_inspected=self._poles_inspected,
            elapsed_minutes=self.get_elapsed_minutes(),
            battery_percent=self._battery_percent,
            distance_from_home=self._distance_from_home,
            coverage_percent=self._coverage,
            stop_reason=self._stop_reason
        )

    def get_summary(self) -> MissionSummary:
        """Get mission summary (call after mission ends)."""
        return MissionSummary(
            job_id=self.job_id,
            start_time=self._start_time or 0.0,
            end_time=self._end_time or time.time(),
            duration_minutes=self.get_elapsed_minutes(),
            state=self._state,
            stop_reason=self._stop_reason,
            poles_inspected=self._poles_inspected,
            poles_skipped_duplicate=self._poles_skipped_duplicate,
            poles_skipped_low_confidence=self._poles_skipped_low_confidence,
            final_battery_percent=self._battery_percent,
            final_distance_from_home=self._distance_from_home,
            final_coverage_percent=self._coverage,
            events=[e.to_dict() for e in self._events]
        )

    def get_progress_percent(self) -> float:
        """Get overall mission progress estimate (0-100).

        Progress is based on whichever metric is most limiting.
        """
        progress_values = []

        sc = self.stop_conditions

        # Progress by poles
        if sc.max_poles > 0:
            progress_values.append(
                (self._poles_inspected / sc.max_poles) * 100
            )

        # Progress by time
        if sc.max_time_minutes > 0:
            elapsed = self.get_elapsed_minutes()
            progress_values.append(
                (elapsed / sc.max_time_minutes) * 100
            )

        # Progress by coverage
        if sc.coverage_threshold > 0:
            progress_values.append(
                (self._coverage / sc.coverage_threshold) * 100
            )

        # Progress by battery (inverted - lower battery = more progress)
        battery_used = 100.0 - self._battery_percent
        battery_allowance = 100.0 - sc.min_battery_percent
        if battery_allowance > 0:
            progress_values.append(
                (battery_used / battery_allowance) * 100
            )

        if progress_values:
            return min(100.0, max(progress_values))
        return 0.0

    # =========================================================================
    # Event Logging
    # =========================================================================

    def _log_event(self, event_type: str, data: dict):
        """Log a mission event."""
        event = MissionEvent(
            timestamp=time.time(),
            event_type=event_type,
            data=data
        )
        self._events.append(event)

    def log_custom_event(self, event_type: str, data: dict):
        """Log a custom event (for external use)."""
        self._log_event(event_type, data)

    def get_events(self) -> List[dict]:
        """Get all logged events."""
        return [e.to_dict() for e in self._events]

    # =========================================================================
    # Reset
    # =========================================================================

    def reset(self):
        """Reset mission manager for a new mission."""
        self._state = MissionState.NOT_STARTED
        self._stop_reason = StopReason.NONE
        self._start_time = None
        self._end_time = None
        self._pause_time = None
        self._paused_duration = 0.0
        self._poles_inspected = 0
        self._poles_skipped_duplicate = 0
        self._poles_skipped_low_confidence = 0
        self._coverage = 0.0
        self._battery_percent = 100.0
        self._distance_from_home = 0.0
        self._events.clear()


# =============================================================================
# Convenience Functions
# =============================================================================

def create_mission_manager(config_path: str = 'config/settings.yaml',
                           job_id: str = "") -> MissionManager:
    """Create a MissionManager with config from file."""
    return MissionManager(config_path=config_path, job_id=job_id)


# =============================================================================
# Test
# =============================================================================

if __name__ == '__main__':
    print("Mission Manager Test")
    print("=" * 50)

    # Create stop conditions
    stop_conditions = StopConditions(
        max_poles=10,
        max_time_minutes=5,
        min_battery_percent=25.0
    )

    # Create manager
    manager = MissionManager(stop_conditions, job_id="test-job-001")

    # Start mission
    manager.start()
    print(f"State: {manager.get_state().value}")

    # Simulate progress
    for i in range(1, 12):
        # Simulate pole inspection
        manager.record_pole_inspected(str(i), f"uuid-{i}")
        print(f"Inspected pole {i}, total: {manager._poles_inspected}")

        # Check stop conditions
        should_stop, reason = manager.should_stop(
            battery_percent=100.0 - (i * 5),  # Battery drains
            distance_from_home=i * 10.0       # Moving away from home
        )

        if should_stop:
            print(f"Stop condition met: {reason.value}")
            manager.complete(reason)
            break

        time.sleep(0.1)

    # Print summary
    summary = manager.get_summary()
    print("\nMission Summary:")
    print(f"  State: {summary.state.value}")
    print(f"  Stop reason: {summary.stop_reason.value}")
    print(f"  Poles inspected: {summary.poles_inspected}")
    print(f"  Duration: {summary.duration_minutes:.2f} minutes")
    print(f"  Final battery: {summary.final_battery_percent:.1f}%")
