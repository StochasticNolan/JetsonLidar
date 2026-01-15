"""
Mission Simulation Tests

End-to-end tests for the pole inspection mission system.
Tests the full flow from job creation to pole reporting with fake data.

Features:
- Fake pole generation
- Fake photo generation
- State machine simulation
- AeroSync integration verification

Usage:
    # Run all tests
    python tests/test_mission_simulation.py

    # Run specific test
    python tests/test_mission_simulation.py --test line_follow
"""

import argparse
import json
import math
import os
import sys
import time
import uuid
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import modules being tested
from perception_nav.src.pole_inspector import PoleInspector, InspectionConfig, InspectionResult
from perception_nav.src.mission_manager import MissionManager, StopConditions, MissionState, StopReason
from perception_nav.src.aerosync_client import MockAeroSyncClient, Job
from tests.fake_aerosync_server import FakeAeroSyncServer


# =============================================================================
# Fake Data Generators
# =============================================================================

@dataclass
class FakeFusedPole:
    """Simulated FusedPole for testing."""
    id: int
    x: float                      # Forward (m)
    y: float                      # Left (m)
    z: float = 10.0               # Height (m)
    radius: float = 0.15
    range_xy: float = 0.0         # Computed from x, y
    fused_confidence: float = 0.8
    lidar_confidence: float = 0.9
    camera_confidence: float = 0.7
    camera_matched: bool = True
    camera_class: str = "h_frame"
    frames_seen: int = 15
    height: float = 12.0

    def __post_init__(self):
        if self.range_xy == 0.0:
            self.range_xy = math.sqrt(self.x**2 + self.y**2)


def generate_line_of_poles(num_poles: int = 10,
                           spacing: float = 75.0,
                           lateral_offset: float = 5.0,
                           noise: float = 2.0,
                           start_distance: float = 50.0) -> List[FakeFusedPole]:
    """Generate a line of poles for testing LINE_FOLLOW pattern.

    Args:
        num_poles: Number of poles to generate
        spacing: Distance between poles (m)
        lateral_offset: Offset from drone path (m)
        noise: Random position noise (m)
        start_distance: Distance to first pole (m)

    Returns:
        List of FakeFusedPole objects
    """
    import random

    poles = []
    for i in range(num_poles):
        x = start_distance + (i * spacing) + random.uniform(-noise, noise)
        y = -lateral_offset + random.uniform(-noise/2, noise/2)  # Right of drone
        z = random.uniform(8, 15)

        pole = FakeFusedPole(
            id=i + 1,
            x=x,
            y=y,
            z=z,
            radius=random.uniform(0.1, 0.2),
            fused_confidence=random.uniform(0.7, 0.95),
            camera_matched=random.random() > 0.1,  # 90% have camera match
            camera_class=random.choice(['h_frame', 'single_pole', 'wood_distribution']),
            frames_seen=random.randint(10, 30),
            height=random.uniform(10, 18)
        )
        poles.append(pole)

    return poles


def generate_fake_photo(pole_id: str, pole_type: str, output_dir: str) -> str:
    """Generate a fake pole photo (placeholder image).

    In real use, this would be an actual photo. For testing, we create
    a simple text file or placeholder.

    Args:
        pole_id: Pole UUID
        pole_type: Type of pole
        output_dir: Directory to save image

    Returns:
        Path to generated image
    """
    os.makedirs(output_dir, exist_ok=True)

    # Create a simple placeholder file
    filename = f"pole_{pole_id[:8]}_{int(time.time())}.txt"
    filepath = os.path.join(output_dir, filename)

    with open(filepath, 'w') as f:
        f.write(f"Fake photo for pole {pole_id}\n")
        f.write(f"Type: {pole_type}\n")
        f.write(f"Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")

    return filepath


# =============================================================================
# Mission Simulator
# =============================================================================

class MissionSimulator:
    """Simulates a full mission for testing."""

    def __init__(self, job: Job):
        """Initialize simulator.

        Args:
            job: Job configuration from AeroSync
        """
        self.job = job

        # Initialize components
        self.inspector = PoleInspector(InspectionConfig(
            min_fused_confidence=job.inspection_config.get('min_confidence', 0.6),
            camera_match_required=job.inspection_config.get('camera_required', True),
            capture_distance=job.inspection_config.get('capture_distance', 10.0)
        ))

        self.mission = MissionManager(
            StopConditions.from_dict(job.stop_conditions),
            job_id=job.job_id
        )

        self.aerosync = MockAeroSyncClient()
        self.aerosync.add_mock_job(job)

        # Simulation state
        self.drone_position = (0.0, 0.0, 30.0)  # Start position
        self.drone_heading = 0.0  # Degrees
        self.battery = 100.0
        self.speed = 3.0  # m/s

        # Results
        self.inspected_poles: List[InspectionResult] = []
        self.simulation_log: List[dict] = []

    def simulate_approach(self, pole: FakeFusedPole, steps: int = 10) -> List[FakeFusedPole]:
        """Simulate drone approaching a pole.

        Returns pole observations at decreasing distances.
        """
        observations = []

        start_distance = pole.range_xy
        end_distance = 8.0  # Capture zone

        for i in range(steps):
            # Interpolate distance
            t = i / (steps - 1)
            current_distance = start_distance + (end_distance - start_distance) * t

            # Create observation at this distance
            obs = FakeFusedPole(
                id=pole.id,
                x=current_distance,  # Simplified: pole directly ahead
                y=pole.y,
                z=pole.z,
                radius=pole.radius,
                fused_confidence=pole.fused_confidence,
                camera_matched=pole.camera_matched,
                camera_class=pole.camera_class,
                frames_seen=pole.frames_seen + i,
                height=pole.height
            )
            observations.append(obs)

            # Update drone position
            self.drone_position = (
                self.drone_position[0] + self.speed * 0.5,
                self.drone_position[1],
                self.drone_position[2]
            )

        return observations

    def run_simulation(self, poles: List[FakeFusedPole],
                       output_dir: str = '/tmp/mission_sim') -> dict:
        """Run full mission simulation.

        Args:
            poles: List of poles to discover
            output_dir: Directory for fake photos

        Returns:
            Simulation results dict
        """
        os.makedirs(output_dir, exist_ok=True)

        # Start mission
        self.aerosync.start_job(self.job.job_id)
        self.mission.start()

        self._log("Mission started", {'job_id': self.job.job_id})

        # Process each pole
        for pole in poles:
            # Check stop conditions
            should_stop, reason = self.mission.should_stop(
                battery_percent=self.battery,
                distance_from_home=self.drone_position[0]
            )

            if should_stop:
                self._log("Stop condition met", {'reason': reason.value})
                break

            # Check if should inspect
            if not self.inspector.should_inspect(pole):
                self._log("Pole skipped", {
                    'pole_id': pole.id,
                    'reason': 'criteria not met'
                })
                continue

            # Simulate approach
            self._log("Approaching pole", {'pole_id': pole.id, 'distance': pole.range_xy})
            observations = self.simulate_approach(pole)

            # Check capture zone
            final_obs = observations[-1]
            if self.inspector.is_in_capture_zone(final_obs):
                # Declare found
                photo_path = generate_fake_photo(
                    str(uuid.uuid4()),
                    pole.camera_class,
                    output_dir
                )

                result = self.inspector.declare_found(
                    final_obs,
                    drone_heading=self.drone_heading,
                    photo_paths=[photo_path]
                )

                self.inspected_poles.append(result)
                self.mission.record_pole_inspected(str(pole.id), result.global_pole_id)

                # Report to AeroSync
                self.aerosync.report_pole(self.job.job_id, result)
                if photo_path:
                    self.aerosync.upload_photo(
                        self.job.job_id,
                        result.global_pole_id,
                        photo_path
                    )

                self._log("Pole inspected", {
                    'pole_id': pole.id,
                    'global_id': result.global_pole_id,
                    'position_gps': result.position_gps
                })

            # Simulate battery drain and movement
            self.battery -= 0.5  # 0.5% per pole

        # Complete mission
        self.mission.complete(StopReason.MAX_POLES_REACHED)
        summary = self.mission.get_summary()
        self.aerosync.complete_job(self.job.job_id, summary)

        self._log("Mission completed", {
            'poles_inspected': summary.poles_inspected,
            'duration_minutes': summary.duration_minutes
        })

        # Return results
        return {
            'success': True,
            'poles_inspected': len(self.inspected_poles),
            'poles_reported_to_aerosync': len(self.aerosync.get_reported_poles()),
            'photos_uploaded': len(self.aerosync.get_uploaded_photos()),
            'final_battery': self.battery,
            'duration_minutes': summary.duration_minutes,
            'log': self.simulation_log
        }

    def _log(self, message: str, data: dict = None):
        """Log simulation event."""
        entry = {
            'timestamp': time.time(),
            'message': message,
            'data': data or {}
        }
        self.simulation_log.append(entry)
        print(f"[SIM] {message}: {data}")


# =============================================================================
# Test Cases
# =============================================================================

def test_line_follow_mission():
    """Test LINE_FOLLOW pattern with 10 poles."""
    print("\n" + "=" * 60)
    print("TEST: Line Follow Mission (10 poles)")
    print("=" * 60)

    # Create job
    job = Job(
        job_id="test-line-001",
        name="Line Follow Test",
        search_pattern="line_follow",
        search_config={},
        stop_conditions={
            'max_poles': 10,
            'max_time_minutes': 30,
            'min_battery_percent': 20.0
        },
        inspection_config={
            'min_confidence': 0.6,
            'camera_required': True,
            'capture_distance': 15.0
        },
        previously_inspected=[]
    )

    # Generate poles
    poles = generate_line_of_poles(num_poles=10, spacing=75.0)
    print(f"Generated {len(poles)} poles")

    # Run simulation
    sim = MissionSimulator(job)
    results = sim.run_simulation(poles)

    # Verify
    assert results['success'], "Simulation should succeed"
    assert results['poles_inspected'] > 0, "Should inspect at least one pole"
    assert results['poles_reported_to_aerosync'] == results['poles_inspected'], \
        "All inspected poles should be reported to AeroSync"

    print(f"\nResults: {results['poles_inspected']} poles inspected")
    print("TEST PASSED")
    return True


def test_deduplication():
    """Test that duplicate poles are not re-inspected."""
    print("\n" + "=" * 60)
    print("TEST: Deduplication")
    print("=" * 60)

    # Create inspector
    config = InspectionConfig(
        min_fused_confidence=0.5,
        camera_match_required=False,
        dedup_distance_threshold=10.0
    )
    inspector = PoleInspector(config)

    # Create two poles at nearly the same location
    pole1 = FakeFusedPole(id=1, x=50.0, y=5.0, range_xy=50.2)
    pole2 = FakeFusedPole(id=2, x=52.0, y=6.0, range_xy=52.3)  # Within 10m of pole1

    # Inspect first pole
    assert inspector.should_inspect(pole1), "First pole should be inspectable"
    result1 = inspector.declare_found(pole1)
    print(f"Inspected pole 1: {result1.global_pole_id[:8]}...")

    # Second pole should be flagged as duplicate
    should_inspect_2 = inspector.should_inspect(pole2)
    print(f"Should inspect pole 2 (nearby): {should_inspect_2}")
    assert not should_inspect_2, "Second pole should be flagged as duplicate"

    # Third pole far away should be inspectable
    pole3 = FakeFusedPole(id=3, x=150.0, y=5.0, range_xy=150.1)
    assert inspector.should_inspect(pole3), "Distant pole should be inspectable"

    print("TEST PASSED")
    return True


def test_stop_conditions():
    """Test various stop conditions."""
    print("\n" + "=" * 60)
    print("TEST: Stop Conditions")
    print("=" * 60)

    # Test max poles
    print("\n1. Testing max_poles stop condition...")
    conditions = StopConditions(max_poles=3)
    manager = MissionManager(conditions, job_id="test-stop-001")
    manager.start()

    for i in range(5):
        manager.record_pole_inspected(str(i), f"uuid-{i}")
        should_stop, reason = manager.should_stop()
        print(f"  After pole {i+1}: should_stop={should_stop}, reason={reason.value}")
        if should_stop:
            break

    assert manager._poles_inspected == 3, "Should stop at 3 poles"
    assert reason == StopReason.MAX_POLES_REACHED

    # Test battery
    print("\n2. Testing battery stop condition...")
    conditions = StopConditions(min_battery_percent=25.0)
    manager = MissionManager(conditions, job_id="test-stop-002")
    manager.start()

    should_stop, reason = manager.should_stop(battery_percent=30.0)
    assert not should_stop, "Should not stop at 30% battery"

    should_stop, reason = manager.should_stop(battery_percent=20.0)
    assert should_stop, "Should stop at 20% battery"
    assert reason == StopReason.LOW_BATTERY

    print("TEST PASSED")
    return True


def test_aerosync_integration():
    """Test AeroSync API integration with mock server."""
    print("\n" + "=" * 60)
    print("TEST: AeroSync Integration")
    print("=" * 60)

    # Start fake server
    server = FakeAeroSyncServer(port=8181)
    server.start()

    try:
        # Create job
        job = server.create_job(
            job_id="test-aerosync-001",
            name="AeroSync Integration Test",
            stop_conditions={'max_poles': 5}
        )
        print(f"Created job: {job.job_id}")

        # Use mock client
        mock = MockAeroSyncClient()
        mock.add_mock_job(Job.from_dict(job.to_dict()))

        # Simulate pole reports
        for i in range(3):
            @dataclass
            class MockResult:
                global_pole_id: str = f"pole-{uuid.uuid4()}"
                timestamp_iso: str = time.strftime('%Y-%m-%dT%H:%M:%SZ')
                timestamp: float = time.time()
                position_gps: tuple = (41.0 + i * 0.001, -105.0 + i * 0.001, 2000.0)
                position_local: tuple = (50.0 + i * 75, 5.0, 10.0)
                confidence: float = 0.85
                pole_type: str = "h_frame"
                lidar_confidence: float = 0.9
                camera_confidence: float = 0.8
                pole_height: float = 15.0
                pole_radius: float = 0.15
                metadata: dict = field(default_factory=dict)

            result = MockResult()
            pole_id = mock.report_pole(job.job_id, result)
            print(f"Reported pole: {pole_id[:8]}...")

        # Verify
        reported = mock.get_reported_poles()
        assert len(reported) == 3, f"Should have 3 reported poles, got {len(reported)}"

        print(f"\nTotal poles reported: {len(reported)}")
        print("TEST PASSED")
        return True

    finally:
        server.stop()


def test_persistence():
    """Test save/load of inspected poles."""
    print("\n" + "=" * 60)
    print("TEST: Persistence")
    print("=" * 60)

    import tempfile

    # Create inspector and inspect some poles
    inspector = PoleInspector(InspectionConfig(
        min_fused_confidence=0.5,
        camera_match_required=False
    ))

    poles = generate_line_of_poles(num_poles=5, spacing=100.0)
    for pole in poles:
        if inspector.should_inspect(pole):
            inspector.declare_found(pole)

    original_count = inspector.get_inspected_count()
    print(f"Inspected {original_count} poles")

    # Save to file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        temp_path = f.name

    inspector.save_inspected_poles(temp_path)
    print(f"Saved to {temp_path}")

    # Load into new inspector
    inspector2 = PoleInspector(InspectionConfig())
    loaded_count = inspector2.load_inspected_poles(temp_path)
    print(f"Loaded {loaded_count} poles")

    assert loaded_count == original_count, "Should load same number of poles"

    # Verify deduplication works after load
    test_pole = FakeFusedPole(id=99, x=poles[0].x, y=poles[0].y, range_xy=poles[0].range_xy)
    is_dup = inspector2.is_already_inspected(test_pole)
    assert is_dup, "Should detect duplicate after reload"

    # Cleanup
    os.remove(temp_path)

    print("TEST PASSED")
    return True


# =============================================================================
# Main
# =============================================================================

def run_all_tests():
    """Run all tests."""
    tests = [
        ("Line Follow Mission", test_line_follow_mission),
        ("Deduplication", test_deduplication),
        ("Stop Conditions", test_stop_conditions),
        ("AeroSync Integration", test_aerosync_integration),
        ("Persistence", test_persistence),
    ]

    results = []
    for name, test_fn in tests:
        try:
            success = test_fn()
            results.append((name, success, None))
        except Exception as e:
            results.append((name, False, str(e)))
            print(f"TEST FAILED: {e}")

    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)

    passed = sum(1 for _, success, _ in results if success)
    total = len(results)

    for name, success, error in results:
        status = "PASSED" if success else f"FAILED: {error}"
        print(f"  {name}: {status}")

    print(f"\n{passed}/{total} tests passed")

    return passed == total


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Mission Simulation Tests')
    parser.add_argument('--test', choices=[
        'line_follow', 'dedup', 'stop', 'aerosync', 'persistence', 'all'
    ], default='all', help='Which test to run')
    args = parser.parse_args()

    if args.test == 'all':
        success = run_all_tests()
    elif args.test == 'line_follow':
        success = test_line_follow_mission()
    elif args.test == 'dedup':
        success = test_deduplication()
    elif args.test == 'stop':
        success = test_stop_conditions()
    elif args.test == 'aerosync':
        success = test_aerosync_integration()
    elif args.test == 'persistence':
        success = test_persistence()

    sys.exit(0 if success else 1)
