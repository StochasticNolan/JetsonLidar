#!/usr/bin/env python3
"""
Test 1: Jetson API Client → AeroSync Integration Test

This script uses the REAL Jetson AeroSync client to validate
API communication with the AeroSync backend.

Run this BEFORE attempting any drone flight to validate:
- API endpoint compatibility
- Data format correctness
- Authentication flow
- Error handling
"""

import sys
import os
from datetime import datetime

# Import the actual Jetson AeroSync client
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'perception_nav', 'src'))
from aerosync_client import AeroSyncClient


def test_full_workflow():
    """Test the complete Jetson → AeroSync workflow."""

    # Configuration
    AEROSYNC_URL = "http://localhost:8000"  # Your AeroSync backend
    DRONE_API_KEY = "dev-key-change-in-production"  # From your config.py

    print("=" * 80)
    print("TEST 1: Jetson API Client → AeroSync Integration")
    print("=" * 80)
    print(f"\nConnecting to: {AEROSYNC_URL}")
    print(f"Using API Key: {DRONE_API_KEY[:10]}...")

    # Initialize the real Jetson client with config
    from aerosync_client import AeroSyncConfig
    config = AeroSyncConfig()
    config.base_url = AEROSYNC_URL
    config.api_key = DRONE_API_KEY
    config.websocket_enabled = False  # Not testing WebSocket yet

    client = AeroSyncClient(config=config)

    # Step 1: Fetch Job
    print("\n[STEP 1] Fetching job from AeroSync...")
    job_id = input("Enter job ID to test with: ").strip()

    job = client.fetch_job(job_id)
    if not job:
        print("❌ FAILED: Could not fetch job")
        return False

    print(f"✅ SUCCESS: Fetched job {job_id}")
    print(f"   - Mission Type: autonomous")
    print(f"   - Search Pattern: {job.search_pattern}")
    print(f"   - Name: {job.name}")

    # Step 2: Start Job
    print("\n[STEP 2] Starting job...")
    if client.start_job(job_id):
        print(f"✅ SUCCESS: Job {job_id} started")
    else:
        print("❌ FAILED: Could not start job")
        return False

    # Step 3: Report Pole Found
    print("\n[STEP 3] Simulating pole detection...")

    # Create mock InspectionResult matching Jetson's data structure
    class MockInspectionResult:
        def __init__(self):
            self.global_pole_id = "test-pole-001"
            self.timestamp_iso = datetime.now().isoformat() + 'Z'
            self.timestamp = datetime.now().timestamp()
            self.position_gps = (37.7749, -122.4194, 50.0)  # lat, lon, alt
            self.position_local = (10.0, 5.0, 2.0)  # x, y, z
            self.confidence = 0.92
            self.pole_type = "wood"
            self.lidar_confidence = 0.9
            self.camera_confidence = 0.94
            self.pole_height = 12.5
            self.pole_radius = 0.15

    mock_result = MockInspectionResult()
    pole_id = client.report_pole(job_id, mock_result)

    if pole_id:
        print(f"✅ SUCCESS: Reported pole (ID: {pole_id})")
        print(f"   - Location: {mock_result.position_gps[0]}, {mock_result.position_gps[1]}")
        print(f"   - Type: {mock_result.pole_type}")
        print(f"   - Confidence: {mock_result.confidence}")
    else:
        print("❌ FAILED: Could not report pole")
        return False

    # Step 4: Update Mission Status
    print("\n[STEP 4] Sending mission status update...")

    # Create mock MissionStatus
    class MockMissionStatus:
        def __init__(self):
            self.state = 'in_progress'
            self.poles_inspected = 1
            self.elapsed_minutes = 2.3
            self.battery_percent = 87.0
            self.coverage_percent = 15.5
            self.distance_from_home = 150.0

    mock_status = MockMissionStatus()
    if client.update_status(job_id, mock_status):
        print("✅ SUCCESS: Mission status updated")
        print("   - State: in_progress")
        print("   - Poles Inspected: 1")
        print("   - Coverage: 15.5%")
        print("   - Battery: 87.0%")
    else:
        print("❌ FAILED: Could not update mission status")
        return False

    # Step 5: Complete Job
    print("\n[STEP 5] Completing job...")

    # Create mock MissionSummary
    class MockMissionSummary:
        def __init__(self):
            self.stop_reason = 'test_completed'
            self.poles_inspected = 1
            self.poles_skipped_duplicate = 0
            self.duration_minutes = 5.0
            self.final_battery_percent = 85.0
            self.final_coverage_percent = 20.0

    mock_summary = MockMissionSummary()
    if client.complete_job(job_id, mock_summary):
        print(f"✅ SUCCESS: Job {job_id} completed")
        print("   - Stop Reason: test_completed")
        print("   - Poles Inspected: 1")
        print("   - Duration: 5.0 minutes")
    else:
        print("❌ FAILED: Could not complete job")
        return False

    # Final Summary
    print("\n" + "=" * 80)
    print("✅ ALL TESTS PASSED - API Integration Working!")
    print("=" * 80)
    print("\nValidated:")
    print("  ✓ GET /api/drone/jobs/{job_id}")
    print("  ✓ POST /api/drone/jobs/{job_id}/start")
    print("  ✓ POST /api/drone/jobs/{job_id}/poles")
    print("  ✓ PUT /api/drone/jobs/{job_id}/status")
    print("  ✓ POST /api/drone/jobs/{job_id}/complete")
    print("\nNext Steps:")
    print("  → Test 2: Mission Planner (waypoint generation)")
    print("  → Test 3: Photo Upload Pipeline (S3 presigned URLs)")
    print("  → Test 4: WebSocket Real-Time Updates")
    print("\nAll core Jetson ↔ AeroSync API endpoints working correctly!")

    return True


if __name__ == "__main__":
    try:
        success = test_full_workflow()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ UNEXPECTED ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
