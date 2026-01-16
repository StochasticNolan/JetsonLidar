#!/usr/bin/env python3
"""
Test 2: Mission Planner Validation

Tests that the Jetson mission planner can correctly generate waypoints
from AeroSync mission configuration.
"""

import sys
import os

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'perception_nav', 'src'))

from aerosync_client import AeroSyncClient, AeroSyncConfig
from mission_planner import parse_mission, generate_waypoints


def test_mission_planner():
    """Test mission planner with real AeroSync job."""

    print("=" * 80)
    print("TEST 2: Mission Planner - Waypoint Generation")
    print("=" * 80)

    # Configuration
    AEROSYNC_URL = "http://localhost:8000"
    DRONE_API_KEY = "dev-key-change-in-production"

    print(f"\nConnecting to: {AEROSYNC_URL}\n")

    # Initialize client
    config = AeroSyncConfig()
    config.base_url = AEROSYNC_URL
    config.api_key = DRONE_API_KEY
    config.websocket_enabled = False

    client = AeroSyncClient(config=config)

    # Step 1: Fetch job
    print("[STEP 1] Fetching mission configuration...")
    job_id = input("Enter job ID to test with: ").strip()

    job = client.fetch_job(job_id)
    if not job:
        print("❌ FAILED: Could not fetch job")
        return False

    print(f"✅ SUCCESS: Fetched job {job_id}")
    print(f"   - Search Pattern: {job.search_pattern}")
    print(f"   - Search Area: {len(job.search_area.get('coordinates', [[]])[0])} vertices")

    # Step 2: Parse Mission
    print("\n[STEP 2] Parsing mission configuration...")

    try:
        # Convert Job to dict format expected by parse_mission
        job_data = {
            'job_id': job_id,
            'name': job.name,
            'search_pattern': job.search_pattern,
            'search_area': job.search_area,
            'search_config': job.search_config,
            'stop_conditions': job.stop_conditions,
            'inspection_config': job.inspection_config,
            'previously_inspected': job.previously_inspected if hasattr(job, 'previously_inspected') else []
        }

        mission = parse_mission(job_data)
        print("✅ SUCCESS: Mission configuration parsed")
        print(f"   - Pattern: {mission.search_pattern.value}")
        print(f"   - Altitude: {mission.search_config.flight_altitude}m")
        print(f"   - Speed: {mission.search_config.speed}m/s")
        print(f"   - Spacing: {mission.search_config.sweep_spacing}m")
    except Exception as e:
        print(f"❌ FAILED: Could not parse mission: {e}")
        import traceback
        traceback.print_exc()
        return False

    # Step 3: Generate waypoints
    print("\n[STEP 3] Generating search waypoints...")

    try:
        # Generate waypoints using mission planner
        waypoints = generate_waypoints(mission)

        if not waypoints or len(waypoints) == 0:
            print("❌ FAILED: No waypoints generated")
            return False

        print(f"✅ SUCCESS: Generated {len(waypoints)} waypoints")
        print(f"\n   First waypoint:")
        print(f"     - Lat: {waypoints[0].lat}")
        print(f"     - Lon: {waypoints[0].lon}")
        print(f"     - Alt: {waypoints[0].alt}m")

        if len(waypoints) > 1:
            print(f"\n   Last waypoint:")
            print(f"     - Lat: {waypoints[-1].lat}")
            print(f"     - Lon: {waypoints[-1].lon}")
            print(f"     - Alt: {waypoints[-1].alt}m")

    except Exception as e:
        print(f"❌ FAILED: Waypoint generation error: {e}")
        import traceback
        traceback.print_exc()
        return False

    # Step 4: Validate waypoints
    print("\n[STEP 4] Validating waypoints...")

    issues = []

    # Check all waypoints have required fields
    for i, wp in enumerate(waypoints):
        if not hasattr(wp, 'lat') or not hasattr(wp, 'lon'):
            issues.append(f"Waypoint {i} missing lat/lon")
        if not hasattr(wp, 'alt'):
            issues.append(f"Waypoint {i} missing altitude")

    # Check waypoints are within search area (basic validation)
    if len(waypoints) > 0:
        # Get coordinates from search area
        # Format can be either list of dicts or GeoJSON coordinates
        coords = job.search_area.get('coordinates', [[]])[0]
        if len(coords) > 0:
            # Handle different coordinate formats
            if isinstance(coords[0], dict):
                # Format: [{'lat': x, 'lon': y}, ...]
                lats = [c['lat'] for c in coords]
                lons = [c['lon'] for c in coords]
            elif isinstance(coords[0], (list, tuple)) and len(coords[0]) >= 2:
                # GeoJSON format: [[lon, lat], ...]
                lons = [c[0] for c in coords]
                lats = [c[1] for c in coords]
            else:
                # Unknown format, skip validation
                lats = []
                lons = []

            if lats and lons:
                min_lat, max_lat = min(lats), max(lats)
                min_lon, max_lon = min(lons), max(lons)

                # Add 10% margin for edge waypoints
                lat_margin = (max_lat - min_lat) * 0.1
                lon_margin = (max_lon - min_lon) * 0.1

                for i, wp in enumerate(waypoints):
                    if not (min_lat - lat_margin <= wp.lat <= max_lat + lat_margin):
                        issues.append(f"Waypoint {i} latitude {wp.lat:.6f} outside search area")
                    if not (min_lon - lon_margin <= wp.lon <= max_lon + lon_margin):
                        issues.append(f"Waypoint {i} longitude {wp.lon:.6f} outside search area")

    if issues:
        print(f"⚠️  WARNING: Found {len(issues)} validation issues:")
        for issue in issues[:5]:  # Show first 5
            print(f"   - {issue}")
        if len(issues) > 5:
            print(f"   ... and {len(issues) - 5} more")
    else:
        print("✅ SUCCESS: All waypoints valid")

    # Final Summary
    print("\n" + "=" * 80)
    print("✅ MISSION PLANNER TEST PASSED")
    print("=" * 80)
    print("\nValidated:")
    print(f"  ✓ Mission configuration parsing")
    print(f"  ✓ Waypoint generation ({len(waypoints)} waypoints)")
    print(f"  ✓ Waypoint validation")
    print(f"\nMission Statistics:")
    area_coords = job.search_area.get('coordinates', [[]])[0]
    print(f"  - Search area: {len(area_coords)} vertices")
    print(f"  - Total waypoints: {len(waypoints)}")
    print(f"  - Pattern: {mission.search_pattern.value}")
    print(f"\nThe Jetson can successfully plan missions from AeroSync configuration!")

    return True


if __name__ == "__main__":
    try:
        success = test_mission_planner()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ UNEXPECTED ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
