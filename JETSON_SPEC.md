# Jetson Drone Code Specification

**Version:** 2.0
**Last Updated:** January 17, 2026

## AeroSync Integration Status

### Completed
- **Mission Planner** - `mission_planner.py` parses AeroSync jobs, generates waypoints
- **Mission Executor** - `mission_executor.py` orchestrates full autonomous missions
- **Flight Control** - `guidance.py` has takeoff, navigate_to_waypoint, RTL, land
- **AeroSync Client** - REST API + WebSocket + S3 upload fully implemented
- **GPS Telemetry** - Position, battery, heading tracked from FCU

### Ready for Integration Testing
- All API endpoints use `/api/drone/` prefix (matches AEROSYNC_SPEC.md)
- WebSocket events match spec: `drone_position`, `abort`, `joined`
- S3 presigned URL upload flow implemented
- Data formats match AeroSync backend expectations

### 🔧 Required Jetson Code Fixes (Jan 16, 2026)

**Status:** AeroSync backend has been updated with all required endpoints. The following Jetson code changes are needed for full compatibility:

#### Fix #1: Send Drone Position Metadata in Photo Confirmation

**File:** `perception_nav/src/aerosync_client.py` (around line 433)

**Issue:** Photo confirmation doesn't include drone position/heading when photo was captured

**Required Change:**
```python
def confirm_photo_upload(self, job_id: str, pole_id: str,
                          photo_id: str, s3_key: str,
                          lat: float = None, lon: float = None,
                          drone_lat: float = None, drone_lon: float = None,
                          drone_alt: float = None, drone_heading: float = None,
                          captured_at: str = None) -> bool:
    """Confirm photo upload with drone position metadata."""
    url = f"{self.config.base_url}/api/drone/jobs/{job_id}/poles/{pole_id}/photo/confirm"
    payload = {
        'photo_id': photo_id,
        's3_key': s3_key,
        'lat': lat,  # Pole GPS
        'lon': lon,
        'drone_lat': drone_lat,  # REQUIRED: Current drone latitude
        'drone_lon': drone_lon,  # REQUIRED: Current drone longitude
        'drone_alt': drone_alt,  # REQUIRED: Current drone altitude
        'drone_heading': drone_heading,  # REQUIRED: Current drone heading
        'captured_at': captured_at  # REQUIRED: ISO8601 timestamp
    }
    self._request_with_retry('POST', url, json=payload)
    return True
```

**Impact:** Without this, photo records won't have drone position data (needed for flight path reconstruction)

---

#### Fix #2: Update Calling Code to Pass Drone Metadata

**File:** Wherever `aerosync_client.confirm_photo_upload()` is called (likely `capture_system.py` or `mission_executor.py`)

**Required Change:**
```python
# Get current drone state from flight controller
current_lat, current_lon = guidance.get_gps_position()
current_alt = guidance.get_altitude()
current_heading = guidance.get_heading()

# Confirm photo upload with full metadata
success = aerosync_client.confirm_photo_upload(
    job_id=job_id,
    pole_id=pole_id,
    photo_id=photo_id,
    s3_key=s3_key,
    lat=pole_gps[0],  # Pole GPS (from detection)
    lon=pole_gps[1],
    drone_lat=current_lat,  # Drone GPS (at capture time)
    drone_lon=current_lon,
    drone_alt=current_alt,
    drone_heading=current_heading,
    captured_at=datetime.utcnow().isoformat() + 'Z'
)
```

**Impact:** Critical for flight path reconstruction, photo analysis, and debugging

---

See [AEROSYNC_SPEC.md](./AEROSYNC_SPEC.md) for complete API reference.

---

## Overview

JetsonLidar is the autonomous drone perception and navigation system running on NVIDIA Jetson hardware. It handles:
1. LiDAR pole detection and tracking
2. Camera + LiDAR sensor fusion
3. Autonomous navigation following utility pole lines
4. Photo capture with GPS tagging
5. Communication with AeroSync mission control

**Technology Stack:** Python + ROS + MAVLink + OpenCV + PyTorch + NumPy

---

## AeroSync API Integration Reference

### API Endpoints Used by Jetson

All endpoints require `Authorization: Bearer {drone_api_key}` header.

| Endpoint | Method | Purpose | Jetson File |
|----------|--------|---------|-------------|
| `/api/drone/jobs/{id}` | GET | Fetch mission config | aerosync_client.py |
| `/api/drone/jobs/{id}/start` | POST | Signal mission start | aerosync_client.py |
| `/api/drone/jobs/{id}/status` | PUT | Periodic status update | aerosync_client.py |
| `/api/drone/jobs/{id}/complete` | POST | Signal mission end | aerosync_client.py |
| `/api/drone/jobs/{id}/poles` | POST | Report detected pole | aerosync_client.py |
| `/api/drone/jobs/{id}/poles/{pole_id}/photo/upload-url` | GET | Get S3 presigned URL | aerosync_client.py |
| `/api/drone/jobs/{id}/poles/{pole_id}/photo/confirm` | POST | Confirm S3 upload | aerosync_client.py |
| `/api/drone/jobs/{id}/poles/{pole_id}/photo` | POST | Direct upload (fallback) | aerosync_client.py |

### Mission Fetch Response (GET /api/drone/jobs/{id})

Jetson expects this response format:

```json
{
  "job_id": "123",
  "name": "Highway 287 Survey",
  "search_pattern": "line_follow",
  "search_area": {
    "type": "Polygon",
    "coordinates": [[[lon, lat], [lon, lat], ...]]
  },
  "search_config": {
    "sweep_spacing": 50.0,
    "flight_altitude": 30.0,
    "speed": 5.0
  },
  "stop_conditions": {
    "max_poles": 50,
    "max_time_minutes": 30,
    "min_battery_percent": 25.0,
    "coverage_threshold": 0.9
  },
  "inspection_config": {
    "min_confidence": 0.6,
    "camera_required": true,
    "capture_distance": 10.0,
    "photos_per_pole": 1
  },
  "previously_inspected": [
    {"id": "456", "lat": 41.318, "lon": -105.568}
  ],
  "state": "pending"
}
```

**Parsed by:** `aerosync_client.Job.from_dict()` → `mission_planner.parse_mission()`

### Pole Report Request (POST /api/drone/jobs/{id}/poles)

Jetson sends this format:

```json
{
  "pole_id": "uuid-from-drone",
  "timestamp": "2026-01-16T14:40:23Z",
  "position": {
    "latitude": 41.12845,
    "longitude": -105.46123,
    "altitude_msl": 2050.5,
    "altitude_agl": 12.3,
    "local_x": 125.4,
    "local_y": -8.2,
    "local_z": 10.5
  },
  "detection": {
    "confidence": 0.87,
    "pole_type": "h_frame",
    "lidar_confidence": 0.92,
    "camera_confidence": 0.81,
    "height_estimate": 14.5,
    "radius_estimate": 0.18
  },
  "metadata": {
    "frames_tracked": 25,
    "drone_heading": 45.2
  }
}
```

**Sent by:** `aerosync_client.report_pole()`

### Status Update Request (PUT /api/drone/jobs/{id}/status)

Jetson sends this format every 10-30 seconds:

```json
{
  "state": "running",
  "guidance_state": "FOLLOW",
  "poles_inspected": 12,
  "elapsed_minutes": 8.5,
  "battery_percent": 85.2,
  "coverage_percent": 0.35,
  "distance_from_home": 450.0,
  "current_position": {
    "lat": 41.318285,
    "lon": -105.568625,
    "alt": 30.5
  }
}
```

**Sent by:** `aerosync_client.update_status()`

### Mission Complete Request (POST /api/drone/jobs/{id}/complete)

Jetson sends this format:

```json
{
  "stop_reason": "coverage_complete",
  "summary": {
    "poles_inspected": 47,
    "poles_skipped_duplicate": 3,
    "duration_minutes": 28.5,
    "final_battery_percent": 32.1,
    "final_coverage_percent": 0.95
  }
}
```

**Stop reasons:** `coverage_complete`, `max_poles_reached`, `time_limit`, `low_battery`, `user_abort`, `failsafe`, `error`

**Sent by:** `aerosync_client.complete_job()`

### WebSocket Events

#### Drone → Server

**drone_position** (1 Hz):
```json
{
  "event": "drone_position",
  "job_id": "123",
  "lat": 41.318285,
  "lon": -105.568625,
  "alt": 30.5,
  "heading": 45.2,
  "speed": 3.5,
  "battery": 85.2,
  "timestamp": 1705412423.456
}
```

**Sent by:** `aerosync_client.send_position()`

#### Server → Drone

**abort** - Operator requested mission abort:
```json
{
  "event": "abort",
  "job_id": "123"
}
```

**Handled by:** `aerosync_client._handle_ws_message()` → `_on_abort` callback

### S3 Photo Upload Flow

1. **Get presigned URL:**
   ```
   GET /api/drone/jobs/{id}/poles/{pole_id}/photo/upload-url
   Response: {upload_url, photo_id, s3_key, expires_in}
   ```

2. **Upload to S3:**
   ```
   PUT {upload_url}
   Content-Type: image/jpeg
   Body: raw image bytes
   ```

3. **Confirm upload:**
   ```
   POST /api/drone/jobs/{id}/poles/{pole_id}/photo/confirm
   Body: {photo_id, s3_key}
   ```

**Implemented in:** `aerosync_client.upload_photo_s3()`

---

## System Architecture

### Module Overview

```
perception_nav/src/
├── pole_detector.py      # LiDAR pole detection + Kalman tracking
├── fusion.py             # Camera-LiDAR sensor fusion
├── line_follower.py      # Single-line pole following
├── smooth_controller.py  # 100Hz Tesla-style control
├── guidance.py           # MAVLink FCU interface + flight commands
├── mission_planner.py    # Parse AeroSync jobs → waypoints
├── mission_manager.py    # Mission state + stop conditions
├── mission_executor.py   # Full autonomous mission orchestration
├── aerosync_client.py    # REST + WebSocket + S3 client
├── capture_system.py     # Photo capture + async bundling
├── gps_reader.py         # GPS module interface
└── pole_inspector.py     # Pole validation + capture readiness
```

### Data Flow

```
AeroSync                    Jetson
   │                          │
   │ GET /api/drone/jobs/{id} │
   │◄─────────────────────────│
   │ {job config}             │
   │─────────────────────────►│
   │                          │
   │                          ▼
   │                    mission_planner.py
   │                    parse_mission()
   │                    generate_waypoints()
   │                          │
   │                          ▼
   │                    mission_executor.py
   │                    takeoff → transit → search
   │                          │
   │ POST jobs/{id}/start     │
   │◄─────────────────────────│
   │                          │
   │                          ▼
   │                    Perception Loop (10-30 Hz)
   │                    ├── pole_detector.py
   │                    ├── fusion.py
   │                    └── line_follower.py
   │                          │
   │ POST jobs/{id}/poles     │
   │◄─────────────────────────│ (on pole detection)
   │                          │
   │ WS: drone_position       │
   │◄─────────────────────────│ (1 Hz)
   │                          │
   │ PUT jobs/{id}/status     │
   │◄─────────────────────────│ (every 10-30s)
   │                          │
   │ GET .../photo/upload-url │
   │◄─────────────────────────│
   │ {presigned URL}          │
   │─────────────────────────►│
   │                          │
   │ PUT S3 (direct)          │
   │◄─────────────────────────│
   │                          │
   │ POST .../photo/confirm   │
   │◄─────────────────────────│
   │                          │
   │                          ▼
   │                    Stop condition met
   │                    RTL → Land
   │                          │
   │ POST jobs/{id}/complete  │
   │◄─────────────────────────│
   │ {summary}                │
```

---

## Mission Execution Flow

### Full Autonomous Mission (`mission_executor.py`)

```python
from perception_nav.src.mission_executor import MissionExecutor

executor = MissionExecutor(config_path='config/settings.yaml')
executor.initialize()
executor.connect_fcu('/dev/ttyACM0')
await executor.run_mission('job-uuid')
```

### Execution States

```
IDLE
  │
  ▼
FETCHING_MISSION ──► (fetch job from AeroSync)
  │
  ▼
VALIDATING ─────────► (validate mission params)
  │
  ▼
TAKEOFF ────────────► (arm, guided mode, takeoff)
  │
  ▼
TRANSIT_TO_AREA ───► (navigate to closest corner of search area)
  │
  ▼
SEARCHING ─────────► (execute lawnmower pattern)
  │
  ├── (poles detected) ──► LINE_FOLLOWING
  │                            │
  ▼                            ▼
INSPECTING ◄──────────────────┘ (capture photos)
  │
  ▼
RETURNING ─────────► (RTL mode)
  │
  ▼
LANDING ───────────► (wait for disarm)
  │
  ▼
COMPLETED
```

### Stop Conditions Monitored

| Condition | Check | Action |
|-----------|-------|--------|
| Max poles reached | `poles_inspected >= max_poles` | Complete mission |
| Time limit | `elapsed > max_time_minutes` | Complete mission |
| Low battery | `battery < min_battery_percent` | RTL immediately |
| Coverage complete | `coverage >= coverage_threshold` | Complete mission |
| Max distance | `distance_from_home > max_distance` | RTL |
| User abort | WebSocket `abort` event | RTL immediately |
| Heartbeat loss | FCU connection lost | Hold position → RTL |
| Obstacle detected | LiDAR corridor check | Stop → resume when clear |

---

## Search Patterns

### 1. Line Follow (`search_pattern: "line_follow"`)

- Jetson navigates to start point
- Autonomous pole following using `line_follower.py`
- Lateral offset (fly 10m left/right of poles)
- Look-ahead path planning

**Waypoints generated:** 1 (just start point)

### 2. Lawnmower (`search_pattern: "lawnmower"`)

- Parallel north-south sweeps across polygon
- Starts from closest corner to home (minimizes transit)
- Sweep spacing from `search_config.sweep_spacing`
- Alternating direction for efficient coverage

**Waypoints generated:** Multiple (depends on area size and spacing)

### 3. Spiral (`search_pattern: "spiral"`)

- Outward spiral from polygon center
- Good for searching outward from a known center point
- Spacing controlled by `sweep_spacing`

**Waypoints generated:** Multiple (capped at 500 for performance)

---

## Flight Control Interface

### High-Level Commands (`guidance.py`)

```python
# Takeoff to altitude
success = guidance.takeoff(altitude=30.0, timeout=60.0)

# Navigate to GPS waypoint
success = guidance.navigate_to_waypoint(
    lat=41.318, lon=-105.568, alt=30.0,
    speed=5.0, threshold_m=3.0, timeout=300.0
)

# Return to launch
guidance.return_to_launch()

# Land at current position
guidance.land()

# Hold position
guidance.hold_position()
```

### Telemetry Available

```python
lat, lon = guidance.get_gps_position()  # Degrees
alt = guidance.get_altitude()            # Meters AGL
battery = guidance.get_battery_percent() # 0-100%
```

### Velocity Control (for autonomous navigation)

```python
guidance.send_velocity(
    vx=1.0,      # Forward (m/s)
    vy=0.0,      # Right (m/s)
    vz=0.0,      # Down (m/s)
    yaw_rate=0.0 # Rad/s
)
```

---

## Configuration

### AeroSync Connection (`config/settings.yaml`)

```yaml
aerosync:
  base_url: "https://aerosync.example.com"
  api_key: "your-drone-api-key"
  websocket_enabled: true
  upload_photos: true
  position_update_rate_hz: 1.0
  retry_max_attempts: 3
  retry_base_delay: 1.0
  timeout_seconds: 30.0
```

### Flight Controller

```yaml
guidance:
  connection: '/dev/ttyACM0'
  baud_rate: 57600
  max_forward_velocity: 5.0
  max_lateral_velocity: 2.0
  max_vertical_velocity: 1.0
  heartbeat_timeout: 2.0
```

### Mission Defaults

```yaml
mission:
  default_altitude: 30.0
  default_speed: 3.0
  arrival_threshold: 5.0
  waypoint_timeout: 300.0
  mission_timeout: 3600.0
```

---

## Testing

### Without Hardware (Office Testing)

```bash
# Replay recorded flight data
python3 fusion_ros.py --replay --lidar-topic /aeva_0/points --camera-topic /camera/image_raw

# Play rosbag
rosbag play flight_captures/<timestamp>/*.bag --loop
```

### SITL Testing

```bash
# Start ArduPilot SITL
sim_vehicle.py -v ArduCopter --console --map

# Connect Jetson code
guidance = MavlinkGuidance()
guidance.connect('udp:127.0.0.1:14550')
```

### Integration Test (No Flight)

```python
# Test full mission flow without actual flight
executor = MissionExecutor()
executor.initialize()
# Skip FCU connect
await executor.run_mission('test-job-id')  # Will fail at takeoff
```

---

## Error Handling

### Network Errors

- REST requests: Exponential backoff retry (1s, 2s, 4s)
- WebSocket: Auto-reconnect with 5s delay
- S3 upload: 3 retries with backoff

### FCU Errors

- Heartbeat loss: Hold position, then RTL after 5s
- Arming failure: Abort mission
- Navigation timeout: Log warning, continue to next waypoint

### Perception Errors

- No poles detected: Continue search pattern
- Confidence drop: Enter RESCAN state
- Obstacle detected: OBSTACLE_STOP state

---

## Deployment Checklist

### Hardware Setup
- [ ] Jetson Orin/Xavier with Ubuntu 20.04
- [ ] Aeva Atlas LiDAR connected (10.5.50.100)
- [ ] USB camera connected (/dev/video0)
- [ ] CubeOrange+ connected (/dev/ttyACM0)
- [ ] GPS module connected
- [ ] Network connectivity (Starlink/LTE)

### Software Setup
```bash
pip install pymavlink numpy scipy scikit-learn pyyaml requests aiohttp
# PyTorch: follow Jetson-specific instructions
```

### Configuration
- [ ] Update `config/settings.yaml` with AeroSync URL and API key
- [ ] Calibrate camera-LiDAR extrinsics
- [ ] Configure ArduPilot parameters
- [ ] Test sensor connectivity

### Pre-Flight
- [ ] GPS fix (>4 satellites)
- [ ] LiDAR publishing data
- [ ] Camera streaming
- [ ] FCU connection verified
- [ ] AeroSync reachable
- [ ] Battery charged

---

## File Reference

| File | Purpose | Lines |
|------|---------|-------|
| `mission_planner.py` | Parse jobs, generate waypoints | ~850 |
| `mission_executor.py` | Orchestrate missions | ~600 |
| `mission_manager.py` | State tracking, stop conditions | ~600 |
| `aerosync_client.py` | REST + WebSocket + S3 | ~800 |
| `guidance.py` | MAVLink FCU interface | ~1200 |
| `pole_detector.py` | LiDAR pole detection | ~1000 |
| `fusion.py` | Sensor fusion | ~550 |
| `line_follower.py` | Line following | ~530 |
| `smooth_controller.py` | 100Hz control | ~930 |
| `capture_system.py` | Photo capture | ~840 |

---

## Support

**Repository:** `/home/nvidia/Projects/JetsonLidar`

**AeroSync Integration:** See `AEROSYNC_SPEC.md`

**ArduPilot Docs:** https://ardupilot.org/copter/

**Aeva Atlas:** http://10.5.50.100/ (when connected)
