# Jetson Drone Code Specification

**Version:** 1.0
**Last Updated:** January 16, 2026

## Overview

JetsonLidar is the autonomous drone perception and navigation system running on NVIDIA Jetson hardware. It handles:
1. LiDAR pole detection and tracking
2. Camera + LiDAR sensor fusion
3. Autonomous navigation following utility pole lines
4. Photo capture with GPS tagging
5. Communication with AeroSync mission control

**Technology Stack:** Python + ROS + MAVLink + OpenCV + PyTorch + NumPy

---

## System Goals

### Operational Workflow
1. Fetch mission from AeroSync (search area, altitude, speed, stop conditions)
2. Auto-takeoff to mission altitude
3. Navigate to mission start point using flight controller autopilot
4. Switch to Jetson control (guided mode)
5. Execute autonomous pole inspection:
   - Detect poles with LiDAR + camera
   - Report poles to AeroSync immediately
   - Capture photos when near poles
   - Upload photos to S3 via AeroSync API
6. Monitor stop conditions (poles found, time, battery, coverage)
7. Return to launch when mission complete
8. Land autonomously

### Hardware Integration
- **Jetson Orin/Xavier** - Main computer running perception + navigation
- **Aeva Atlas FMCW LiDAR** - 4D LiDAR for pole detection
- **USB Camera** - Visual inspection with YOLO object detection
- **CubeOrange+ Flight Controller** - ArduPilot running on Pixhawk
- **Jetson GPS Module** - Position tracking
- **Starlink + LTE** - Network connectivity
- **Gremsy Gimbal** (optional) - Camera stabilization and pointing

---

## What We Have

### Perception System (95% Complete)

#### Pole Detection (`perception_nav/src/pole_detector.py`)

**Features:**
- RANSAC ground plane removal
- DBSCAN clustering for pole candidates
- Cylinder fitting with verticality check
- Kalman filter tracking (maintains pole IDs across frames)
- Moving object rejection using FMCW velocity data
- Temporal consistency filtering
- Deduplication (spatial grid)

**Performance:**
- Real-time on Jetson Orin (10-20 Hz)
- Detection range: 0.5-50 meters
- Minimum pole height: 0.5m
- Maximum pole height: 20m
- Pole radius: 0.05-0.3m

**Output:**
```python
class DetectedPole:
    pole_id: int              # Unique track ID
    position: np.ndarray      # [x, y, z] in local NED
    confidence: float         # 0-1
    pole_type: str            # 'utility_pole', 'h_frame', etc.
    height: float             # Meters
    radius: float             # Meters
    frames_tracked: int       # Tracking stability
    lidar_confidence: float   # LiDAR-specific confidence
```

#### Sensor Fusion (`perception_nav/src/fusion.py`)

**Features:**
- LiDAR + camera fusion with temporal filtering
- Camera-LiDAR extrinsic calibration
- Project LiDAR points into camera frame
- Associate YOLO detections with LiDAR poles
- Confidence weighting (60% LiDAR, 40% camera, +20% boost for matches)
- Temporal window tracking (10 frames)

**Configuration:**
```yaml
sensor_fusion:
  camera_fx: 800.0
  camera_fy: 800.0
  camera_cx: 640.0
  camera_cy: 360.0
  extrinsic_tx: 0.0
  extrinsic_ty: 0.0
  extrinsic_tz: -0.1
  association_threshold_px: 150.0
  weight_lidar: 0.6
  weight_camera: 0.4
  min_fused_confidence: 0.3
```

**Output:**
```python
class FusedPole:
    pole_id: int
    position: np.ndarray
    confidence: float            # Combined confidence
    lidar_confidence: float
    camera_confidence: float
    pole_type: str
    camera_matched: bool         # True if YOLO confirmed
```

#### GPS Reader (`perception_nav/src/gps_reader.py`)

**Features:**
- Serial GPS reading (NMEA protocol)
- EMA smoothing
- Stale data detection
- Lat/lon/alt output at 5 Hz

**Configuration:**
```yaml
gps:
  enabled: true
  port: '/dev/ttyUSB1'
  baud: 9600
  min_satellites: 4
  smoothing_alpha: 0.3
  max_age: 2.0
```

#### Photo Capture (`perception_nav/src/capture_system.py`)

**Features:**
- Ring buffer for camera frames (zero-copy, O(1) insert)
- Event-driven capture scheduler (pre-computed triggers)
- Async data bundler (never blocks main loop)
- GPS tagging
- Path corridor obstacle detection

**Usage:**
```python
capture_system = CaptureSystem(output_dir='/data/captures')

# Schedule capture when near pole
capture_system.schedule_capture(
    pole_id=pole.pole_id,
    pole_position=pole.position,
    distance=10.0
)

# Push camera frames to ring buffer
capture_system.push_frame(frame, timestamp)

# Async save to disk
capture_system.save_async(image, metadata)
```

### Navigation System (90% Complete)

#### Line Follower (`perception_nav/src/line_follower.py`)

**Features:**
- Follow single line of utility poles
- Lateral offset control (fly left or right of poles)
- EMA smoothing of pole line estimate
- Look-ahead path following
- Capture distance triggering
- Pole spacing estimation (expected 75m apart)

**Configuration:**
```yaml
line_follower:
  lateral_offset: 10.0          # Fly 10m left of poles
  fly_on_left: true
  min_poles_for_line: 1
  look_ahead_distance: 20.0
  expected_pole_spacing: 75.0
  capture_distance: 10.0
  min_confidence: 0.3
```

**Output:**
```python
class NavigationCommand:
    lateral_offset: float      # Meters left of line
    heading: float             # Radians
    speed: float               # m/s
    capture_triggered: bool    # True when near pole
    confidence: float          # Navigation confidence
```

#### Smooth Controller (`perception_nav/src/smooth_controller.py`)

**Features:**
- 100 Hz control loop (Tesla-style)
- Dynamic look-ahead (increases with speed)
- Jerk limiting for smooth motion
- Curvature-based speed reduction
- Predictive control (MPC-lite)
- Path spline smoothing
- Max acceleration/deceleration limits

**Configuration:**
```yaml
smooth_controller:
  control_rate_hz: 100.0
  max_speed: 5.0
  cruise_speed: 3.0
  max_accel: 1.0
  max_decel: 1.5
  max_jerk: 2.0
  lateral_p: 0.8
  lateral_d: 0.3
  heading_p: 1.2
  heading_d: 0.4
```

**Output:**
```python
class VelocityCommand:
    vx: float    # Forward velocity (m/s)
    vy: float    # Lateral velocity (m/s)
    vz: float    # Vertical velocity (m/s)
    yaw_rate: float  # Radians/sec
```

#### Corridor Follower (`perception_nav/src/corridor_follower.py`)

**Features:**
- Check corridor ahead for obstacles
- Slow down when obstacle detected
- Stop if obstacle too close
- Resume when clear

**Configuration:**
```yaml
obstacle_detection:
  enabled: true
  corridor_length: 15.0
  corridor_width: 2.0
  min_obstacle_points: 10
  danger_distance: 5.0
```

#### Flight Controller Interface (`perception_nav/src/guidance.py`)

**Features:**
- MAVLink communication with CubeOrange+
- Velocity command generation (NED frame)
- Heartbeat monitoring
- Failsafe handling (GPS loss, RC loss, battery)
- Mode switching (GUIDED, AUTO, RTL)
- Telemetry reading (position, attitude, battery)

**Configuration:**
```yaml
guidance:
  connection: '/dev/ttyACM0'
  baud_rate: 57600
  max_forward_velocity: 2.0
  max_lateral_velocity: 1.0
  max_vertical_velocity: 0.5
  max_angular_velocity: 0.5
  heartbeat_timeout: 2.0
```

**Methods:**
```python
def send_velocity(vx, vy, vz, yaw_rate):
    """Send velocity command to FC."""

def get_position():
    """Get current GPS position."""

def get_attitude():
    """Get current roll/pitch/yaw."""

def get_battery():
    """Get battery percent."""
```

### Mission Management (70% Complete)

#### Mission Manager (`perception_nav/src/mission_manager.py`)

**Features:**
- Mission state tracking (NOT_STARTED, RUNNING, PAUSED, COMPLETED, ABORTED)
- Stop conditions monitoring:
  - Max poles reached
  - Time limit exceeded
  - Low battery
  - Coverage threshold met
  - Max distance from home
- Mission summary generation
- Event logging

**Usage:**
```python
stop_conditions = StopConditions(
    max_poles=50,
    max_time_minutes=30,
    min_battery_percent=25.0,
    coverage_threshold=0.9
)

manager = MissionManager(stop_conditions)
manager.start()

# Check stop conditions
should_stop, reason = manager.should_stop(
    battery_percent=current_battery,
    distance_from_home=dist
)
```

**Missing:**
- Mission parser (convert AeroSync job to Mission object)
- Waypoint generator (convert search area to flight path)

#### AeroSync Client (`perception_nav/src/aerosync_client.py`)

**Features:**
- REST API client with retry logic
- Methods for job fetch, pole report, status update
- Offline queue for store-and-forward
- Configuration from YAML

**Methods Implemented:**
```python
async def fetch_job(job_id: int) -> dict:
    """Fetch mission config from AeroSync."""

async def start_job(job_id: int):
    """Signal mission start."""

async def update_status(job_id: int, status: dict):
    """Send status update."""

async def report_pole(job_id: int, pole: DetectedPole):
    """Report detected pole."""

async def complete_job(job_id: int, summary: dict):
    """Signal mission completion."""
```

**Missing:**
- WebSocket client (position broadcast, command reception)
- Photo upload to S3 (presigned URL flow)

### Configuration System (Complete)

**File:** `config/settings.yaml`

Complete configuration for all systems:
- Camera, LiDAR, GPS settings
- YOLO detection parameters
- Sensor fusion calibration
- Navigation gains and limits
- Mission stop conditions
- AeroSync connection details

---

## What We Need

### Priority 1: Mission Receiver (1-2 days)

**Goal:** Parse mission from AeroSync and generate flight waypoints.

**File to Modify:** `perception_nav/src/mission_manager.py`

**Requirements:**

1. **Mission Parser:**

```python
@dataclass
class Mission:
    """Parsed mission from AeroSync."""
    job_id: int
    name: str
    search_area: dict              # GeoJSON polygon
    search_pattern: str            # 'line_follow' | 'lawnmower' | 'spiral'
    start_point: Tuple[float, float, float]  # (lat, lon, alt)
    home_point: Tuple[float, float, float]
    search_config: dict            # {sweep_spacing, flight_altitude, speed}
    stop_conditions: StopConditions
    inspection_config: dict        # {min_confidence, camera_required, capture_distance}
    previously_inspected: List[Tuple[float, float]]  # [(lat, lon), ...]

def parse_mission(job_data: dict) -> Mission:
    """Parse job data from AeroSync into Mission object."""
    return Mission(
        job_id=job_data['job_id'],
        name=job_data['name'],
        search_area=job_data['search_area'],
        search_pattern=job_data['search_pattern'],
        start_point=(
            job_data['search_area']['coordinates'][0][0][1],  # lat
            job_data['search_area']['coordinates'][0][0][0],  # lon
            job_data['search_config']['flight_altitude']
        ),
        search_config=job_data['search_config'],
        stop_conditions=StopConditions.from_dict(job_data['stop_conditions']),
        inspection_config=job_data['inspection_config'],
        previously_inspected=[
            (p['lat'], p['lon']) for p in job_data.get('previously_inspected', [])
        ]
    )
```

2. **Waypoint Generator:**

```python
@dataclass
class Waypoint:
    lat: float
    lon: float
    alt: float
    speed: float

def generate_waypoints(mission: Mission) -> List[Waypoint]:
    """Generate waypoint list based on search pattern."""

    if mission.search_pattern == 'line_follow':
        # Simple: just start point
        # Jetson will follow detected poles autonomously
        return [Waypoint(
            lat=mission.start_point[0],
            lon=mission.start_point[1],
            alt=mission.start_point[2],
            speed=mission.search_config['speed']
        )]

    elif mission.search_pattern == 'lawnmower':
        # Generate parallel sweeps across polygon
        return generate_lawnmower_waypoints(
            polygon=mission.search_area['coordinates'][0],
            spacing=mission.search_config['sweep_spacing'],
            altitude=mission.search_config['flight_altitude'],
            speed=mission.search_config['speed']
        )

    elif mission.search_pattern == 'spiral':
        # Generate outward spiral from center
        return generate_spiral_waypoints(
            polygon=mission.search_area['coordinates'][0],
            altitude=mission.search_config['flight_altitude'],
            speed=mission.search_config['speed']
        )

    else:
        raise ValueError(f"Unknown search pattern: {mission.search_pattern}")
```

3. **Lawnmower Pattern Generator:**

```python
def generate_lawnmower_waypoints(polygon: List[List[float]],
                                  spacing: float,
                                  altitude: float,
                                  speed: float) -> List[Waypoint]:
    """Generate parallel sweep waypoints covering polygon."""

    # Find bounding box
    lons = [p[0] for p in polygon]
    lats = [p[1] for p in polygon]
    min_lon, max_lon = min(lons), max(lons)
    min_lat, max_lat = min(lats), max(lats)

    # Convert spacing from meters to degrees (rough approximation)
    # 1 degree latitude ≈ 111km = 111000m
    lat_spacing = spacing / 111000.0

    waypoints = []
    current_lat = min_lat
    sweep_direction = 1  # 1 = west to east, -1 = east to west

    while current_lat <= max_lat:
        if sweep_direction == 1:
            wp = Waypoint(current_lat, min_lon, altitude, speed)
        else:
            wp = Waypoint(current_lat, max_lon, altitude, speed)

        waypoints.append(wp)
        current_lat += lat_spacing
        sweep_direction *= -1

    return waypoints
```

4. **Mission Validation:**

```python
def validate_mission(mission: Mission) -> Tuple[bool, str]:
    """Validate mission parameters before execution."""

    # Check altitude
    if mission.start_point[2] <= 0:
        return False, "Altitude must be > 0"

    if mission.start_point[2] > 100:
        return False, "Altitude too high (max 100m)"

    # Check speed
    speed = mission.search_config.get('speed', 0)
    if speed <= 0 or speed > 10:
        return False, "Speed must be 0-10 m/s"

    # Check polygon is closed
    coords = mission.search_area['coordinates'][0]
    if coords[0] != coords[-1]:
        return False, "Search area polygon not closed"

    # Estimate battery consumption
    # TODO: Calculate estimated time and distance
    # Compare with battery capacity

    return True, "Valid"
```

### Priority 2: WebSocket Client (1 day)

**Goal:** Real-time communication with AeroSync (position broadcast, command reception).

**File to Modify:** `perception_nav/src/aerosync_client.py`

**Requirements:**

1. **Install WebSocket Library:**

```bash
pip install websockets python-socketio aiohttp
```

2. **WebSocket Connection:**

```python
import socketio

class AeroSyncClient:
    def __init__(self, base_url: str, api_key: str):
        self.base_url = base_url
        self.api_key = api_key
        self.sio = socketio.AsyncClient()
        self.ws_connected = False

        # Register event handlers
        self.sio.on('connect', self._on_connect)
        self.sio.on('disconnect', self._on_disconnect)
        self.sio.on('abort', self._on_abort)
        self.sio.on('pause', self._on_pause)
        self.sio.on('resume', self._on_resume)

    async def connect_websocket(self, job_id: int):
        """Connect to AeroSync WebSocket."""
        ws_url = self.base_url.replace('http', 'ws')

        try:
            await self.sio.connect(ws_url)
            self.ws_connected = True

            # Join job room as drone
            await self.sio.emit('join_job', {
                'job_id': job_id,
                'type': 'drone'
            })

            logger.info(f"Connected to WebSocket for job {job_id}")

        except Exception as e:
            logger.error(f"WebSocket connection failed: {e}")
            self.ws_connected = False

    async def _on_connect(self):
        """Handle WebSocket connect."""
        logger.info("WebSocket connected")

    async def _on_disconnect(self):
        """Handle WebSocket disconnect."""
        logger.warning("WebSocket disconnected")
        self.ws_connected = False
        # TODO: Auto-reconnect

    async def _on_abort(self, data):
        """Handle abort command from AeroSync."""
        logger.warning(f"Abort command received: {data}")
        # Trigger RTL in mission manager
        self.mission_manager.abort_mission()

    async def _on_pause(self, data):
        """Handle pause command."""
        logger.info("Pause command received")
        self.mission_manager.pause_mission()

    async def _on_resume(self, data):
        """Handle resume command."""
        logger.info("Resume command received")
        self.mission_manager.resume_mission()
```

3. **Position Broadcast:**

```python
async def position_broadcast_loop(self, job_id: int):
    """Broadcast position at 1 Hz."""

    while self.ws_connected and self.mission_active:
        try:
            # Get current position from GPS + IMU
            position = self.get_current_position()
            battery = self.get_battery_percent()

            # Send position update
            await self.sio.emit('drone_position', {
                'job_id': job_id,
                'lat': position.lat,
                'lon': position.lon,
                'alt': position.alt,
                'heading': position.heading,
                'speed': position.speed,
                'battery': battery,
                'timestamp': time.time()
            })

            await asyncio.sleep(1.0)  # 1 Hz

        except Exception as e:
            logger.error(f"Position broadcast error: {e}")
            await asyncio.sleep(1.0)
```

4. **Start Background Thread:**

```python
async def start_position_broadcast(self, job_id: int):
    """Start position broadcast in background."""
    self.mission_active = True
    self.broadcast_task = asyncio.create_task(
        self.position_broadcast_loop(job_id)
    )

async def stop_position_broadcast(self):
    """Stop position broadcast."""
    self.mission_active = False
    if self.broadcast_task:
        self.broadcast_task.cancel()
```

### Priority 3: Photo Upload with S3 (1-2 days)

**Goal:** Upload captured photos to S3 via AeroSync presigned URLs.

**File to Modify:** `perception_nav/src/aerosync_client.py`

**Requirements:**

1. **Request Presigned URL:**

```python
async def get_photo_upload_url(self, job_id: int, pole_id: int) -> dict:
    """Request S3 presigned URL for photo upload."""
    url = f"{self.base_url}/api/drone/jobs/{job_id}/poles/{pole_id}/photo/upload-url"

    async with aiohttp.ClientSession() as session:
        async with session.get(url, headers=self.headers) as response:
            response.raise_for_status()
            data = await response.json()
            return data
            # Returns: {upload_url, photo_id, s3_key, expires_in}
```

2. **Upload to S3:**

```python
async def upload_photo_to_s3(self, upload_url: str, image_data: bytes) -> bool:
    """Upload photo directly to S3 using presigned URL."""
    try:
        async with aiohttp.ClientSession() as session:
            async with session.put(
                upload_url,
                data=image_data,
                headers={'Content-Type': 'image/jpeg'}
            ) as response:
                return response.status == 200

    except Exception as e:
        logger.error(f"S3 upload failed: {e}")
        return False
```

3. **Confirm Upload:**

```python
async def confirm_photo_upload(self, job_id: int, pole_id: int,
                                 photo_id: str, s3_key: str):
    """Confirm photo upload completion to AeroSync."""
    url = f"{self.base_url}/api/drone/jobs/{job_id}/poles/{pole_id}/photo/confirm"

    async with aiohttp.ClientSession() as session:
        async with session.post(url,
                                 json={'photo_id': photo_id, 's3_key': s3_key},
                                 headers=self.headers) as response:
            response.raise_for_status()
```

4. **Complete Upload Flow:**

```python
async def upload_photo(self, job_id: int, pole_id: int,
                        image: np.ndarray, metadata: dict):
    """Complete photo upload flow."""

    # 1. Encode image as JPEG
    success, buffer = cv2.imencode('.jpg', image,
                                   [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not success:
        logger.error("Failed to encode image")
        return

    image_data = buffer.tobytes()

    # 2. Request presigned URL
    try:
        upload_info = await self.get_photo_upload_url(job_id, pole_id)
    except Exception as e:
        logger.error(f"Failed to get upload URL: {e}")
        return

    # 3. Upload to S3
    success = await self.upload_photo_to_s3(upload_info['upload_url'], image_data)
    if not success:
        logger.error("S3 upload failed")
        return

    # 4. Confirm upload
    try:
        await self.confirm_photo_upload(
            job_id, pole_id,
            upload_info['photo_id'],
            upload_info['s3_key']
        )
        logger.info(f"Photo uploaded successfully: pole {pole_id}")
    except Exception as e:
        logger.error(f"Upload confirmation failed: {e}")
```

5. **Retry Logic:**

```python
async def upload_photo_with_retry(self, job_id: int, pole_id: int,
                                    image: np.ndarray, metadata: dict,
                                    max_retries: int = 3):
    """Upload photo with exponential backoff retry."""

    for attempt in range(max_retries):
        try:
            await self.upload_photo(job_id, pole_id, image, metadata)
            return  # Success

        except Exception as e:
            if attempt == max_retries - 1:
                logger.error(f"Upload failed after {max_retries} attempts: {e}")
                # Queue for later retry
                self.offline_queue.add('photo', {
                    'job_id': job_id,
                    'pole_id': pole_id,
                    'image': image,
                    'metadata': metadata
                })
                return

            # Exponential backoff: 1s, 2s, 4s
            await asyncio.sleep(2 ** attempt)
```

6. **Wire to Capture System:**

```python
# In capture_system.py
async def on_photo_captured(self, pole_id: int, image: np.ndarray, metadata: dict):
    """Called when photo is captured."""
    # Upload asynchronously (don't block main loop)
    asyncio.create_task(
        self.aerosync_client.upload_photo_with_retry(
            self.job_id, pole_id, image, metadata
        )
    )
```

### Priority 4: Flight Control Integration (2-3 days)

**Goal:** Full mission execution from takeoff to landing.

**File to Modify:** `perception_nav/src/guidance.py`

**Requirements:**

1. **Auto-Takeoff:**

```python
def takeoff(self, altitude: float, timeout: float = 60.0):
    """Takeoff to specified altitude."""

    logger.info(f"Takeoff to {altitude}m")

    # Set mode to GUIDED
    self.vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)

    # Arm motors
    self.vehicle.armed = True
    while not self.vehicle.armed:
        logger.info("Waiting for arming...")
        time.sleep(1)

    logger.info("Armed")

    # Send takeoff command
    self.vehicle.simple_takeoff(altitude)

    # Wait until altitude reached
    start_time = time.time()
    while time.time() - start_time < timeout:
        current_alt = self.vehicle.location.global_relative_frame.alt

        if current_alt >= altitude * 0.95:
            logger.info(f"Reached target altitude: {current_alt}m")
            return True

        logger.info(f"Altitude: {current_alt}m")
        time.sleep(0.5)

    logger.error("Takeoff timeout")
    return False
```

2. **Navigate to Start Point:**

```python
def navigate_to_waypoint(self, lat: float, lon: float, alt: float,
                          threshold_m: float = 2.0, timeout: float = 300.0):
    """Navigate to waypoint using FC autopilot."""

    logger.info(f"Navigate to ({lat}, {lon}, {alt})")

    # Set mode to GUIDED
    self.vehicle.mode = VehicleMode("GUIDED")

    # Create target location
    target = LocationGlobalRelative(lat, lon, alt)

    # Send goto command
    self.vehicle.simple_goto(target)

    # Wait until arrived
    start_time = time.time()
    while time.time() - start_time < timeout:
        distance = self.get_distance_to_target(target)

        if distance < threshold_m:
            logger.info(f"Arrived at waypoint (distance: {distance}m)")
            return True

        logger.info(f"Distance to target: {distance}m")
        time.sleep(1.0)

    logger.error("Navigation timeout")
    return False

def get_distance_to_target(self, target: LocationGlobalRelative) -> float:
    """Calculate distance to target in meters."""
    from math import radians, sin, cos, sqrt, atan2

    # Haversine formula
    lat1 = radians(self.vehicle.location.global_frame.lat)
    lon1 = radians(self.vehicle.location.global_frame.lon)
    lat2 = radians(target.lat)
    lon2 = radians(target.lon)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    distance = 6371000 * c  # Earth radius in meters

    return distance
```

3. **Return to Launch:**

```python
def return_to_launch(self):
    """Command drone to return home and land."""

    logger.info("RTL commanded")
    self.vehicle.mode = VehicleMode("RTL")

    # Monitor until landed
    while self.vehicle.armed:
        logger.info(f"RTL - Alt: {self.vehicle.location.global_relative_frame.alt}m")
        time.sleep(2.0)

    logger.info("Landed")
```

4. **Mission Execution Flow:**

```python
async def execute_mission(self, job_id: int):
    """Execute full autonomous mission."""

    logger.info(f"Starting mission {job_id}")

    # 1. Fetch mission from AeroSync
    job_data = await self.aerosync_client.fetch_job(job_id)
    mission = parse_mission(job_data)

    # 2. Validate mission
    valid, error = validate_mission(mission)
    if not valid:
        logger.error(f"Mission validation failed: {error}")
        return

    # 3. Notify AeroSync of start
    await self.aerosync_client.start_job(job_id)

    # 4. Connect WebSocket for real-time updates
    await self.aerosync_client.connect_websocket(job_id)
    await self.aerosync_client.start_position_broadcast(job_id)

    # 5. Takeoff
    success = self.guidance.takeoff(mission.search_config['flight_altitude'])
    if not success:
        logger.error("Takeoff failed")
        await self.aerosync_client.complete_job(job_id, {'stop_reason': 'takeoff_failed'})
        return

    # 6. Navigate to start point
    start_lat, start_lon, start_alt = mission.start_point
    success = self.guidance.navigate_to_waypoint(start_lat, start_lon, start_alt)
    if not success:
        logger.error("Navigation to start failed")
        self.guidance.return_to_launch()
        await self.aerosync_client.complete_job(job_id, {'stop_reason': 'navigation_failed'})
        return

    # 7. Switch to Jetson control (already in GUIDED mode)
    logger.info("Jetson control enabled")

    # 8. Run perception + navigation loop
    await self.run_navigation_loop(mission)

    # 9. Return to launch
    self.guidance.return_to_launch()

    # 10. Stop position broadcast
    await self.aerosync_client.stop_position_broadcast()

    # 11. Notify AeroSync of completion
    summary = self.mission_manager.get_summary()
    await self.aerosync_client.complete_job(job_id, summary)

    # 12. Disconnect WebSocket
    await self.sio.disconnect()

    logger.info("Mission complete")
```

5. **Navigation Loop:**

```python
async def run_navigation_loop(self, mission: Mission):
    """Main perception + navigation loop."""

    rate = 100  # Hz
    dt = 1.0 / rate

    while True:
        loop_start = time.time()

        # 1. Get fused poles from perception
        fused_poles = self.fusion.get_fused_poles()

        # 2. Check stop conditions
        should_stop, reason = self.mission_manager.should_stop(
            poles_inspected=len(fused_poles),
            battery_percent=self.guidance.get_battery(),
            distance_from_home=self.guidance.get_distance_to_home()
        )

        if should_stop:
            logger.info(f"Mission stopping: {reason}")
            break

        # 3. Report new poles to AeroSync
        for pole in fused_poles:
            if pole.is_new:
                await self.aerosync_client.report_pole(mission.job_id, pole)

        # 4. Compute navigation command
        nav_cmd = self.line_follower.compute_command(fused_poles)

        # 5. Smooth control
        vel_cmd = self.smooth_controller.update(nav_cmd)

        # 6. Send velocity command to FC
        self.guidance.send_velocity(
            vel_cmd.vx,
            vel_cmd.vy,
            vel_cmd.vz,
            vel_cmd.yaw_rate
        )

        # 7. Check if capture triggered
        if nav_cmd.capture_triggered:
            # Schedule photo capture
            self.capture_system.schedule_capture(
                pole_id=nav_cmd.target_pole_id,
                pole_position=nav_cmd.target_pole_position,
                distance=nav_cmd.distance_to_pole
            )

        # 8. Maintain loop rate
        elapsed = time.time() - loop_start
        if elapsed < dt:
            await asyncio.sleep(dt - elapsed)
```

### Priority 5: Gremsy Gimbal Integration (1-2 days, Optional)

**Goal:** Point camera at poles during inspection.

**Create File:** `perception_nav/src/gimbal_control.py`

**Requirements:**

1. **Determine Gremsy Protocol:**
   - Check Gremsy documentation
   - Likely MAVLink or serial protocol

2. **Gimbal Control:**

```python
class GremsyGimbal:
    def __init__(self, connection_string: str):
        # Initialize gimbal connection
        pass

    def point_at_position(self, target_ned: np.ndarray, drone_ned: np.ndarray):
        """Calculate and send gimbal angles to point at target."""

        # Calculate relative position
        dx = target_ned[0] - drone_ned[0]
        dy = target_ned[1] - drone_ned[1]
        dz = target_ned[2] - drone_ned[2]

        # Calculate gimbal angles
        distance_xy = np.sqrt(dx**2 + dy**2)
        pitch = np.arctan2(dz, distance_xy)  # Pitch down to look at pole
        yaw = np.arctan2(dy, dx)

        # Convert to degrees
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)

        # Send gimbal command
        self.send_command(pitch_deg, yaw_deg)

    def send_command(self, pitch: float, yaw: float):
        """Send gimbal command via MAVLink or serial."""
        # Implementation depends on Gremsy protocol
        pass

    def trigger_photo(self):
        """Trigger photo capture via gimbal."""
        # Implementation depends on Gremsy protocol
        pass
```

**Alternative if No Gimbal:**
- Use fixed camera mount (straight down or 45° angle)
- Trigger photo via software command to camera

---

## Testing

### Unit Tests

**Perception:**
```bash
pytest tests/test_pole_detector.py
pytest tests/test_fusion.py
```

**Navigation:**
```bash
pytest tests/test_line_follower.py
pytest tests/test_controller.py
```

### SITL Testing

**Install ArduPilot SITL:**
```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./waf configure --board sitl
./waf copter
```

**Run SITL:**
```bash
sim_vehicle.py -v ArduCopter --console --map
```

**Test Flight Commands:**
```python
# Connect to SITL
guidance = Guidance(connection='udp:127.0.0.1:14550')

# Test takeoff
guidance.takeoff(10.0)

# Test velocity control
guidance.send_velocity(1.0, 0, 0, 0)  # Forward at 1 m/s

# Test RTL
guidance.return_to_launch()
```

### Hardware-in-Loop Testing

**Without Props:**
1. Connect Jetson to CubeOrange+ via USB
2. Power on FC (no props!)
3. Verify MAVLink communication
4. Test all commands
5. Monitor telemetry

### Integration Testing

**Fake Mission (No Flight):**
```bash
# Connect to AeroSync
python run_mission.py --job-id 1 --no-flight
```

This will:
- Fetch mission from AeroSync
- Parse mission
- Connect WebSocket
- Send fake position updates
- Detect poles (if LiDAR connected)
- Report poles to AeroSync
- Skip actual flight commands

---

## Configuration

### AeroSync Connection

Edit `config/settings.yaml`:
```yaml
aerosync:
  base_url: "https://aerosync.example.com"
  api_key: "your-drone-api-key"
  websocket_enabled: true
  upload_photos: true
  position_update_rate_hz: 1.0
  retry_max_attempts: 3
  timeout_seconds: 30.0
```

### Flight Controller

```yaml
guidance:
  connection: '/dev/ttyACM0'  # USB connection
  # Or for SITL: 'udp:127.0.0.1:14550'
  baud_rate: 57600
  max_forward_velocity: 2.0
  max_lateral_velocity: 1.0
  heartbeat_timeout: 2.0
```

### Stop Conditions

```yaml
stop_conditions:
  max_poles: 0                     # 0 = unlimited
  max_time_minutes: 0              # 0 = unlimited
  min_battery_percent: 20.0        # RTL when battery hits this
  coverage_threshold: 0.95         # Coverage % to complete (0-1)
  max_distance_from_home: 0        # 0 = unlimited, meters
```

---

## Deployment Checklist

### Hardware Setup
- [ ] Jetson Orin/Xavier with Ubuntu 20.04
- [ ] Aeva Atlas LiDAR connected and configured
- [ ] USB camera connected
- [ ] CubeOrange+ connected via USB
- [ ] Jetson GPS module connected to USB
- [ ] Starlink router configured
- [ ] All sensors powered

### Software Installation
```bash
# Install dependencies
pip install -r requirements.txt

# Install ROS (if using Aeva ROS bridge)
# Follow ROS Noetic installation

# Verify hardware
ls /dev/ttyACM0  # Flight controller
ls /dev/ttyUSB1  # GPS
ls /dev/video0   # Camera

# Test LiDAR
rostopic echo /aeva_0/points
```

### Configuration
- [ ] Update `config/settings.yaml` with AeroSync URL and API key
- [ ] Calibrate camera intrinsics
- [ ] Calibrate camera-LiDAR extrinsics
- [ ] Configure ArduPilot parameters (failsafes, geofence)
- [ ] Test all sensors individually

### Pre-Flight Checklist
- [ ] GPS fix (> 4 satellites)
- [ ] LiDAR publishing data
- [ ] Camera streaming
- [ ] MAVLink connection to FC
- [ ] Network connection to AeroSync
- [ ] Battery charged
- [ ] Props secured
- [ ] Geofence configured
- [ ] Home position set

---

## API Client Usage

```python
from perception_nav.src.aerosync_client import AeroSyncClient
from perception_nav.src.mission_manager import MissionManager

# Initialize client
client = AeroSyncClient(
    base_url="https://aerosync.example.com",
    api_key="your-api-key"
)

# Fetch mission
job = await client.fetch_job(job_id=1)

# Start mission
await client.start_job(job_id=1)

# Connect WebSocket
await client.connect_websocket(job_id=1)
await client.start_position_broadcast(job_id=1)

# Report pole
from perception_nav.src.pole_detector import DetectedPole
pole = DetectedPole(...)
await client.report_pole(job_id=1, pole=pole)

# Upload photo
image = cv2.imread('photo.jpg')
await client.upload_photo(job_id=1, pole_id=456, image=image, metadata={})

# Complete mission
summary = {'poles_inspected': 47, 'duration_minutes': 28.5}
await client.complete_job(job_id=1, summary=summary)
```

---

## Time Estimates

| Task | Time | Priority |
|------|------|----------|
| Mission receiver + parser | 1-2 days | P1 |
| WebSocket client | 1 day | P1 |
| Photo S3 upload | 1-2 days | P1 |
| Flight control integration | 2-3 days | P1 |
| SITL testing | 1 day | P1 |
| Gremsy gimbal (optional) | 1-2 days | P2 |
| Hardware integration | 1 day | P1 |
| Flight testing | 2-3 days | P1 |
| **Total (MVP)** | **10-14 days** | |

---

## Support & References

**Repository:** `/Users/nolannachbar/Testing:Dev/JetsonLidar`

**Key Files:**
- Pole Detection: `perception_nav/src/pole_detector.py`
- Sensor Fusion: `perception_nav/src/fusion.py`
- Navigation: `perception_nav/src/line_follower.py`, `smooth_controller.py`
- Flight Control: `perception_nav/src/guidance.py`
- Mission Management: `perception_nav/src/mission_manager.py`
- AeroSync Client: `perception_nav/src/aerosync_client.py`
- Configuration: `config/settings.yaml`

**AeroSync Integration:** See `AEROSYNC_SPEC.md` for web platform implementation.

**ArduPilot Documentation:** https://ardupilot.org/copter/
**Aeva Atlas Documentation:** http://10.5.50.100/ (when connected to LiDAR)
