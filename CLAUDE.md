# Perception-Nav MVP - Fusion System

## Overview

Autonomous pole-following navigation system for drone operation. Fuses Aeva Atlas FMCW LiDAR with camera-based YOLO detection for following a single line of utility poles (H-frame, transmission poles).

## Hardware

- **Platform**: NVIDIA Jetson (Linux 5.10.192-tegra)
- **LiDAR**: Aeva Atlas (10.5.50.100) - FMCW with velocity data
- **Camera**: USB camera (device 0)
- **Flight Controller**: CubeOrange+ (ArduPilot) via MAVLink

## Current State

`start_fusion.sh` launches:
1. ROS Noetic core
2. `aeva_simple_bridge` - publishes to `/aeva_0/points` (PointCloud2)
3. `fusion_ros.py` - Full perception + navigation system

**Operational modes:**
- Visualization only (default)
- Navigation enabled (press 'n', requires FCU connection)
- FCU control only active in GUIDED mode (pilot controls via RC)

---

## MVP Components

### 1. Pole Detection (LiDAR) - DONE

`perception_nav/src/pole_detector.py`

- **Ground plane removal**: RANSAC with temporal smoothing
- **Clustering**: DBSCAN on XY plane (eps=0.5m, min_samples=5)
- **Cylinder fitting**: PCA for principal axis + percentile-based radius
- **Validation**: Height, verticality, radius bounds, aspect ratio
- **Tracking**: Kalman filter with stable IDs across frames
- **Moving object rejection**: FMCW velocity filtering

**Output**: `PoleDetection(id, x, y, z, radius, confidence, velocity, height, ...)`

### 2. Pole Detection (Camera) - DONE

Using YOLOv5 in `fusion_ros.py`

- **Model**: Custom trained on utility poles (Wyoming dataset)
- **Classes**: Metal h-frame, Metal tree pole, Single pole, Wood distribution, h-frame
- **Inference**: GPU-accelerated on Jetson
- **Confidence threshold**: Adjustable via slider (default 30%)

**Output**: Bounding boxes with class and confidence

### 3. Sensor Fusion - DONE

`perception_nav/src/fusion.py`

- **Projection**: LiDAR poles projected to camera image plane
- **Association**: Hungarian algorithm for optimal LiDAR-camera matching
- **Weighted blend**: `fused_conf = 0.6 * lidar + 0.4 * camera + boost`
- **Temporal hold**: Camera match retained for 5 frames
- **Blacklist**: Suppress persistent false positives by location
- **Output gating**: Only output poles with `fused_conf > 0.3`

**Output**: `FusedPole(id, x, y, z, fused_confidence, camera_matched, ...)`

### 4. Line Following - DONE

`perception_nav/src/line_follower.py`

Follows a **single line** of utility poles (not a corridor between two rows).

- **Line fitting**: PCA-based line fit through pole positions
- **Lateral offset**: Configurable distance to fly from pole line (default 10m)
- **Side selection**: Fly left or right of poles
- **Lateral error**: Distance from desired offset line
- **Heading error**: Angle to align with pole line direction
- **Look-ahead**: Configurable look-ahead distance (default 20m)
- **Next-pole tracking**: For capture triggering

**Output**: `LineState(lateral_error, heading_error, look_ahead_x/y, confidence, ...)`

### 5. Guidance Output - DONE

`perception_nav/src/guidance.py`

MAVLink communication with Cube Blue flight controller.

- **Connection**: USB (`/dev/ttyACM0`) or Serial (`/dev/ttyTHS1`)
- **Protocol**: MAVLink via pymavlink
- **Commands**: Body-frame velocity (SET_POSITION_TARGET_LOCAL_NED)
- **Controller**: PD + pure-pursuit for lateral/heading correction
- **Rate limiting**: Acceleration-limited smooth transitions
- **Flight mode safety**: Only sends commands when FCU is in GUIDED mode

**State machine**:
```
IDLE → SEARCH → FOLLOW → RESCAN → ABORT
              ↑________|
```

- `IDLE`: Waiting for activation
- `SEARCH`: Rotate to find poles (30s timeout)
- `FOLLOW`: Normal line tracking
- `RESCAN`: Lost poles, slow search (5s timeout)
- `ABORT`: Failsafe triggered, RTL

### 6. Smooth Controller - DONE

`perception_nav/src/smooth_controller.py`

Tesla-style high-performance path following.

- **100Hz control loop**: Separate thread from perception (10-30Hz)
- **Cubic spline path fitting**: Smooth curves through pole positions
- **Dynamic look-ahead**: `look_ahead = base + speed * gain`
- **Jerk-limited velocity profiling**: Smooth acceleration/deceleration
- **MPC-lite predictive control**: Anticipates path curvature
- **Curvature-based speed**: Slows down in curves

### 7. Mission Planning - DONE

`perception_nav/src/mission_planner.py`

Parses AeroSync job data and generates flight waypoints.

- **Mission Parser**: Converts AeroSync JSON to Mission dataclass
- **Waypoint Generation**: Three search patterns supported
  - `line_follow`: Single start waypoint, Jetson follows poles autonomously
  - `lawnmower`: Parallel north-south sweeps across polygon (starts from closest corner)
  - `spiral`: Outward spiral from polygon center
- **Closest Corner Start**: Automatically finds closest corner of search area to home position
- **Mission Validation**: Checks altitude, speed, polygon closure, area size
- **GeoJSON Support**: Parses polygon coordinates from GeoJSON format
- **Coordinate Utils**: Haversine distance, meters-to-degrees conversions

**Usage:**
```python
from perception_nav.src.mission_planner import parse_mission, generate_waypoints, validate_mission, compute_entry_point

mission = parse_mission(job_data)
valid, error = validate_mission(mission)
entry_point = compute_entry_point(mission)  # Closest corner to home
waypoints = generate_waypoints(mission)
```

**Output**: `Mission`, `Waypoint`, `SearchConfig`, `InspectionConfig` dataclasses

### 8. Flight Control - DONE

`perception_nav/src/guidance.py` - High-level flight commands

- **takeoff(altitude)**: Arms, sets GUIDED mode, takes off to altitude
- **navigate_to_waypoint(lat, lon, alt)**: GPS waypoint navigation with arrival detection
- **return_to_launch()**: Commands RTL mode
- **land()**: Commands LAND mode
- **hold_position()**: Zero-velocity hold

**GPS Telemetry**: Now tracks GPS position, altitude, heading, and battery from FCU

### 9. Mission Executor - DONE

`perception_nav/src/mission_executor.py`

Orchestrates complete autonomous missions end-to-end.

**Execution Flow:**
1. Fetch mission from AeroSync
2. Parse and validate mission
3. Takeoff to mission altitude
4. Navigate to closest corner of search area
5. Execute search pattern (lawnmower → line-follow when poles found)
6. Report poles to AeroSync
7. Capture photos and upload
8. Monitor stop conditions
9. Return to launch
10. Land

**Usage:**
```python
from perception_nav.src.mission_executor import MissionExecutor

executor = MissionExecutor(config_path='config/settings.yaml')
executor.initialize()
executor.connect_fcu('/dev/ttyACM0')
await executor.run_mission('job-uuid')
```

**States**: IDLE → FETCHING_MISSION → VALIDATING → TAKEOFF → TRANSIT_TO_AREA → SEARCHING → LINE_FOLLOWING → RETURNING → LANDING → COMPLETED

### 10. Capture Scheduler - PARTIAL

`perception_nav/src/capture_system.py`

Infrastructure implemented:
- Ring buffer for frame history
- Async data bundler (non-blocking)
- Capture event scheduling
- Obstacle detection integration

Needs verification:
- Integration with main loop
- ROI gating

### 11. Data Bundling - PARTIAL

Infrastructure in `capture_system.py`:
- Async file I/O worker thread
- Metadata JSON with detections
- MD5 checksums
- Directory structure per capture

### 12. S3 Photo Upload - DONE

`perception_nav/src/aerosync_client.py` - S3 presigned URL upload flow

- **get_photo_upload_url()**: Request presigned URL from AeroSync
- **upload_to_s3()**: Direct PUT to S3 with presigned URL
- **confirm_photo_upload()**: Confirm upload completion
- **upload_photo_s3()**: Complete flow with retry logic

**Two upload options:**
1. Direct upload via `upload_photo()` (multipart to AeroSync)
2. S3 presigned URL via `upload_photo_s3()` (3-step flow)

### 13. Failsafes - DONE

| Condition | Response | Status |
|-----------|----------|--------|
| Confidence drop | RESCAN state | Done |
| Heartbeat loss | ABORT + RTL | Done |
| Perception stale | Hold position | Done |
| Poles too far apart | RESCAN state | Done |
| GUIDED mode exit | Stop commands | Done |
| Obstacle detected | OBSTACLE_STOP + alert | Done |

**Obstacle Detection Failsafe:**
- Uses LiDAR corridor check (15m ahead, 4m wide, -1m to +2m altitude)
- Emergency stop at 3m (OBSTACLE_STOP state)
- Slows down from 8m
- Resumes at 5m clear distance
- Visual feedback in status bar (red = STOP, orange = slow, yellow = detected)

---

## File Structure

```
JetsonLidar/
├── fusion_ros.py              # Main application (visualization + nav)
├── start_fusion.sh            # Launch script
├── config/
│   └── settings.yaml          # All configuration parameters
├── perception_nav/
│   └── src/
│       ├── pole_detector.py   # LiDAR pole extraction + tracking
│       ├── fusion.py          # LiDAR + camera fusion
│       ├── line_follower.py   # Single-line pole following
│       ├── guidance.py        # MAVLink commands + flight control
│       ├── smooth_controller.py # 100Hz Tesla-style control
│       ├── mission_planner.py # Mission parser + waypoint generation
│       ├── mission_manager.py # Mission state + stop conditions
│       ├── mission_executor.py # Full autonomous mission orchestration
│       ├── aerosync_client.py # AeroSync REST + WebSocket + S3 upload
│       ├── capture_system.py  # Photo capture + data bundling
│       └── gps_reader.py      # GPS module interface
├── yolov5/                    # YOLO model and inference
└── build/
    └── aeva_simple_bridge     # LiDAR ROS bridge
```

---

## Configuration

All parameters in `config/settings.yaml`:

- **camera**: Device, resolution
- **lidar**: IP, port, ROS topic, range limits
- **yolo**: Model path, confidence, GPU toggle
- **pole_detector**: RANSAC, clustering, cylinder validation, tracking, velocity filter
- **sensor_fusion**: Camera intrinsics, extrinsics, association, confidence weights
- **line_follower**: Lateral offset, look-ahead, pole spacing
- **guidance**: Connection, velocity limits, controller gains, state machine timeouts
- **smooth_controller**: Control rate, look-ahead, velocity/acceleration limits, gains

---

## Controls

| Key | Action |
|-----|--------|
| `n` | Toggle navigation ON/OFF |
| `c` | Connect to FCU (without enabling nav) |
| `b` | Toggle training data recording |
| `d` | Toggle camera detection |
| `l` | Toggle LiDAR detection |
| `s` | Toggle sensor fusion |
| `t` | Reset trackers |
| `r` | Reset view (overhead) |
| `f` | Front view |
| `p` | Change point size |
| `+/-` | Zoom in/out |
| `q` | Quit |

**Herelink Controller:**
| Button | Action |
|--------|--------|
| C button | Toggle training data recording |

Mouse drag on LiDAR panel rotates view.

### Training Data Recording

Press `b` or Herelink C button to start/stop recording:

**Output structure:**
```
flight_captures/<timestamp>/
├── flight_<timestamp>.bag    # Rosbag for replay
├── images/
│   ├── frame_000000.jpg     # Camera frames (~10fps)
│   └── ...
└── lidar/
    ├── frame_000000.npy     # Point clouds as numpy
    ├── frame_000000.json    # Metadata + detections
    └── ...
```

**What's recorded:**
- **Rosbag**: `/aeva_0/points` + `/camera/image_raw` with synchronized timestamps
- **Images**: JPG frames at ~10fps (for YOLO training)
- **Metadata**: JSON with YOLO detections and LiDAR pole positions

**To replay in the office:**
```bash
# Terminal 1: Start ROS and replay bag
roscore &
rosbag play flight_captures/<timestamp>/*.bag --loop

# Terminal 2: Run fusion in REPLAY MODE
python3 fusion_ros.py --replay
```

**Replay mode options:**
```bash
# Full options
python3 fusion_ros.py --replay \
    --lidar-topic /aeva_0/points \
    --camera-topic /camera/image_raw

# Check what's in a bag file
rosbag info flight_captures/<timestamp>/*.bag

# Play at different speeds
rosbag play *.bag --rate 0.5   # Half speed
rosbag play *.bag --rate 2.0   # Double speed
```

---

## Quick Start

```bash
# Launch full system
./start_fusion.sh

# Connect to FCU and enable navigation
# 1. Press 'c' to connect to Cube Blue
# 2. Press 'n' to enable navigation
# 3. Flip RC transmitter to GUIDED mode
# 4. System will follow pole line automatically
# 5. Flip RC away from GUIDED to take manual control
```

---

## Dependencies

- ROS Noetic
- Python 3.8+
- OpenCV (cv2)
- NumPy
- SciPy (spline fitting, Hungarian algorithm)
- scikit-learn (DBSCAN clustering)
- PyTorch (YOLO inference)
- pymavlink (FCU communication)
- PyYAML

Install:
```bash
pip install numpy scipy scikit-learn pymavlink pyyaml
# PyTorch: follow Jetson-specific instructions
```

---

## Notes

- Aeva Atlas provides per-point velocity (FMCW) - used for moving object rejection
- Coordinate frame: X=forward, Y=left, Z=up (Aeva/body convention)
- Camera-LiDAR extrinsics in settings.yaml (may need calibration)
- Navigation only active when pilot selects GUIDED mode on RC
- 100Hz smooth controller runs in separate thread from 10-30Hz perception

---

## Testing Session Notes (Jan 2026)

### FCU Connection

**Port**: CubeOrange+ is on `/dev/ttyACM0` (main MAVLink interface)
- `if00` -> `/dev/ttyACM0` - **Main MAVLink port**
- `if02` -> `/dev/ttyACM1` - Secondary (SLCAN/CAN bus)

**Permission Issue**: Running `./start_fusion.sh` without sudo fails with:
```
[Errno 13] Permission denied: '/dev/ttyACM0'
```

**Fix options**:
1. One-time: `sudo chmod 666 /dev/ttyACM0`
2. Permanent: `sudo usermod -aG dialout $USER` then logout/login

**Testing without sudo workaround**:
```bash
sudo chmod 666 /dev/ttyACM0
./start_fusion.sh
```

### Current System Status

All components initialize successfully:
- ROS Noetic core starts
- Aeva bridge publishes to `/aeva_0/points`
- Camera opens (device 0)
- YOLO model loads on GPU
- Pole detector ready
- Line follower ready
- Smooth controller ready (100Hz thread)
- Capture system ready

**To test FCU connection**:
1. Run `./start_fusion.sh`
2. Press 'c' to connect to FCU
3. Should see "FCU: ARMED" or "FCU: DISARMED" in status bar

**Standalone FCU test** (works with sudo):
```bash
sudo python perception_nav/src/guidance.py --port /dev/ttyACM0 --baud 57600
```

### Hardware Setup

- Drone not assembled (testing software only)
- CubeOrange+ FCU connected via USB to Jetson
- Camera connected (USB device 0)
- Aeva Atlas LiDAR on network (10.5.50.100)
