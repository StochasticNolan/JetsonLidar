# AeroSync Web Application Requirements

## Overview

AeroSync is the web application that manages autonomous drone pole inspection missions. It communicates with drones running the AeroVision perception system (JetsonLidar codebase) to:

1. Create and configure inspection jobs
2. Receive real-time drone telemetry and position
3. Display found poles and photos as they're discovered
4. Track mission progress and status

The drone runs on NVIDIA Jetson with:
- Aeva Atlas FMCW LiDAR for pole detection
- Camera with YOLO for visual confirmation
- CubeOrange+ flight controller (ArduPilot)
- Starlink for connectivity

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        AEROSYNC                                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐ │
│  │   React     │  │   FastAPI   │  │      PostgreSQL         │ │
│  │   Frontend  │◄─┤   Backend   │◄─┤   + PostGIS (geo)       │ │
│  └─────────────┘  └──────┬──────┘  └─────────────────────────┘ │
│                          │                                       │
│                    ┌─────┴─────┐                                │
│                    │    S3     │  (photo storage)               │
│                    └───────────┘                                │
└─────────────────────────┬───────────────────────────────────────┘
                          │ REST API + WebSocket
                          │ (via Starlink)
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                     DRONE (Jetson)                               │
│  - Fetches job config via GET /api/jobs/{id}                    │
│  - Reports poles via POST /api/jobs/{id}/poles                  │
│  - Uploads photos via POST .../photo (or S3 presigned)          │
│  - Streams position via WebSocket                                │
│  - Sends status updates via PUT /api/jobs/{id}/status           │
└─────────────────────────────────────────────────────────────────┘
```

---

## Database Schema

### Tables

#### `jobs`
```sql
CREATE TABLE jobs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(255) NOT NULL,
    description TEXT,
    state VARCHAR(50) DEFAULT 'pending',  -- pending, running, paused, completed, aborted

    -- Search area (PostGIS polygon)
    search_area GEOMETRY(POLYGON, 4326),

    -- Key points
    start_point GEOMETRY(POINT, 4326),
    home_point GEOMETRY(POINT, 4326),

    -- Configuration (JSONB for flexibility)
    stop_conditions JSONB DEFAULT '{}',
    search_config JSONB DEFAULT '{}',
    inspection_config JSONB DEFAULT '{}',

    -- Timing
    created_at TIMESTAMP DEFAULT NOW(),
    started_at TIMESTAMP,
    completed_at TIMESTAMP,

    -- Results summary
    poles_found INTEGER DEFAULT 0,
    coverage_percent FLOAT DEFAULT 0,

    -- Foreign keys
    created_by UUID REFERENCES users(id),
    drone_id UUID REFERENCES drones(id)
);

-- Index for geo queries
CREATE INDEX idx_jobs_search_area ON jobs USING GIST(search_area);
```

#### `poles`
```sql
CREATE TABLE poles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    job_id UUID REFERENCES jobs(id) ON DELETE CASCADE,

    -- Position
    location GEOMETRY(POINT, 4326),  -- GPS coordinates
    local_x FLOAT,  -- Local NED X (for reference)
    local_y FLOAT,
    local_z FLOAT,
    altitude_msl FLOAT,
    altitude_agl FLOAT,

    -- Detection info
    pole_type VARCHAR(100),  -- 'h_frame', 'single_pole', 'wood_distribution', etc.
    confidence FLOAT,
    lidar_confidence FLOAT,
    camera_confidence FLOAT,
    height_estimate FLOAT,
    radius_estimate FLOAT,

    -- Metadata
    detected_at TIMESTAMP DEFAULT NOW(),
    metadata JSONB DEFAULT '{}',

    -- Indexes
    CONSTRAINT unique_pole_location UNIQUE (job_id, location)
);

CREATE INDEX idx_poles_job ON poles(job_id);
CREATE INDEX idx_poles_location ON poles USING GIST(location);
```

#### `photos`
```sql
CREATE TABLE photos (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    pole_id UUID REFERENCES poles(id) ON DELETE CASCADE,
    job_id UUID REFERENCES jobs(id) ON DELETE CASCADE,

    -- Storage
    s3_key VARCHAR(500),
    s3_bucket VARCHAR(100),
    url VARCHAR(1000),  -- CloudFront URL
    thumbnail_url VARCHAR(1000),

    -- Metadata
    file_size INTEGER,
    width INTEGER,
    height INTEGER,
    captured_at TIMESTAMP,
    uploaded_at TIMESTAMP DEFAULT NOW(),

    -- Drone position when captured
    drone_lat FLOAT,
    drone_lon FLOAT,
    drone_alt FLOAT,
    drone_heading FLOAT
);

CREATE INDEX idx_photos_pole ON photos(pole_id);
CREATE INDEX idx_photos_job ON photos(job_id);
```

#### `telemetry` (time-series, consider TimescaleDB)
```sql
CREATE TABLE telemetry (
    id BIGSERIAL PRIMARY KEY,
    job_id UUID REFERENCES jobs(id),
    drone_id UUID REFERENCES drones(id),

    -- Position
    latitude FLOAT,
    longitude FLOAT,
    altitude_msl FLOAT,
    altitude_agl FLOAT,

    -- Attitude
    heading FLOAT,
    pitch FLOAT,
    roll FLOAT,

    -- Status
    battery_percent FLOAT,
    ground_speed FLOAT,
    guidance_state VARCHAR(50),

    timestamp TIMESTAMP DEFAULT NOW()
);

-- Partition by time if using TimescaleDB
-- SELECT create_hypertable('telemetry', 'timestamp');

CREATE INDEX idx_telemetry_job_time ON telemetry(job_id, timestamp DESC);
```

#### `drones`
```sql
CREATE TABLE drones (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    name VARCHAR(100),
    serial_number VARCHAR(100) UNIQUE,

    -- Current status
    status VARCHAR(50) DEFAULT 'offline',  -- offline, idle, flying, error
    last_seen TIMESTAMP,
    current_job_id UUID REFERENCES jobs(id),

    -- Position (last known)
    last_lat FLOAT,
    last_lon FLOAT,
    last_alt FLOAT,
    last_battery FLOAT,

    -- Config
    home_location GEOMETRY(POINT, 4326),
    config JSONB DEFAULT '{}'
);
```

---

## REST API Endpoints

### Jobs

#### `POST /api/jobs` - Create Job
**Request:**
```json
{
  "name": "Highway 287 Survey",
  "description": "Inspect poles along Highway 287 mile markers 10-25",
  "search_area": {
    "type": "Polygon",
    "coordinates": [[[lon1, lat1], [lon2, lat2], [lon3, lat3], [lon4, lat4], [lon1, lat1]]]
  },
  "start_point": {"lat": 41.123, "lon": -105.456},
  "home_point": {"lat": 41.120, "lon": -105.450},
  "stop_conditions": {
    "max_poles": 50,
    "max_time_minutes": 30,
    "min_battery_percent": 25,
    "coverage_threshold": 0.9
  },
  "search_config": {
    "pattern": "lawnmower",
    "sweep_spacing": 30,
    "flight_altitude": 30
  },
  "inspection_config": {
    "min_confidence": 0.6,
    "camera_required": true,
    "capture_distance": 10,
    "photos_per_pole": 1
  },
  "drone_id": "uuid-of-assigned-drone"
}
```

**Response:** `201 Created`
```json
{
  "job_id": "550e8400-e29b-41d4-a716-446655440000",
  "name": "Highway 287 Survey",
  "state": "pending",
  "created_at": "2026-01-16T14:30:00Z"
}
```

#### `GET /api/jobs/{job_id}` - Get Job (Drone calls this)
**Response:** `200 OK`
```json
{
  "job_id": "550e8400-e29b-41d4-a716-446655440000",
  "name": "Highway 287 Survey",
  "state": "pending",
  "search_area": {
    "type": "Polygon",
    "coordinates": [[[...]]]
  },
  "start_point": {"lat": 41.123, "lon": -105.456, "alt": 30},
  "home_point": {"lat": 41.120, "lon": -105.450},
  "stop_conditions": {
    "max_poles": 50,
    "max_time_minutes": 30,
    "min_battery_percent": 25,
    "coverage_threshold": 0.9
  },
  "search_config": {
    "pattern": "lawnmower",
    "sweep_spacing": 30,
    "flight_altitude": 30
  },
  "inspection_config": {
    "min_confidence": 0.6,
    "camera_required": true,
    "capture_distance": 10,
    "photos_per_pole": 1
  },
  "previously_inspected": [
    {"id": "pole-uuid-1", "lat": 41.125, "lon": -105.460},
    {"id": "pole-uuid-2", "lat": 41.130, "lon": -105.465}
  ]
}
```

#### `POST /api/jobs/{job_id}/start` - Drone Acknowledges Start
**Request:** (empty body or optional metadata)
```json
{
  "drone_battery": 98.5,
  "gps_fix": true
}
```

**Response:** `200 OK`
```json
{
  "status": "started",
  "started_at": "2026-01-16T14:35:00Z"
}
```

**Side effects:**
- Set `jobs.state = 'running'`
- Set `jobs.started_at = NOW()`
- Broadcast to WebSocket subscribers

#### `PUT /api/jobs/{job_id}/status` - Update Status (Drone calls periodically)
**Request:**
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
    "lat": 41.128,
    "lon": -105.462,
    "alt": 30.5
  }
}
```

**Response:** `200 OK`
```json
{"status": "updated"}
```

**Side effects:**
- Update `jobs.poles_found`, `jobs.coverage_percent`
- Broadcast status to WebSocket
- Insert into `telemetry` table

#### `POST /api/jobs/{job_id}/complete` - Mission Complete
**Request:**
```json
{
  "stop_reason": "coverage_complete",
  "summary": {
    "poles_inspected": 47,
    "poles_skipped_duplicate": 3,
    "poles_skipped_low_confidence": 5,
    "duration_minutes": 28.5,
    "final_battery_percent": 32.1,
    "final_coverage_percent": 0.95
  }
}
```

**Response:** `200 OK`

**Side effects:**
- Set `jobs.state = 'completed'`
- Set `jobs.completed_at = NOW()`
- Broadcast completion to WebSocket

#### `POST /api/jobs/{job_id}/abort` - Abort Mission (from web UI)
**Response:** `200 OK`

**Side effects:**
- Broadcast abort command via WebSocket to drone

---

### Poles

#### `POST /api/jobs/{job_id}/poles` - Report Found Pole (Drone calls this)
**Request:**
```json
{
  "pole_id": "550e8400-e29b-41d4-a716-446655440001",
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

**Response:** `201 Created`
```json
{
  "pole_id": "550e8400-e29b-41d4-a716-446655440001",
  "status": "created"
}
```

**Side effects:**
- Insert into `poles` table
- Increment `jobs.poles_found`
- Broadcast `pole_found` event via WebSocket

#### `GET /api/jobs/{job_id}/poles` - List All Poles for Job
**Response:**
```json
{
  "poles": [
    {
      "pole_id": "...",
      "latitude": 41.128,
      "longitude": -105.461,
      "pole_type": "h_frame",
      "confidence": 0.87,
      "detected_at": "2026-01-16T14:40:23Z",
      "photos": [
        {"id": "photo-uuid", "thumbnail_url": "https://..."}
      ]
    }
  ],
  "total": 47
}
```

---

### Photos

#### `POST /api/jobs/{job_id}/poles/{pole_id}/photo` - Upload Photo

**Option A: Direct upload (simple)**
- Content-Type: `multipart/form-data`
- Body: photo file + metadata

**Option B: S3 Presigned URL (faster, recommended)**

First, get presigned URL:
```
GET /api/jobs/{job_id}/poles/{pole_id}/photo/upload-url
```
**Response:**
```json
{
  "upload_url": "https://s3.amazonaws.com/bucket/key?X-Amz-Signature=...",
  "photo_id": "new-photo-uuid",
  "expires_in": 300
}
```

Drone uploads directly to S3, then confirms:
```
POST /api/jobs/{job_id}/poles/{pole_id}/photo/confirm
{
  "photo_id": "new-photo-uuid",
  "s3_key": "jobs/job-id/poles/pole-id/photo.jpg"
}
```

**Side effects:**
- Lambda trigger on S3 generates thumbnail
- Insert into `photos` table
- Broadcast `photo_uploaded` event via WebSocket

---

### WebSocket

#### Connection
```
WSS /api/jobs/{job_id}/live
Headers:
  Authorization: Bearer <token>
```

#### Messages: Drone → Server
```json
// Position update (1 Hz)
{
  "type": "position",
  "lat": 41.128,
  "lon": -105.462,
  "alt": 30.5,
  "heading": 45.2,
  "speed": 3.5,
  "battery": 85.2,
  "timestamp": 1705412423.456
}

// Status update
{
  "type": "status",
  "guidance_state": "INSPECT_POLE",
  "poles_found": 12,
  "coverage": 0.35
}

// Pole found (real-time notification)
{
  "type": "pole_found",
  "pole_id": "uuid",
  "lat": 41.129,
  "lon": -105.463,
  "pole_type": "h_frame",
  "confidence": 0.87
}

// Photo uploaded
{
  "type": "photo_ready",
  "pole_id": "uuid",
  "photo_url": "https://cdn.../photo.jpg",
  "thumbnail_url": "https://cdn.../thumb.jpg"
}
```

#### Messages: Server → Drone
```json
// Pause mission
{
  "type": "pause"
}

// Resume mission
{
  "type": "resume"
}

// Abort mission (RTL)
{
  "type": "abort"
}

// Update stop conditions mid-flight
{
  "type": "update_config",
  "stop_conditions": {
    "max_poles": 30  // Operator reduced target
  }
}
```

#### Messages: Server → Web Clients (broadcast)
```json
// Drone position (1 Hz)
{
  "type": "drone_position",
  "lat": 41.128,
  "lon": -105.462,
  "alt": 30.5,
  "heading": 45.2,
  "battery": 85.2
}

// New pole found
{
  "type": "pole_found",
  "pole": {
    "id": "uuid",
    "lat": 41.129,
    "lon": -105.463,
    "pole_type": "h_frame"
  }
}

// Photo available
{
  "type": "photo_ready",
  "pole_id": "uuid",
  "thumbnail_url": "https://..."
}

// Mission status
{
  "type": "status",
  "poles_found": 12,
  "coverage": 0.35,
  "elapsed_minutes": 8.5
}

// Mission complete
{
  "type": "mission_complete",
  "reason": "coverage_complete",
  "summary": {...}
}
```

---

## Frontend Components

### Pages

#### 1. Job List (`/jobs`)
- Table of all jobs with status badges
- Filter by state (pending, running, completed)
- Search by name
- "New Job" button

#### 2. Create Job (`/jobs/new`)
```
┌────────────────────────────────────────────────────────────────┐
│  Create New Inspection Job                                      │
├────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Job Name: [________________________]                          │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │                                                          │  │
│  │                    INTERACTIVE MAP                       │  │
│  │                                                          │  │
│  │   [Draw polygon tool]  [Marker tools]                   │  │
│  │                                                          │  │
│  │         ┌─────────────────┐                             │  │
│  │         │   Search Area   │  (blue polygon)             │  │
│  │         │    ┌─┐          │                             │  │
│  │         │    │S│ Start    │  (green marker)             │  │
│  │         │    └─┘          │                             │  │
│  │         │         ┌─┐     │                             │  │
│  │         │         │H│Home │  (red marker)               │  │
│  │         │         └─┘     │                             │  │
│  │         └─────────────────┘                             │  │
│  │                                                          │  │
│  └─────────────────────────────────────────────────────────┘  │
│                                                                 │
│  Stop Conditions:                                               │
│  ┌─────────────────┐ ┌─────────────────┐                      │
│  │ Max Poles: [50] │ │ Max Time: [30]m │                      │
│  └─────────────────┘ └─────────────────┘                      │
│  ┌─────────────────┐ ┌─────────────────┐                      │
│  │ Min Battery:[25]│ │ Coverage: [90]% │                      │
│  └─────────────────┘ └─────────────────┘                      │
│                                                                 │
│  Search Pattern: [Lawnmower ▼]                                 │
│  Sweep Spacing:  [30] meters                                   │
│  Flight Altitude: [30] meters AGL                              │
│                                                                 │
│  Assign Drone: [Drone-001 (idle) ▼]                           │
│                                                                 │
│  [Cancel]                              [Create Job]            │
└────────────────────────────────────────────────────────────────┘
```

**Map Features:**
- Mapbox GL JS or Google Maps
- Polygon drawing tool (for search area)
- Draggable markers (start point, home point)
- Layer toggle for satellite/terrain
- Show existing pole data if available (from previous surveys)

#### 3. Live Dashboard (`/jobs/{id}/live`)
```
┌────────────────────────────────────────────────────────────────┐
│  Highway 287 Survey                    [Pause] [Abort] [Back]  │
├────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────────────────────────┐ ┌──────────────────────┐│
│  │                                   │ │ Status              ││
│  │           LIVE MAP               │ │                      ││
│  │                                   │ │ State: FOLLOWING    ││
│  │    ✈ (drone icon, rotates)       │ │ Battery: 85%  ████░ ││
│  │    ─── (flight path trail)       │ │ Elapsed: 8:32       ││
│  │    📍 (pole markers as found)    │ │ Coverage: 35%       ││
│  │    ▢ (search area outline)       │ │                      ││
│  │                                   │ │ Poles Found: 12     ││
│  │                                   │ │ Poles Skipped: 3    ││
│  │                                   │ │                      ││
│  │                                   │ │ Speed: 3.5 m/s      ││
│  │                                   │ │ Altitude: 30m AGL   ││
│  └──────────────────────────────────┘ └──────────────────────┘│
│                                                                 │
│  Recent Photos (live feed):                                    │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐      │
│  │        │ │        │ │        │ │        │ │        │      │
│  │ thumb  │ │ thumb  │ │ thumb  │ │ thumb  │ │ thumb  │      │
│  │        │ │        │ │        │ │        │ │        │      │
│  └────────┘ └────────┘ └────────┘ └────────┘ └────────┘      │
│  Pole #12   Pole #11   Pole #10   Pole #9    Pole #8         │
│  h_frame    single     h_frame    h_frame    wood            │
│  87%        92%        85%        89%        78%             │
│                                                                 │
└────────────────────────────────────────────────────────────────┘
```

**Real-time Updates (WebSocket):**
- Drone position updates map marker (1 Hz)
- Flight path draws as trail
- Pole markers appear as discovered
- Photos appear in feed within 5 seconds
- Status panel updates continuously

#### 4. Job Results (`/jobs/{id}/results`)
- Map with all discovered poles
- Click pole → show photos, details
- Export options (CSV, GeoJSON, KML)
- Generate report (PDF)

### Components

#### `<MapView>`
- Props: `searchArea`, `startPoint`, `homePoint`, `poles`, `dronePosition`, `flightPath`
- Editable mode (for job creation)
- View-only mode (for dashboard)

#### `<DroneStatus>`
- Props: `battery`, `state`, `speed`, `altitude`, `coverage`
- Visual battery indicator
- State badge with color

#### `<PhotoGrid>`
- Props: `photos[]`
- Thumbnail grid, click to enlarge
- Auto-scrolls as new photos arrive

#### `<PoleMarker>`
- Props: `pole`, `onClick`
- Icon based on pole type
- Color based on confidence

---

## S3 & Photo Pipeline

### Bucket Structure
```
s3://aerovision-photos/
├── jobs/
│   └── {job_id}/
│       └── poles/
│           └── {pole_id}/
│               ├── original.jpg
│               └── thumbnail.jpg
```

### Upload Flow
1. Drone requests presigned URL
2. Drone uploads directly to S3
3. S3 triggers Lambda on upload
4. Lambda:
   - Generates thumbnail (300x300)
   - Updates `photos` table
   - Publishes to SNS/WebSocket
5. CloudFront serves photos

### Lambda Function (Python)
```python
def handler(event, context):
    bucket = event['Records'][0]['s3']['bucket']['name']
    key = event['Records'][0]['s3']['object']['key']

    # Parse key: jobs/{job_id}/poles/{pole_id}/original.jpg
    parts = key.split('/')
    job_id = parts[1]
    pole_id = parts[3]

    # Generate thumbnail
    thumbnail_key = key.replace('original.jpg', 'thumbnail.jpg')
    # ... resize with Pillow ...

    # Update database
    # ... insert into photos table ...

    # Notify via WebSocket
    # ... publish to API Gateway WebSocket ...
```

---

## Authentication

### API Authentication
- Bearer token in `Authorization` header
- Drone has long-lived API key
- Web users have JWT (short-lived)

### Drone Registration
- Each drone has unique `drone_id` and `api_key`
- API key stored securely on Jetson
- Rate limiting per drone

---

## Error Handling

### API Errors
```json
{
  "error": {
    "code": "JOB_NOT_FOUND",
    "message": "Job with ID xyz not found",
    "details": {}
  }
}
```

### Common Error Codes
| Code | HTTP | Description |
|------|------|-------------|
| `JOB_NOT_FOUND` | 404 | Job ID doesn't exist |
| `JOB_ALREADY_RUNNING` | 409 | Can't modify running job |
| `DRONE_OFFLINE` | 503 | Assigned drone not responding |
| `INVALID_POLYGON` | 400 | Search area geometry invalid |
| `UPLOAD_FAILED` | 500 | S3 upload error |

---

## Testing

### Mock Drone Client
AeroSync should have a mock drone mode for testing:
```bash
# Start mock drone that simulates a mission
python -m aerosync.mock_drone --job-id <id> --speed 10x
```

This should:
- Connect to WebSocket
- Send fake position updates along lawnmower path
- Generate fake pole discoveries
- Upload placeholder photos
- Complete mission after time/coverage limit

### API Tests
- Create job with valid polygon
- Create job with invalid polygon (should fail)
- Report pole to non-existent job (should fail)
- WebSocket connection and message handling
- S3 presigned URL generation
- Photo upload and thumbnail generation

---

## Environment Variables

```bash
# Database
DATABASE_URL=postgresql://user:pass@host:5432/aerosync
REDIS_URL=redis://host:6379

# AWS
AWS_REGION=us-west-2
S3_BUCKET=aerovision-photos
CLOUDFRONT_DOMAIN=d123456.cloudfront.net

# Auth
JWT_SECRET=...
DRONE_API_KEY_SALT=...

# WebSocket
WS_ENDPOINT=wss://api.aerosync.com/ws
```

---

## Deployment Notes

### Expected Load
- 1-10 concurrent missions
- ~1 pole/minute during flight
- ~1 photo/pole (1-5 MB each)
- 1 Hz position updates per drone

### Latency Requirements
- Photo thumbnail visible: <5 seconds from capture
- Position update to map: <1 second
- Pole marker on map: <2 seconds from detection

### Scaling Considerations
- WebSocket connections: Use API Gateway or dedicated WS server
- Photo processing: Lambda scales automatically
- Database: Read replicas for dashboard queries
- Telemetry: Consider TimescaleDB for time-series

---

## Integration Checklist

Before connecting to real drone, verify:

- [ ] `GET /api/jobs/{id}` returns correct schema
- [ ] `POST /api/jobs/{id}/start` updates state to 'running'
- [ ] `POST /api/jobs/{id}/poles` creates pole record
- [ ] `PUT /api/jobs/{id}/status` updates job progress
- [ ] `POST /api/jobs/{id}/complete` finalizes job
- [ ] WebSocket accepts drone connection
- [ ] WebSocket broadcasts to web clients
- [ ] S3 presigned URLs work
- [ ] Photo thumbnails generate
- [ ] Map displays poles in real-time
- [ ] Photo feed updates in real-time
