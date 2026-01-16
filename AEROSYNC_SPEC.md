# AeroSync Platform Specification

**Version:** 1.0
**Last Updated:** January 16, 2026 (Backend Complete)

## Overview

AeroSync is the web-based mission control platform for autonomous drone pole inspection. It allows operators to:
1. Create autonomous inspection missions by selecting search areas on a map
2. Monitor live drone telemetry and position during missions
3. View detected poles as they are discovered in real-time
4. Access captured photos within 5 seconds of capture
5. Review mission results and generate reports

**Technology Stack:** Flask (Python) + Angular 20 + PostgreSQL + S3 + WebSocket

---

## System Goals

### User Workflow
1. Operator opens AeroSync, clicks "Create Mission"
2. Selects plot of land containing poles to inspect on map
3. Configures mission parameters (altitude, speed, stop conditions)
4. Starts mission - drone takes off automatically
5. Watches live dashboard as drone flies and finds poles
6. Views photos streaming in < 5 seconds after capture
7. Mission completes - drone returns home automatically
8. Reviews results, exports data, generates report

### Technical Requirements
- Real-time position updates (1 Hz from drone)
- Photo delivery < 5 seconds from capture to display
- Support multiple concurrent missions (1-10 drones)
- Reliable WebSocket connection with auto-reconnect
- S3 storage for photos with CDN delivery
- Postgres database with spatial support

---

## Status Update (Jan 16, 2026)

### ✅ Completed Today
- **Job Model Updated** - Added all 11 mission columns to `app/models.py` (lines 180-191)
- **Database Schema Synced** - SQLAlchemy model now matches migration
- **Backend Server Running** - All APIs operational on localhost:8000
- **Mission Creation Endpoint Ready** - `POST /api/jobs` accepts all mission fields

### 🔨 Next Steps
1. Test mission creation from frontend
2. Verify live dashboard map rendering
3. Test WebSocket events with simulator

---

## What We Have

### Database Schema (Complete ✅)

**Migration:** `migrations/add_live_mission_support.py` (applied)
**SQLAlchemy Model:** `app/models.py` - Job class (lines 164-197)

**jobs table:**
```sql
id                  INTEGER PRIMARY KEY
client_id           INTEGER (FK to clients)
name                VARCHAR(255)
mission_type        VARCHAR(50)     -- 'static' | 'autonomous'
mission_state       VARCHAR(50)     -- 'pending' | 'running' | 'completed' | 'aborted'
search_area         JSON            -- GeoJSON polygon
search_pattern      VARCHAR(50)     -- 'line_follow' | 'lawnmower' | 'spiral'
search_config       JSON            -- {sweep_spacing, flight_altitude, speed}
stop_conditions     JSON            -- {max_poles, max_time_minutes, min_battery_percent, coverage_threshold}
inspection_config   JSON            -- {min_confidence, camera_required, capture_distance, photos_per_pole}
started_at          TIMESTAMP
completed_at        TIMESTAMP
poles_found         INTEGER
coverage_percent    FLOAT
```

**pins table (with detection fields):**
```sql
id                   INTEGER PRIMARY KEY
job_id               INTEGER (FK to jobs)
lat, lng             NUMERIC(10,8)
detection_source     VARCHAR(20)     -- 'manual' | 'lidar' | 'camera' | 'hybrid'
confidence           FLOAT           -- Overall confidence
lidar_confidence     FLOAT
camera_confidence    FLOAT
height_estimate      FLOAT           -- Meters
radius_estimate      FLOAT           -- Meters
altitude_msl         FLOAT
altitude_agl         FLOAT
local_x, local_y, local_z  FLOAT   -- Local coordinates
detected_at          TIMESTAMP
detection_metadata   JSON
```

**photos table (with drone metadata):**
```sql
id              INTEGER PRIMARY KEY
job_id          INTEGER (FK to jobs)
pin_id          INTEGER (FK to pins)
s3_url          TEXT
filename        VARCHAR(255)
lat, lng        NUMERIC
drone_lat       FLOAT           -- Drone position when photo taken
drone_lng       FLOAT
drone_alt       FLOAT
drone_heading   FLOAT
captured_at     TIMESTAMP
```

### REST API (Complete)

**File:** `app/api/drone.py`

All endpoints require `Authorization: Bearer {drone_api_key}` header.

#### Mission Lifecycle

**GET /api/drone/jobs/{job_id}**
Fetch mission configuration.

Response:
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
    "sweep_spacing": 30,
    "flight_altitude": 30,
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

**POST /api/drone/jobs/{job_id}/start**
Drone signals mission start.

Request: `{}` (optional: `{"drone_battery": 98.5, "gps_fix": true}`)

Response:
```json
{
  "status": "started",
  "started_at": "2026-01-16T14:35:00Z"
}
```

**PUT /api/drone/jobs/{job_id}/status**
Periodic status update (every 10-30 seconds recommended).

Request:
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

Response: `{"status": "updated"}`

**POST /api/drone/jobs/{job_id}/complete**
Mission completion.

Request:
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

Response: `{"status": "completed"}`

**POST /api/drone/jobs/{job_id}/abort**
Abort mission (can be called by operator via UI or drone).

Response: `{"status": "aborted"}`

Side effect: Broadcasts abort command to drone via WebSocket.

#### Pole Detection

**POST /api/drone/jobs/{job_id}/poles**
Report detected pole.

Request:
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

Response:
```json
{
  "pole_id": "789",
  "status": "created"
}
```

Side effects:
- Creates Pin record in database
- Increments `jobs.poles_found`
- Broadcasts `pole_found` event via WebSocket to all web clients

**GET /api/drone/jobs/{job_id}/poles**
List all detected poles for a job (for verification).

Response:
```json
{
  "poles": [
    {
      "pole_id": "789",
      "latitude": 41.12845,
      "longitude": -105.46123,
      "pole_type": "h_frame",
      "confidence": 0.87,
      "detected_at": "2026-01-16T14:40:23Z"
    }
  ],
  "total": 47
}
```

#### Photo Upload

**GET /api/drone/jobs/{job_id}/poles/{pole_id}/photo/upload-url**
Request S3 presigned URL for photo upload.

Response:
```json
{
  "upload_url": "https://s3.amazonaws.com/bucket/key?X-Amz-Signature=...",
  "photo_id": "photo-uuid",
  "s3_key": "jobs/123/poles/789/photo_1705412423.jpg",
  "expires_in": 300
}
```

**PUT {upload_url}** (Direct to S3)
Drone uploads photo directly to S3 using presigned URL.

Content-Type: `image/jpeg`
Body: Raw image bytes

**POST /api/drone/jobs/{job_id}/poles/{pole_id}/photo/confirm**
Confirm photo upload completion.

Request:
```json
{
  "photo_id": "photo-uuid",
  "s3_key": "jobs/123/poles/789/photo_1705412423.jpg"
}
```

Response: `{"status": "confirmed"}`

Side effects:
- Creates Photo record in database
- Triggers thumbnail generation (Lambda or server-side)
- Broadcasts `photo_ready` event via WebSocket

**Alternative: POST /api/drone/jobs/{job_id}/poles/{pole_id}/photo**
Direct multipart upload (for testing/fallback).

Content-Type: `multipart/form-data`
Body: photo file + metadata

Response:
```json
{
  "photo_id": "photo-uuid",
  "status": "uploaded"
}
```

#### Health Check

**GET /api/drone/health**
Health check (no auth required).

Response:
```json
{
  "status": "ok",
  "service": "aerosync-drone-api",
  "version": "1.0.0"
}
```

### WebSocket Server (Complete)

**File:** `app/websocket.py`

Uses Flask-SocketIO with room-based messaging.

#### Connection

```javascript
// Client connects
const socket = io('http://localhost:8000');

// Server responds
socket.on('connected', (data) => {
  console.log('Connected:', data.sid);
});
```

#### Join Job Room

```javascript
// Web client joins
socket.emit('join_job', {
  job_id: 123,
  type: 'web'
});

// Drone client joins
socket.emit('join_job', {
  job_id: 123,
  type: 'drone'
});

// Server confirms
socket.on('joined', (data) => {
  console.log('Joined room:', data.room);
});
```

Rooms: `job_{id}_web` and `job_{id}_drone`

#### Events: Drone → Server

**drone_position**
```javascript
socket.emit('drone_position', {
  job_id: 123,
  lat: 41.318285,
  lon: -105.568625,
  alt: 30.5,
  heading: 45.2,
  speed: 3.5,
  battery: 85.2,
  timestamp: 1705412423.456
});
```

Server automatically broadcasts to web clients in `job_{id}_web` room.

#### Events: Server → Drone

**abort**
```javascript
socket.on('abort', (data) => {
  console.log('Abort command received:', data.job_id);
  // Drone should initiate RTL
});
```

Triggered when operator clicks abort button in UI or API receives abort request.

#### Events: Server → Web Clients

**drone_position**
```javascript
socket.on('drone_position', (data) => {
  updateDroneMarker(data.lat, data.lon, data.heading);
  updateBattery(data.battery);
});
```

**pole_found**
```javascript
socket.on('pole_found', (data) => {
  addPoleMarker(data.lat, data.lon, data.pole_type, data.confidence);
  incrementPoleCounter();
});
```

**photo_ready**
```javascript
socket.on('photo_ready', (data) => {
  addPhotoToGallery(data.photo_url, data.thumbnail_url);
});
```

**mission_status**
```javascript
socket.on('mission_status', (data) => {
  updateStats(data.poles_found, data.coverage_percent, data.battery_percent);
});
```

**mission_complete**
```javascript
socket.on('mission_complete', (data) => {
  showCompletionDialog(data.stop_reason, data.summary);
});
```

### Frontend Services (Partial)

**MissionService** (`frontend/src/app/core/services/mission.service.ts`)
- ✅ TypeScript interfaces: `Mission`, `MissionConfig`, `CreateMissionRequest`
- ✅ CRUD methods: `loadMissions()`, `getMission()`, `createMission()`, `updateMission()`, `abortMission()`
- ✅ State management with BehaviorSubject

**WebSocketService** (`frontend/src/app/core/services/websocket.service.ts`)
- ✅ Socket.IO client connection
- ✅ Auto-reconnect logic
- ✅ Event types defined: `DronePosition`, `MissionStatusEvent`, `MissionCompleteEvent`
- ✅ Methods: `connect()`, `joinJob()`, `leaveJob()`

**LiveMissionDashboardComponent** (`frontend/src/app/shared/components/live-mission-dashboard.component.ts`)
- ✅ Displays mission stats (poles found, coverage, battery, elapsed time)
- ✅ Abort mission button with confirmation
- ✅ WebSocket subscription for live updates
- ❌ Missing: Map with drone marker
- ❌ Missing: Pole markers
- ❌ Missing: Photo gallery

### Testing Tools (Complete)

**Create Test Mission** (`dev-tools/create_test_mission.py`)
```bash
python dev-tools/create_test_mission.py --client-id 1 --name "Test Mission"
```

Creates autonomous mission in database with full configuration.

**Mission Simulator** (`tests/test_drone_mission.py`)
```bash
python tests/test_drone_mission.py --job-id 1 --num-poles 10
```

Simulates complete mission:
1. Fetches job config
2. Starts mission
3. Reports fake poles with realistic GPS offsets
4. Sends status updates
5. Completes mission

Use this to test AeroSync without real drone.

---

## What We Need

### Priority 1: Mission Creation UI (1-2 days)

**Goal:** Operator can create autonomous mission by drawing on map and filling form.

**Create File:** `frontend/src/app/shared/components/mission-creation-modal.component.ts`

**Requirements:**

1. **Map Component:**
   - Leaflet map (reuse existing map service)
   - Polygon drawing tool (Leaflet.draw or custom)
   - Draggable markers for start point (green) and home point (red)
   - Satellite/terrain layer toggle

2. **Form Fields:**
   ```typescript
   interface MissionForm {
     name: string;                    // Mission name
     search_pattern: 'line_follow' | 'lawnmower' | 'spiral';

     // Search config
     sweep_spacing: number;           // Meters (for lawnmower/spiral)
     flight_altitude: number;         // Meters AGL
     speed: number;                   // m/s

     // Stop conditions
     max_poles: number;               // 0 = unlimited
     max_time_minutes: number;        // 0 = unlimited
     min_battery_percent: number;     // Return to launch threshold
     coverage_threshold: number;      // 0-1 (0.9 = 90%)

     // Inspection config
     min_confidence: number;          // 0-1
     camera_required: boolean;        // Require visual confirmation
     capture_distance: number;        // Meters from pole
     photos_per_pole: number;
   }
   ```

3. **Validation:**
   - Polygon must be closed (first point = last point)
   - Start and home points must be inside polygon
   - Altitude > 0, speed > 0
   - Max poles and time can be 0 (unlimited)
   - Battery threshold 20-50%
   - Coverage threshold 0.5-1.0

4. **Preview (Optional):**
   - Estimate flight time based on area and speed
   - Estimate distance
   - Show simplified flight path preview

5. **Submit:**
   - Call `MissionService.createMission()`
   - Show success message with mission ID
   - Navigate to live dashboard

**Files to Modify:**
- Create `frontend/src/app/shared/components/mission-creation-modal.component.ts`
- Update `frontend/src/app/features/map.component.ts` to open modal
- Verify `POST /api/jobs` accepts all mission fields (should already work)

### Priority 2: Live Map Updates (1 day)

**Goal:** Show live drone position and detected poles on map during mission.

**Files to Modify:**
- `frontend/src/app/shared/components/live-mission-dashboard.component.ts`
- `frontend/src/app/features/map.component.ts` (reuse existing map)

**Requirements:**

1. **Add Map to Dashboard:**
   - Embed Leaflet map in dashboard
   - Show search area polygon (blue outline)
   - Initialize map centered on search area

2. **Drone Marker:**
   - Custom drone/aircraft icon
   - Rotate icon based on heading
   - Update position on `drone_position` event (1 Hz)
   - Optional: Pan map to follow drone (toggle in UI)

3. **Flight Path Trail:**
   - Polyline connecting recent drone positions
   - Keep last 50-100 positions
   - Color: cyan or yellow
   - Fade older positions (optional)

4. **Pole Markers:**
   - Add marker when `pole_found` event received
   - Icon: pin or circle
   - Color based on confidence:
     - Green: 0.8-1.0
     - Yellow: 0.6-0.8
     - Red: < 0.6
   - Click pole marker to show popup:
     - Pole type
     - Confidence
     - Detection time
     - Link to photos

5. **Start/Home Markers:**
   - Show start point (green flag)
   - Show home point (red flag)
   - Load from mission config

**Implementation:**
```typescript
// Subscribe to WebSocket events
this.wsService.dronePosition$.subscribe(pos => {
  this.updateDroneMarker(pos.lat, pos.lon, pos.heading);
  this.addToFlightPath(pos.lat, pos.lon);
});

this.wsService.poleFound$.subscribe(pole => {
  this.addPoleMarker(pole.lat, pole.lon, pole);
});
```

### Priority 3: Photo Upload Pipeline (2-3 days)

**Goal:** Photos uploaded to S3 appear in UI within 5 seconds.

**Backend Files to Modify:**
- `app/api/drone.py` (implement presigned URL endpoints)
- Create `app/utils/s3_helpers.py` (S3 utilities)

**Requirements:**

1. **S3 Presigned URL Generation:**

Add to `app/api/drone.py`:
```python
import boto3
from datetime import datetime, timedelta

s3_client = boto3.client('s3',
    region_name=Config.S3_PHOTOS_REGION,
    aws_access_key_id=Config.AWS_ACCESS_KEY_ID,
    aws_secret_access_key=Config.AWS_SECRET_ACCESS_KEY
)

@drone_bp.route('/jobs/<int:job_id>/poles/<int:pole_id>/photo/upload-url', methods=['GET'])
@drone_auth_required
def get_photo_upload_url(job_id, pole_id):
    """Generate S3 presigned URL for photo upload."""

    # Generate unique photo ID
    photo_id = str(uuid.uuid4())
    timestamp = int(time.time())

    # S3 key structure
    s3_key = f"jobs/{job_id}/poles/{pole_id}/photo_{timestamp}.jpg"

    # Generate presigned URL (expires in 5 minutes)
    upload_url = s3_client.generate_presigned_url(
        'put_object',
        Params={
            'Bucket': Config.S3_PHOTOS_BUCKET,
            'Key': s3_key,
            'ContentType': 'image/jpeg'
        },
        ExpiresIn=300
    )

    return jsonify({
        'upload_url': upload_url,
        'photo_id': photo_id,
        's3_key': s3_key,
        'expires_in': 300
    })
```

2. **Photo Upload Confirmation:**

Add to `app/api/drone.py`:
```python
@drone_bp.route('/jobs/<int:job_id>/poles/<int:pole_id>/photo/confirm', methods=['POST'])
@drone_auth_required
def confirm_photo_upload(job_id, pole_id):
    """Confirm photo upload and create database record."""
    data = request.get_json()
    photo_id = data.get('photo_id')
    s3_key = data.get('s3_key')

    with get_db_session() as session:
        pin = session.query(Pin).filter_by(id=pole_id).first()
        if not pin:
            abort(404)

        # Build URLs
        s3_url = f"https://{Config.S3_PHOTOS_BUCKET}.s3.{Config.S3_PHOTOS_REGION}.amazonaws.com/{s3_key}"

        if Config.CLOUDFRONT_PHOTOS_DOMAIN:
            photo_url = f"https://{Config.CLOUDFRONT_PHOTOS_DOMAIN}/{s3_key}"
        else:
            photo_url = s3_url

        # Create photo record
        photo = Photo(
            job_id=job_id,
            pin_id=pole_id,
            s3_url=s3_url,
            filename=s3_key.split('/')[-1],
            lat=pin.lat,
            lng=pin.lng,
            captured_at=func.now()
        )
        session.add(photo)
        session.flush()

        # Trigger thumbnail generation
        generate_thumbnail_async(s3_key)

        # Broadcast to web clients
        thumbnail_url = photo_url.replace('.jpg', '_thumb.jpg')
        broadcast_photo_ready(
            job_id=job_id,
            pole_id=pole_id,
            photo_id=photo.id,
            photo_url=photo_url
        )

        return jsonify({'status': 'confirmed', 'photo_id': photo.id})
```

3. **Thumbnail Generation:**

**Option A: Lambda Function**
- Create Lambda function triggered by S3 upload
- Use Pillow to resize image to 300x300
- Save thumbnail with `_thumb.jpg` suffix

**Option B: Server-Side**
```python
def generate_thumbnail_async(s3_key: str):
    """Generate thumbnail asynchronously."""
    import threading

    def _generate():
        # Download image from S3
        response = s3_client.get_object(
            Bucket=Config.S3_PHOTOS_BUCKET,
            Key=s3_key
        )
        image_data = response['Body'].read()

        # Resize with Pillow
        from PIL import Image
        import io

        img = Image.open(io.BytesIO(image_data))
        img.thumbnail((300, 300), Image.LANCZOS)

        # Save thumbnail
        thumb_buffer = io.BytesIO()
        img.save(thumb_buffer, format='JPEG', quality=85)
        thumb_buffer.seek(0)

        thumb_key = s3_key.replace('.jpg', '_thumb.jpg')
        s3_client.put_object(
            Bucket=Config.S3_PHOTOS_BUCKET,
            Key=thumb_key,
            Body=thumb_buffer,
            ContentType='image/jpeg'
        )

    # Run in background thread
    thread = threading.Thread(target=_generate)
    thread.daemon = True
    thread.start()
```

4. **Environment Variables:**

Add to `.env`:
```bash
S3_PHOTOS_BUCKET=aerosync-photos
S3_PHOTOS_REGION=us-east-1
CLOUDFRONT_PHOTOS_DOMAIN=d123456.cloudfront.net  # Optional
AWS_ACCESS_KEY_ID=your-key
AWS_SECRET_ACCESS_KEY=your-secret
```

5. **S3 Bucket Setup:**
- Create S3 bucket
- Enable CORS:
```json
[
  {
    "AllowedHeaders": ["*"],
    "AllowedMethods": ["PUT", "GET"],
    "AllowedOrigins": ["*"],
    "ExposeHeaders": ["ETag"]
  }
]
```
- Optional: Create CloudFront distribution

### Priority 4: Photo Gallery (1 day)

**Goal:** Display photos in live dashboard as they are uploaded.

**Create File:** `frontend/src/app/shared/components/photo-grid.component.ts`

**Requirements:**

1. **Grid Component:**
   ```typescript
   @Component({
     selector: 'app-photo-grid',
     template: `
       <div class="photo-grid grid grid-cols-3 gap-2">
         <div *ngFor="let photo of photos" class="photo-item">
           <img [src]="photo.thumbnail_url"
                (click)="openFullSize(photo)"
                class="cursor-pointer rounded hover:opacity-80">
           <div class="text-xs mt-1">
             <div>{{ photo.pole_type }}</div>
             <div>{{ photo.confidence }}% confidence</div>
           </div>
         </div>
       </div>
     `
   })
   export class PhotoGridComponent {
     @Input() photos: Photo[] = [];
   }
   ```

2. **Subscribe to Events:**
   ```typescript
   // In live-mission-dashboard.component.ts
   this.wsService.photoReady$.subscribe(photo => {
     this.photos.unshift(photo);  // Add to beginning
     this.scrollToTop();
   });
   ```

3. **Full-Size Modal:**
   - Click thumbnail opens modal
   - Display full-resolution image
   - Show metadata (type, confidence, timestamp, GPS)
   - Next/previous buttons
   - Download button
   - Close on ESC or click outside

4. **Loading States:**
   - Show spinner while uploading
   - Show placeholder until thumbnail loads
   - Handle broken image links

### Priority 5: Backend Job Creation Verification (30 mins)

**Goal:** Ensure `POST /api/jobs` accepts all mission fields.

**File:** `app/api/jobs.py`

**Verify endpoint accepts:**
```python
@jobs_bp.route('', methods=['POST'])
def create_job():
    data = request.get_json()

    job = Job(
        client_id=data['client_id'],
        name=data['name'],
        mission_type=data.get('mission_type', 'static'),
        search_area=data.get('search_area'),
        search_pattern=data.get('search_pattern'),
        search_config=data.get('search_config'),
        stop_conditions=data.get('stop_conditions'),
        inspection_config=data.get('inspection_config'),
        mission_state='pending',
        status='active'
    )

    session.add(job)
    return jsonify(job.to_dict()), 201
```

Add `to_dict()` method to Job model if missing.

---

## Testing

### Unit Tests

**Backend:**
```bash
pytest tests/test_drone_api.py
pytest tests/test_websocket.py
pytest tests/test_s3_upload.py
```

**Frontend:**
```bash
cd frontend
ng test
```

### Integration Testing

**Mission Simulator:**
```bash
# Terminal 1: Start AeroSync
python app/main.py

# Terminal 2: Create test mission
python dev-tools/create_test_mission.py --client-id 1

# Terminal 3: Run simulator
python tests/test_drone_mission.py --job-id 1 --num-poles 10
```

**Manual Testing:**
1. Open AeroSync in browser
2. Create mission via UI
3. Open live dashboard
4. Run simulator
5. Verify:
   - Mission appears in database
   - Simulator fetches mission config
   - Position updates show on map
   - Pole markers appear
   - Status updates in real-time

### Load Testing

Simulate 5 concurrent drones:
```bash
for i in {1..5}; do
  python tests/test_drone_mission.py --job-id $i --num-poles 20 &
done
```

Verify:
- WebSocket handles concurrent connections
- Database not overwhelmed
- UI responsive with multiple live missions

---

## Deployment Checklist

### Environment Variables
```bash
DATABASE_URL=postgresql://...
SECRET_KEY=...
DRONE_API_KEY=...
S3_PHOTOS_BUCKET=aerosync-photos
S3_PHOTOS_REGION=us-east-1
CLOUDFRONT_PHOTOS_DOMAIN=...
AWS_ACCESS_KEY_ID=...
AWS_SECRET_ACCESS_KEY=...
```

### Database Migration
```bash
python migrations/add_live_mission_support.py
```

### S3 Setup
1. Create bucket
2. Configure CORS
3. Create IAM user with S3 permissions
4. Optional: Create CloudFront distribution

### Production Server
```bash
gunicorn -w 4 -k geventwebsocket.gunicorn.workers.GeventWebSocketWorker -b 0.0.0.0:8000 app.main:app
```

Note: Use `geventwebsocket` worker for WebSocket support.

---

## API Reference Summary

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/drone/health` | GET | Health check |
| `/api/drone/jobs/{id}` | GET | Fetch mission config |
| `/api/drone/jobs/{id}/start` | POST | Start mission |
| `/api/drone/jobs/{id}/status` | PUT | Status update |
| `/api/drone/jobs/{id}/complete` | POST | Complete mission |
| `/api/drone/jobs/{id}/abort` | POST | Abort mission |
| `/api/drone/jobs/{id}/poles` | POST | Report pole |
| `/api/drone/jobs/{id}/poles` | GET | List poles |
| `/api/drone/jobs/{id}/poles/{pole_id}/photo/upload-url` | GET | Get S3 upload URL |
| `/api/drone/jobs/{id}/poles/{pole_id}/photo/confirm` | POST | Confirm upload |
| `/api/drone/jobs/{id}/poles/{pole_id}/photo` | POST | Direct upload (fallback) |

## WebSocket Events Summary

| Event | Direction | Purpose |
|-------|-----------|---------|
| `join_job` | Client → Server | Join job room |
| `drone_position` | Drone → Server → Web | Position update (1 Hz) |
| `pole_found` | Server → Web | New pole detected |
| `photo_ready` | Server → Web | Photo available |
| `mission_status` | Server → Web | Status update |
| `mission_complete` | Server → Web | Mission finished |
| `abort` | Server → Drone | Abort command |

---

## Time Estimates

| Task | Time | Priority |
|------|------|----------|
| Mission creation UI | 1-2 days | P1 |
| Live map updates | 1 day | P1 |
| Photo S3 pipeline | 2-3 days | P1 |
| Photo gallery | 1 day | P1 |
| Backend verification | 30 mins | P1 |
| **Total** | **5-6 days** | |

---

## Jetson Drone Compatibility

This section documents what the AeroSync backend must provide for successful integration with the Jetson drone system.

### Required API Response Formats

#### GET /api/drone/jobs/{id} Response

The Jetson `mission_planner.py` expects these fields:

```json
{
  "job_id": "string (required)",
  "name": "string (required)",
  "search_pattern": "line_follow | lawnmower | spiral (required)",
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
    {"lat": 41.318, "lon": -105.568, "pole_id": "optional"}
  ],
  "state": "pending | running | completed | aborted"
}
```

**Important:**
- `search_area` MUST be a valid GeoJSON Polygon
- Polygon coordinates are `[lon, lat]` order (GeoJSON standard)
- `search_config.flight_altitude` used as mission altitude
- `search_config.speed` used as cruise speed

#### POST /api/drone/jobs/{id}/poles Request

Jetson sends poles in this format:

```json
{
  "pole_id": "uuid-generated-by-drone",
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
    "pole_type": "h_frame | metal_tree_pole | single_pole | wood_distribution | utility_pole",
    "lidar_confidence": 0.92,
    "camera_confidence": 0.81,
    "height_estimate": 14.5,
    "radius_estimate": 0.18
  },
  "metadata": {}
}
```

**Backend should:**
- Create Pin record with detection fields
- Increment `jobs.poles_found`
- Broadcast `pole_found` event to web clients

#### PUT /api/drone/jobs/{id}/status Request

Jetson sends status updates every 10-30 seconds:

```json
{
  "state": "running",
  "guidance_state": "IDLE | SEARCH | FOLLOW | RESCAN | OBSTACLE_STOP | ABORT",
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

#### POST /api/drone/jobs/{id}/complete Request

```json
{
  "stop_reason": "coverage_complete | max_poles_reached | time_limit | low_battery | user_abort | failsafe | error",
  "summary": {
    "poles_inspected": 47,
    "poles_skipped_duplicate": 3,
    "duration_minutes": 28.5,
    "final_battery_percent": 32.1,
    "final_coverage_percent": 0.95
  }
}
```

### WebSocket Protocol

#### Connection

Jetson connects to Socket.IO endpoint and emits `join_job`:

```javascript
socket.emit('join_job', {
  job_id: 123,
  type: 'drone'
});
```

Server should add drone to room `job_{id}_drone`.

#### Drone → Server Events

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

Server should broadcast to `job_{id}_web` room.

#### Server → Drone Events

**abort** - Must be sent when operator clicks abort button:

```json
{
  "event": "abort",
  "job_id": "123"
}
```

Jetson will immediately trigger RTL.

### S3 Upload Requirements

#### GET /api/drone/jobs/{id}/poles/{pole_id}/photo/upload-url

Response MUST include:

```json
{
  "upload_url": "https://s3.amazonaws.com/bucket/key?X-Amz-Signature=...",
  "photo_id": "uuid",
  "s3_key": "jobs/123/poles/789/photo_1705412423.jpg",
  "expires_in": 300
}
```

- `upload_url` must be a valid S3 presigned PUT URL
- URL must allow `Content-Type: image/jpeg`
- Expiration should be at least 60 seconds (300 recommended)

#### POST /api/drone/jobs/{id}/poles/{pole_id}/photo/confirm

Request:

```json
{
  "photo_id": "uuid-from-upload-url-response",
  "s3_key": "jobs/123/poles/789/photo_1705412423.jpg"
}
```

Server should:
1. Create Photo record in database
2. Trigger thumbnail generation
3. Broadcast `photo_ready` event to web clients

### Error Handling

Jetson expects these HTTP status codes:

| Status | Meaning | Jetson Behavior |
|--------|---------|-----------------|
| 200 | Success | Continue |
| 201 | Created | Continue |
| 400 | Bad request | Log error, retry |
| 401 | Unauthorized | Stop mission, alert |
| 404 | Not found | Log error, continue |
| 500 | Server error | Retry with backoff |

### Testing with Jetson

To test AeroSync with the Jetson simulator:

```bash
# On Jetson
python3 perception_nav/src/mission_executor.py

# Or use the test script
python3 -c "
from perception_nav.src.aerosync_client import AeroSyncClient
client = AeroSyncClient()
job = client.fetch_job('test-job-id')
print(job)
"
```

### Checklist for Jetson Compatibility

- [ ] GET `/api/drone/jobs/{id}` returns all required fields
- [ ] `search_area` is valid GeoJSON Polygon with `[lon, lat]` coordinates
- [ ] POST `/api/drone/jobs/{id}/poles` accepts pole report format
- [ ] PUT `/api/drone/jobs/{id}/status` accepts status format
- [ ] POST `/api/drone/jobs/{id}/complete` accepts summary format
- [ ] WebSocket `abort` event sent to drone room on abort
- [ ] S3 presigned URL generation working
- [ ] Photo confirm endpoint creates Photo record
- [ ] `photo_ready` event broadcast on upload confirmation

---

## Support & References

**Repository:** `/Users/nolannachbar/Testing:Dev/AeroSync`

**Key Files:**
- Database: `app/models.py`
- Drone API: `app/api/drone.py`
- WebSocket: `app/websocket.py`
- Mission Service: `frontend/src/app/core/services/mission.service.ts`
- WebSocket Service: `frontend/src/app/core/services/websocket.service.ts`
- Dashboard: `frontend/src/app/shared/components/live-mission-dashboard.component.ts`

**Testing:**
- Simulator: `tests/test_drone_mission.py`
- Create Mission: `dev-tools/create_test_mission.py`

**Jetson Integration:** See `JETSON_SPEC.md` for drone-side implementation.
