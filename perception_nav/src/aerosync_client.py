"""
AeroSync Client Module

REST API and WebSocket client for communicating with AeroSync web application.
Handles job fetching, pole reporting, photo upload, and real-time status updates.

Features:
- REST API client (fetch jobs, report poles, upload photos)
- WebSocket for real-time position/status updates
- Automatic reconnection
- Request retry with exponential backoff
- Offline queue for store-and-forward (when connectivity lost)

Usage:
    from aerosync_client import AeroSyncClient

    client = AeroSyncClient(base_url="https://aerosync.example.com", api_key="...")

    # Fetch job
    job = await client.fetch_job("job-uuid")

    # Report found pole
    await client.report_pole(job_id, inspection_result)

    # Real-time updates
    await client.connect_websocket(job_id)
    await client.send_position(lat, lon, alt, heading, speed)
"""

import asyncio
import json
import time
import threading
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional
from queue import Queue

try:
    import aiohttp
    AIOHTTP_AVAILABLE = True
except ImportError:
    AIOHTTP_AVAILABLE = False
    print("Warning: aiohttp not installed. Run: pip install aiohttp")

try:
    import requests
    REQUESTS_AVAILABLE = True
except ImportError:
    REQUESTS_AVAILABLE = False
    print("Warning: requests not installed. Run: pip install requests")

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

# Import local modules
try:
    from .pole_inspector import InspectionResult
    from .mission_manager import MissionStatus, MissionSummary
except ImportError:
    InspectionResult = None
    MissionStatus = None
    MissionSummary = None


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class AeroSyncConfig:
    """Configuration for AeroSync client."""
    base_url: str = "https://aerosync.example.com"
    api_key: str = ""
    websocket_enabled: bool = True
    upload_photos: bool = True
    position_update_rate_hz: float = 1.0
    retry_max_attempts: int = 3
    retry_base_delay: float = 1.0
    timeout_seconds: float = 30.0


@dataclass
class Job:
    """Job data from AeroSync.

    Matches AEROSYNC_SPEC.md GET /api/drone/jobs/{job_id} response format.
    """
    job_id: str
    name: str
    search_pattern: str
    search_area: dict                   # GeoJSON polygon
    search_config: dict
    stop_conditions: dict
    inspection_config: dict
    previously_inspected: List[dict]
    state: str = 'pending'              # pending, running, completed, aborted

    @classmethod
    def from_dict(cls, data: dict) -> 'Job':
        return cls(
            job_id=data.get('job_id', data.get('id', '')),
            name=data.get('name', ''),
            search_pattern=data.get('search_pattern', 'line_follow'),
            search_area=data.get('search_area', {}),
            search_config=data.get('search_config', {}),
            stop_conditions=data.get('stop_conditions', {}),
            inspection_config=data.get('inspection_config', {}),
            previously_inspected=data.get('previously_inspected', []),
            state=data.get('state', 'pending')
        )

    def to_mission_dict(self) -> dict:
        """Convert to dict format expected by mission_planner.parse_mission()."""
        return {
            'job_id': self.job_id,
            'name': self.name,
            'search_pattern': self.search_pattern,
            'search_area': self.search_area,
            'search_config': self.search_config,
            'stop_conditions': self.stop_conditions,
            'inspection_config': self.inspection_config,
            'previously_inspected': self.previously_inspected
        }


# =============================================================================
# Synchronous Client (for main thread use)
# =============================================================================

class AeroSyncClient:
    """Synchronous AeroSync API client.

    Use this when not running in an async context.
    For async operations, use AeroSyncAsyncClient.
    """

    def __init__(self, config: Optional[AeroSyncConfig] = None,
                 config_path: Optional[str] = None):
        """Initialize AeroSync client.

        Args:
            config: AeroSyncConfig object
            config_path: Path to YAML config file
        """
        if config is not None:
            self.config = config
        elif config_path is not None:
            self.config = self._load_config(config_path)
        else:
            self.config = AeroSyncConfig()

        self._session: Optional[requests.Session] = None
        self._ws_thread: Optional[threading.Thread] = None
        self._ws_running = False
        self._ws_queue: Queue = Queue()

        # Callbacks for WebSocket messages
        self._on_pause: Optional[Callable] = None
        self._on_resume: Optional[Callable] = None
        self._on_abort: Optional[Callable] = None
        self._on_update_conditions: Optional[Callable[[dict], None]] = None

    def _load_config(self, config_path: str) -> AeroSyncConfig:
        """Load configuration from YAML file."""
        config = AeroSyncConfig()

        if not YAML_AVAILABLE:
            return config

        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)

            if 'aerosync' in data:
                a = data['aerosync']
                config.base_url = a.get('base_url', config.base_url)
                config.api_key = a.get('api_key', config.api_key)
                config.websocket_enabled = a.get('websocket_enabled',
                                                  config.websocket_enabled)
                config.upload_photos = a.get('upload_photos', config.upload_photos)
                config.position_update_rate_hz = a.get('position_update_rate_hz',
                                                        config.position_update_rate_hz)

        except Exception as e:
            print(f"Warning: Could not load AeroSync config: {e}")

        return config

    def _get_headers(self) -> dict:
        """Get HTTP headers with auth."""
        headers = {
            'Content-Type': 'application/json',
            'Accept': 'application/json'
        }
        if self.config.api_key:
            headers['Authorization'] = f'Bearer {self.config.api_key}'
        return headers

    def _get_session(self) -> requests.Session:
        """Get or create requests session."""
        if not REQUESTS_AVAILABLE:
            raise RuntimeError("requests library not available")

        if self._session is None:
            self._session = requests.Session()
            self._session.headers.update(self._get_headers())

        return self._session

    def _request_with_retry(self, method: str, url: str,
                            **kwargs) -> requests.Response:
        """Make HTTP request with retry logic."""
        session = self._get_session()
        kwargs.setdefault('timeout', self.config.timeout_seconds)

        last_error = None
        for attempt in range(self.config.retry_max_attempts):
            try:
                response = session.request(method, url, **kwargs)
                response.raise_for_status()
                return response

            except requests.RequestException as e:
                last_error = e
                if attempt < self.config.retry_max_attempts - 1:
                    delay = self.config.retry_base_delay * (2 ** attempt)
                    time.sleep(delay)

        raise last_error

    # =========================================================================
    # REST API Methods
    # =========================================================================

    def fetch_job(self, job_id: str) -> Optional[Job]:
        """Fetch job details from AeroSync.

        Args:
            job_id: Job UUID

        Returns:
            Job object or None if failed
        """
        try:
            url = f"{self.config.base_url}/api/drone/jobs/{job_id}"
            response = self._request_with_retry('GET', url)
            data = response.json()
            return Job.from_dict(data)

        except Exception as e:
            print(f"Failed to fetch job {job_id}: {e}")
            return None

    def start_job(self, job_id: str) -> bool:
        """Acknowledge job start.

        Args:
            job_id: Job UUID

        Returns:
            True if successful
        """
        try:
            url = f"{self.config.base_url}/api/drone/jobs/{job_id}/start"
            self._request_with_retry('POST', url)
            return True

        except Exception as e:
            print(f"Failed to start job {job_id}: {e}")
            return False

    def report_pole(self, job_id: str, result: 'InspectionResult') -> Optional[str]:
        """Report a found pole to AeroSync.

        Args:
            job_id: Job UUID
            result: InspectionResult from pole inspector

        Returns:
            Pole UUID from server or None if failed
        """
        try:
            url = f"{self.config.base_url}/api/drone/jobs/{job_id}/poles"

            # Match AEROSYNC_SPEC.md POST /api/drone/jobs/{job_id}/poles format
            payload = {
                'pole_id': result.global_pole_id,
                'timestamp': result.timestamp_iso,
                'position': {
                    'latitude': result.position_gps[0],
                    'longitude': result.position_gps[1],
                    'altitude_msl': result.position_gps[2] if len(result.position_gps) > 2 else 0,
                    'altitude_agl': getattr(result, 'altitude_agl', result.position_local[2] if result.position_local else 0),
                    'local_x': result.position_local[0] if result.position_local else 0,
                    'local_y': result.position_local[1] if result.position_local else 0,
                    'local_z': result.position_local[2] if result.position_local else 0
                },
                'detection': {
                    'confidence': result.confidence,
                    'pole_type': result.pole_type,
                    'lidar_confidence': result.lidar_confidence,
                    'camera_confidence': result.camera_confidence,
                    'height_estimate': result.pole_height,
                    'radius_estimate': result.pole_radius
                },
                'metadata': result.metadata if hasattr(result, 'metadata') else {}
            }

            response = self._request_with_retry('POST', url, json=payload)
            data = response.json()
            return data.get('pole_id', result.global_pole_id)

        except Exception as e:
            print(f"Failed to report pole: {e}")
            return None

    def upload_photo(self, job_id: str, pole_id: str, photo_path: str) -> Optional[str]:
        """Upload a pole photo to AeroSync (direct upload).

        Args:
            job_id: Job UUID
            pole_id: Pole UUID
            photo_path: Local path to photo file

        Returns:
            Photo UUID from server or None if failed
        """
        if not self.config.upload_photos:
            return None

        try:
            url = f"{self.config.base_url}/api/drone/jobs/{job_id}/poles/{pole_id}/photo"

            with open(photo_path, 'rb') as f:
                files = {'photo': (photo_path.split('/')[-1], f, 'image/jpeg')}
                headers = {'Authorization': f'Bearer {self.config.api_key}'}

                session = self._get_session()
                # Don't use JSON content-type for multipart
                response = session.post(
                    url,
                    files=files,
                    headers=headers,
                    timeout=self.config.timeout_seconds
                )
                response.raise_for_status()

            data = response.json()
            return data.get('photo_id')

        except Exception as e:
            print(f"Failed to upload photo: {e}")
            return None

    # =========================================================================
    # S3 Presigned URL Upload (Alternative to Direct Upload)
    # =========================================================================

    def get_photo_upload_url(self, job_id: str, pole_id: str) -> Optional[dict]:
        """Request S3 presigned URL for photo upload.

        This is the first step of the S3 upload flow:
        1. Request presigned URL from AeroSync
        2. Upload directly to S3
        3. Confirm upload with AeroSync

        Args:
            job_id: Job UUID
            pole_id: Pole UUID

        Returns:
            Dict with upload_url, photo_id, s3_key, expires_in
            or None if failed
        """
        try:
            url = f"{self.config.base_url}/api/drone/jobs/{job_id}/poles/{pole_id}/photo/upload-url"
            response = self._request_with_retry('GET', url)
            return response.json()

        except Exception as e:
            print(f"Failed to get upload URL: {e}")
            return None

    def upload_to_s3(self, upload_url: str, image_data: bytes) -> bool:
        """Upload photo directly to S3 using presigned URL.

        Args:
            upload_url: S3 presigned URL from get_photo_upload_url()
            image_data: JPEG image bytes

        Returns:
            True if upload successful
        """
        if not REQUESTS_AVAILABLE:
            print("requests library not available")
            return False

        try:
            # Direct PUT to S3 with presigned URL
            response = requests.put(
                upload_url,
                data=image_data,
                headers={'Content-Type': 'image/jpeg'},
                timeout=60.0  # Longer timeout for large uploads
            )

            return response.status_code == 200

        except Exception as e:
            print(f"S3 upload failed: {e}")
            return False

    def confirm_photo_upload(self, job_id: str, pole_id: str,
                              photo_id: str, s3_key: str,
                              pole_lat: float = None, pole_lon: float = None,
                              drone_lat: float = None, drone_lon: float = None,
                              drone_alt: float = None, drone_heading: float = None,
                              captured_at: str = None) -> bool:
        """Confirm photo upload completion to AeroSync.

        This is the final step after uploading to S3.

        Args:
            job_id: Job UUID
            pole_id: Pole UUID
            photo_id: Photo ID from get_photo_upload_url()
            s3_key: S3 key from get_photo_upload_url()
            pole_lat: Pole GPS latitude (optional)
            pole_lon: Pole GPS longitude (optional)
            drone_lat: Drone latitude at capture time
            drone_lon: Drone longitude at capture time
            drone_alt: Drone altitude AGL at capture time
            drone_heading: Drone heading in degrees at capture time
            captured_at: ISO8601 timestamp of capture

        Returns:
            True if confirmation successful
        """
        try:
            url = f"{self.config.base_url}/api/drone/jobs/{job_id}/poles/{pole_id}/photo/confirm"
            payload = {
                'photo_id': photo_id,
                's3_key': s3_key,
                'lat': pole_lat,
                'lon': pole_lon,
                'drone_lat': drone_lat,
                'drone_lon': drone_lon,
                'drone_alt': drone_alt,
                'drone_heading': drone_heading,
                'captured_at': captured_at
            }
            # Remove None values
            payload = {k: v for k, v in payload.items() if v is not None}
            self._request_with_retry('POST', url, json=payload)
            return True

        except Exception as e:
            print(f"Upload confirmation failed: {e}")
            return False

    def upload_photo_s3(self, job_id: str, pole_id: str,
                         image_data: bytes,
                         pole_lat: float = None, pole_lon: float = None,
                         drone_lat: float = None, drone_lon: float = None,
                         drone_alt: float = None, drone_heading: float = None,
                         captured_at: str = None,
                         max_retries: int = 3) -> Optional[str]:
        """Complete S3 upload flow with retry logic.

        Performs the full 3-step S3 upload:
        1. Get presigned URL
        2. Upload to S3
        3. Confirm upload with drone position metadata

        Args:
            job_id: Job UUID
            pole_id: Pole UUID
            image_data: JPEG image bytes
            pole_lat: Pole GPS latitude (optional)
            pole_lon: Pole GPS longitude (optional)
            drone_lat: Drone latitude at capture time
            drone_lon: Drone longitude at capture time
            drone_alt: Drone altitude AGL at capture time
            drone_heading: Drone heading in degrees at capture time
            captured_at: ISO8601 timestamp of capture
            max_retries: Number of retry attempts

        Returns:
            Photo ID if successful, None otherwise
        """
        if not self.config.upload_photos:
            return None

        for attempt in range(max_retries):
            try:
                # 1. Get presigned URL
                upload_info = self.get_photo_upload_url(job_id, pole_id)
                if not upload_info:
                    raise Exception("Failed to get upload URL")

                # 2. Upload to S3
                success = self.upload_to_s3(upload_info['upload_url'], image_data)
                if not success:
                    raise Exception("S3 upload failed")

                # 3. Confirm upload with drone metadata
                confirmed = self.confirm_photo_upload(
                    job_id, pole_id,
                    upload_info['photo_id'],
                    upload_info['s3_key'],
                    pole_lat=pole_lat,
                    pole_lon=pole_lon,
                    drone_lat=drone_lat,
                    drone_lon=drone_lon,
                    drone_alt=drone_alt,
                    drone_heading=drone_heading,
                    captured_at=captured_at
                )
                if not confirmed:
                    raise Exception("Upload confirmation failed")

                print(f"Photo uploaded successfully: {upload_info['photo_id']}")
                return upload_info['photo_id']

            except Exception as e:
                if attempt == max_retries - 1:
                    print(f"Photo upload failed after {max_retries} attempts: {e}")
                    return None

                # Exponential backoff
                delay = self.config.retry_base_delay * (2 ** attempt)
                print(f"Upload attempt {attempt + 1} failed, retrying in {delay}s...")
                time.sleep(delay)

        return None

    def upload_photo_from_file_s3(self, job_id: str, pole_id: str,
                                   photo_path: str) -> Optional[str]:
        """Upload photo file using S3 presigned URL flow.

        Convenience wrapper that reads a file and uploads via S3.

        Args:
            job_id: Job UUID
            pole_id: Pole UUID
            photo_path: Local path to JPEG file

        Returns:
            Photo ID if successful, None otherwise
        """
        try:
            with open(photo_path, 'rb') as f:
                image_data = f.read()
            return self.upload_photo_s3(job_id, pole_id, image_data)

        except Exception as e:
            print(f"Failed to read photo file: {e}")
            return None

    def update_status(self, job_id: str, status: 'MissionStatus') -> bool:
        """Update mission status on AeroSync.

        Args:
            job_id: Job UUID
            status: MissionStatus object

        Returns:
            True if successful
        """
        try:
            url = f"{self.config.base_url}/api/drone/jobs/{job_id}/status"

            # Match AEROSYNC_SPEC.md format
            payload = {
                'state': status.state.value if hasattr(status.state, 'value') else str(status.state),
                'guidance_state': getattr(status, 'guidance_state', 'FOLLOW'),
                'poles_inspected': status.poles_inspected,
                'elapsed_minutes': status.elapsed_minutes,
                'battery_percent': status.battery_percent,
                'coverage_percent': status.coverage_percent,
                'distance_from_home': status.distance_from_home,
                'current_position': getattr(status, 'current_position', None)
            }

            self._request_with_retry('PUT', url, json=payload)
            return True

        except Exception as e:
            print(f"Failed to update status: {e}")
            return False

    def complete_job(self, job_id: str, summary: 'MissionSummary') -> bool:
        """Mark job as complete and send summary.

        Args:
            job_id: Job UUID
            summary: MissionSummary object

        Returns:
            True if successful
        """
        try:
            url = f"{self.config.base_url}/api/drone/jobs/{job_id}/complete"
            # Match AEROSYNC_SPEC.md format
            payload = {
                'stop_reason': getattr(summary, 'stop_reason', 'completed'),
                'summary': summary.to_dict() if hasattr(summary, 'to_dict') else {
                    'poles_inspected': getattr(summary, 'poles_inspected', 0),
                    'poles_skipped_duplicate': getattr(summary, 'poles_skipped_duplicate', 0),
                    'duration_minutes': getattr(summary, 'duration_minutes', 0),
                    'final_battery_percent': getattr(summary, 'final_battery_percent', 0),
                    'final_coverage_percent': getattr(summary, 'final_coverage_percent', 0)
                }
            }
            self._request_with_retry('POST', url, json=payload)
            return True

        except Exception as e:
            print(f"Failed to complete job: {e}")
            return False

    # =========================================================================
    # WebSocket (Background Thread)
    # =========================================================================

    def connect_websocket(self, job_id: str):
        """Connect WebSocket for real-time updates (runs in background thread).

        Args:
            job_id: Job UUID
        """
        if not self.config.websocket_enabled:
            return

        if self._ws_running:
            return

        self._ws_running = True
        self._ws_thread = threading.Thread(
            target=self._ws_loop,
            args=(job_id,),
            daemon=True
        )
        self._ws_thread.start()

    def disconnect_websocket(self):
        """Disconnect WebSocket."""
        self._ws_running = False
        if self._ws_thread:
            self._ws_thread.join(timeout=2.0)
            self._ws_thread = None

    def _ws_loop(self, job_id: str):
        """WebSocket event loop (runs in background thread)."""
        if not AIOHTTP_AVAILABLE:
            print("Warning: aiohttp not available for WebSocket")
            return

        # Create new event loop for this thread
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        try:
            loop.run_until_complete(self._ws_handler(job_id))
        except Exception as e:
            print(f"WebSocket error: {e}")
        finally:
            loop.close()

    async def _ws_handler(self, job_id: str):
        """Async WebSocket handler."""
        # WebSocket endpoint - AeroSync uses Socket.IO on root
        ws_url = self.config.base_url.replace('https://', 'wss://').replace('http://', 'ws://')
        # Socket.IO connects to root, then joins rooms via events
        # For raw WebSocket fallback, use the job-specific endpoint
        ws_url = f"{ws_url}/socket.io/?job_id={job_id}&type=drone"

        headers = {'Authorization': f'Bearer {self.config.api_key}'}

        while self._ws_running:
            try:
                async with aiohttp.ClientSession() as session:
                    async with session.ws_connect(ws_url, headers=headers) as ws:
                        print(f"WebSocket connected to {ws_url}")

                        # Start send task
                        send_task = asyncio.create_task(self._ws_send_loop(ws))

                        # Receive loop
                        async for msg in ws:
                            if msg.type == aiohttp.WSMsgType.TEXT:
                                await self._handle_ws_message(msg.data)
                            elif msg.type == aiohttp.WSMsgType.ERROR:
                                print(f"WebSocket error: {ws.exception()}")
                                break

                        send_task.cancel()

            except Exception as e:
                print(f"WebSocket connection error: {e}")

            if self._ws_running:
                await asyncio.sleep(5.0)  # Reconnect delay

    async def _ws_send_loop(self, ws):
        """Send queued messages to WebSocket."""
        while True:
            try:
                # Check queue for messages to send
                while not self._ws_queue.empty():
                    msg = self._ws_queue.get_nowait()
                    await ws.send_str(json.dumps(msg))

                await asyncio.sleep(0.1)

            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"WebSocket send error: {e}")

    async def _handle_ws_message(self, data: str):
        """Handle incoming WebSocket message.

        Handles Socket.IO style events from AEROSYNC_SPEC.md:
        - abort: Operator requested mission abort
        - pause: Operator requested pause (custom)
        - resume: Operator requested resume (custom)
        """
        try:
            msg = json.loads(data)

            # Socket.IO messages may have 'event' or 'type' field
            event = msg.get('event', msg.get('type', ''))

            # Handle abort command (AEROSYNC_SPEC: Server → Drone)
            if event == 'abort':
                print("Received abort command from AeroSync")
                if self._on_abort:
                    self._on_abort()

            # Handle pause (custom extension)
            elif event in ('pause', 'pause_mission'):
                print("Received pause command from AeroSync")
                if self._on_pause:
                    self._on_pause()

            # Handle resume (custom extension)
            elif event in ('resume', 'resume_mission'):
                print("Received resume command from AeroSync")
                if self._on_resume:
                    self._on_resume()

            # Handle stop conditions update (custom extension)
            elif event == 'update_stop_conditions':
                if self._on_update_conditions:
                    self._on_update_conditions(msg.get('conditions', msg.get('data', {})))

            # Handle connected acknowledgment
            elif event == 'connected':
                print(f"WebSocket connected, sid: {msg.get('sid', 'unknown')}")

            # Handle joined acknowledgment
            elif event == 'joined':
                print(f"Joined job room: {msg.get('room', 'unknown')}")

        except json.JSONDecodeError:
            pass

    def send_position(self, lat: float, lon: float, alt: float,
                      heading: float, speed: float,
                      battery: float = -1.0, job_id: str = ''):
        """Queue position update for WebSocket.

        Matches AEROSYNC_SPEC.md 'drone_position' event format.

        Args:
            lat: Latitude (degrees)
            lon: Longitude (degrees)
            alt: Altitude AGL (meters)
            heading: Heading (degrees, 0=North)
            speed: Ground speed (m/s)
            battery: Battery percentage (0-100)
            job_id: Job UUID (optional, for context)
        """
        # Match AEROSYNC_SPEC.md drone_position event format
        self._ws_queue.put({
            'event': 'drone_position',
            'job_id': job_id,
            'lat': lat,
            'lon': lon,
            'alt': alt,
            'heading': heading,
            'speed': speed,
            'battery': battery,
            'timestamp': time.time()
        })

    def send_pole_found(self, result: 'InspectionResult'):
        """Queue pole found event for WebSocket.

        Args:
            result: InspectionResult
        """
        self._ws_queue.put({
            'type': 'pole_found',
            'pole_id': result.global_pole_id,
            'lat': result.position_gps[0],
            'lon': result.position_gps[1],
            'confidence': result.confidence,
            'pole_type': result.pole_type,
            'timestamp': time.time()
        })

    def send_status_update(self, status: 'MissionStatus'):
        """Queue status update for WebSocket.

        Args:
            status: MissionStatus
        """
        self._ws_queue.put({
            'type': 'status_update',
            'state': status.state.value,
            'battery': status.battery_percent,
            'coverage': status.coverage_percent,
            'poles_found': status.poles_inspected,
            'timestamp': time.time()
        })

    # =========================================================================
    # Callbacks
    # =========================================================================

    def set_on_pause(self, callback: Callable):
        """Set callback for pause command from AeroSync."""
        self._on_pause = callback

    def set_on_resume(self, callback: Callable):
        """Set callback for resume command from AeroSync."""
        self._on_resume = callback

    def set_on_abort(self, callback: Callable):
        """Set callback for abort command from AeroSync."""
        self._on_abort = callback

    def set_on_update_conditions(self, callback: Callable[[dict], None]):
        """Set callback for stop conditions update from AeroSync."""
        self._on_update_conditions = callback

    # =========================================================================
    # Cleanup
    # =========================================================================

    def close(self):
        """Close all connections."""
        self.disconnect_websocket()
        if self._session:
            self._session.close()
            self._session = None


# =============================================================================
# Convenience Functions
# =============================================================================

def create_aerosync_client(config_path: str = 'config/settings.yaml') -> AeroSyncClient:
    """Create AeroSyncClient with config from file."""
    return AeroSyncClient(config_path=config_path)


# =============================================================================
# Test / Mock Client
# =============================================================================

class MockAeroSyncClient:
    """Mock client for testing without real AeroSync connection."""

    def __init__(self):
        self.jobs: Dict[str, Job] = {}
        self.reported_poles: List[dict] = []
        self.uploaded_photos: List[dict] = []
        self.status_updates: List[dict] = []
        self.positions: List[dict] = []

    def add_mock_job(self, job: Job):
        """Add a mock job for testing."""
        self.jobs[job.job_id] = job

    def fetch_job(self, job_id: str) -> Optional[Job]:
        return self.jobs.get(job_id)

    def start_job(self, job_id: str) -> bool:
        return job_id in self.jobs

    def report_pole(self, job_id: str, result: 'InspectionResult') -> Optional[str]:
        self.reported_poles.append({
            'job_id': job_id,
            'pole_id': result.global_pole_id,
            'position_gps': result.position_gps,
            'timestamp': result.timestamp
        })
        return result.global_pole_id

    def upload_photo(self, job_id: str, pole_id: str, photo_path: str) -> Optional[str]:
        photo_id = f"photo-{len(self.uploaded_photos)}"
        self.uploaded_photos.append({
            'job_id': job_id,
            'pole_id': pole_id,
            'photo_path': photo_path,
            'photo_id': photo_id
        })
        return photo_id

    def update_status(self, job_id: str, status: 'MissionStatus') -> bool:
        self.status_updates.append({
            'job_id': job_id,
            'status': status
        })
        return True

    def complete_job(self, job_id: str, summary: 'MissionSummary') -> bool:
        return True

    def send_position(self, lat: float, lon: float, alt: float,
                      heading: float, speed: float):
        self.positions.append({
            'lat': lat, 'lon': lon, 'alt': alt,
            'heading': heading, 'speed': speed
        })

    def get_reported_poles(self) -> List[dict]:
        return self.reported_poles

    def get_uploaded_photos(self) -> List[dict]:
        return self.uploaded_photos

    def close(self):
        pass


# =============================================================================
# Test
# =============================================================================

if __name__ == '__main__':
    print("AeroSync Client Test")
    print("=" * 50)

    # Test with mock client
    mock = MockAeroSyncClient()

    # Add mock job
    job = Job(
        job_id="test-job-001",
        name="Test Survey",
        search_pattern="line_follow",
        search_config={},
        stop_conditions={'max_poles': 10},
        inspection_config={},
        previously_inspected=[]
    )
    mock.add_mock_job(job)

    # Fetch job
    fetched = mock.fetch_job("test-job-001")
    print(f"Fetched job: {fetched.name if fetched else 'None'}")

    # Start job
    started = mock.start_job("test-job-001")
    print(f"Started job: {started}")

    # Mock pole report
    from dataclasses import dataclass

    @dataclass
    class MockResult:
        global_pole_id: str = "pole-001"
        timestamp_iso: str = "2026-01-16T12:00:00Z"
        timestamp: float = time.time()
        position_gps: tuple = (41.123, -105.456, 2000.0)
        position_local: tuple = (50.0, 2.0, 10.0)
        confidence: float = 0.85
        pole_type: str = "h_frame"
        lidar_confidence: float = 0.9
        camera_confidence: float = 0.8
        pole_height: float = 15.0
        pole_radius: float = 0.15
        metadata: dict = None

        def __post_init__(self):
            if self.metadata is None:
                self.metadata = {}

    result = MockResult()
    pole_id = mock.report_pole("test-job-001", result)
    print(f"Reported pole: {pole_id}")

    # Check what was reported
    print(f"\nReported poles: {len(mock.get_reported_poles())}")
    for p in mock.get_reported_poles():
        print(f"  - {p['pole_id']} at {p['position_gps']}")
