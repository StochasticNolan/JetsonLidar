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
    """Job data from AeroSync."""
    job_id: str
    name: str
    search_pattern: str
    search_config: dict
    stop_conditions: dict
    inspection_config: dict
    previously_inspected: List[dict]

    @classmethod
    def from_dict(cls, data: dict) -> 'Job':
        return cls(
            job_id=data.get('job_id', ''),
            name=data.get('name', ''),
            search_pattern=data.get('search_pattern', 'line_follow'),
            search_config=data.get('search_config', {}),
            stop_conditions=data.get('stop_conditions', {}),
            inspection_config=data.get('inspection_config', {}),
            previously_inspected=data.get('previously_inspected', [])
        )


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
            url = f"{self.config.base_url}/api/jobs/{job_id}"
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
            url = f"{self.config.base_url}/api/jobs/{job_id}/start"
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
            url = f"{self.config.base_url}/api/jobs/{job_id}/poles"

            payload = {
                'pole_id': result.global_pole_id,
                'timestamp': result.timestamp_iso,
                'position': {
                    'latitude': result.position_gps[0],
                    'longitude': result.position_gps[1],
                    'altitude_msl': result.position_gps[2],
                    'local_x': result.position_local[0],
                    'local_y': result.position_local[1],
                    'local_z': result.position_local[2]
                },
                'detection': {
                    'confidence': result.confidence,
                    'pole_type': result.pole_type,
                    'lidar_confidence': result.lidar_confidence,
                    'camera_confidence': result.camera_confidence,
                    'height_estimate': result.pole_height,
                    'radius_estimate': result.pole_radius
                },
                'metadata': result.metadata
            }

            response = self._request_with_retry('POST', url, json=payload)
            data = response.json()
            return data.get('pole_id', result.global_pole_id)

        except Exception as e:
            print(f"Failed to report pole: {e}")
            return None

    def upload_photo(self, job_id: str, pole_id: str, photo_path: str) -> Optional[str]:
        """Upload a pole photo to AeroSync.

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
            url = f"{self.config.base_url}/api/jobs/{job_id}/poles/{pole_id}/photo"

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

    def update_status(self, job_id: str, status: 'MissionStatus') -> bool:
        """Update mission status on AeroSync.

        Args:
            job_id: Job UUID
            status: MissionStatus object

        Returns:
            True if successful
        """
        try:
            url = f"{self.config.base_url}/api/jobs/{job_id}/status"

            payload = {
                'state': status.state.value,
                'poles_inspected': status.poles_inspected,
                'elapsed_minutes': status.elapsed_minutes,
                'battery_percent': status.battery_percent,
                'distance_from_home': status.distance_from_home,
                'coverage_percent': status.coverage_percent
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
            url = f"{self.config.base_url}/api/jobs/{job_id}/complete"
            payload = summary.to_dict()
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
        ws_url = self.config.base_url.replace('https://', 'wss://').replace('http://', 'ws://')
        ws_url = f"{ws_url}/api/jobs/{job_id}/live"

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
        """Handle incoming WebSocket message."""
        try:
            msg = json.loads(data)
            msg_type = msg.get('type', '')

            if msg_type == 'pause_mission':
                if self._on_pause:
                    self._on_pause()

            elif msg_type == 'resume_mission':
                if self._on_resume:
                    self._on_resume()

            elif msg_type == 'abort_mission':
                if self._on_abort:
                    self._on_abort()

            elif msg_type == 'update_stop_conditions':
                if self._on_update_conditions:
                    self._on_update_conditions(msg.get('conditions', {}))

        except json.JSONDecodeError:
            pass

    def send_position(self, lat: float, lon: float, alt: float,
                      heading: float, speed: float):
        """Queue position update for WebSocket.

        Args:
            lat: Latitude (degrees)
            lon: Longitude (degrees)
            alt: Altitude MSL (meters)
            heading: Heading (degrees)
            speed: Ground speed (m/s)
        """
        self._ws_queue.put({
            'type': 'position_update',
            'lat': lat,
            'lon': lon,
            'alt': alt,
            'heading': heading,
            'speed': speed,
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
