"""
Fake AeroSync Server for Testing

A mock HTTP/WebSocket server that simulates the AeroSync API for offline testing.
Allows testing the full mission flow without a real AeroSync connection.

Features:
- REST API endpoints matching AeroSync
- WebSocket for real-time updates
- Job creation and management
- Pole storage and retrieval
- Photo upload simulation

Usage:
    # Start server
    python tests/fake_aerosync_server.py --port 8080

    # Or programmatically
    from fake_aerosync_server import FakeAeroSyncServer
    server = FakeAeroSyncServer(port=8080)
    server.start()

    # Create a job
    server.create_job(job_id="test-001", name="Test Mission")

    # Check received data
    print(server.get_poles("test-001"))
"""

import argparse
import json
import threading
import time
import uuid
from dataclasses import dataclass, field
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Dict, List, Optional
from urllib.parse import urlparse, parse_qs

# Try to import websockets for WebSocket support
try:
    import asyncio
    import websockets
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False


# =============================================================================
# Data Storage
# =============================================================================

@dataclass
class FakeJob:
    """Mock job data."""
    job_id: str
    name: str
    search_pattern: str = "line_follow"
    search_config: dict = field(default_factory=dict)
    stop_conditions: dict = field(default_factory=lambda: {
        'max_poles': 50,
        'max_time_minutes': 30,
        'min_battery_percent': 25.0
    })
    inspection_config: dict = field(default_factory=lambda: {
        'min_confidence': 0.6,
        'camera_required': True,
        'capture_distance': 10.0
    })
    previously_inspected: List[dict] = field(default_factory=list)
    state: str = "pending"
    started_at: Optional[float] = None
    completed_at: Optional[float] = None

    def to_dict(self) -> dict:
        return {
            'job_id': self.job_id,
            'name': self.name,
            'search_pattern': self.search_pattern,
            'search_config': self.search_config,
            'stop_conditions': self.stop_conditions,
            'inspection_config': self.inspection_config,
            'previously_inspected': self.previously_inspected,
            'state': self.state,
            'started_at': self.started_at,
            'completed_at': self.completed_at
        }


@dataclass
class FakePole:
    """Mock pole data."""
    pole_id: str
    job_id: str
    timestamp: str
    position: dict
    detection: dict
    photos: List[str] = field(default_factory=list)
    metadata: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            'pole_id': self.pole_id,
            'job_id': self.job_id,
            'timestamp': self.timestamp,
            'position': self.position,
            'detection': self.detection,
            'photos': self.photos,
            'metadata': self.metadata
        }


class DataStore:
    """In-memory data storage for fake server."""

    def __init__(self):
        self.jobs: Dict[str, FakeJob] = {}
        self.poles: Dict[str, Dict[str, FakePole]] = {}  # job_id -> {pole_id -> pole}
        self.photos: Dict[str, Dict[str, List[str]]] = {}  # job_id -> {pole_id -> [photo_ids]}
        self.status_updates: List[dict] = []
        self.positions: List[dict] = []
        self._lock = threading.Lock()

    def create_job(self, job_id: str, name: str, **kwargs) -> FakeJob:
        """Create a new job."""
        with self._lock:
            job = FakeJob(job_id=job_id, name=name, **kwargs)
            self.jobs[job_id] = job
            self.poles[job_id] = {}
            self.photos[job_id] = {}
            return job

    def get_job(self, job_id: str) -> Optional[FakeJob]:
        """Get job by ID."""
        return self.jobs.get(job_id)

    def start_job(self, job_id: str) -> bool:
        """Mark job as started."""
        with self._lock:
            if job_id in self.jobs:
                self.jobs[job_id].state = "running"
                self.jobs[job_id].started_at = time.time()
                return True
            return False

    def complete_job(self, job_id: str, summary: dict) -> bool:
        """Mark job as complete."""
        with self._lock:
            if job_id in self.jobs:
                self.jobs[job_id].state = "completed"
                self.jobs[job_id].completed_at = time.time()
                return True
            return False

    def add_pole(self, job_id: str, pole_data: dict) -> str:
        """Add a pole to a job."""
        with self._lock:
            if job_id not in self.jobs:
                return None

            pole_id = pole_data.get('pole_id', str(uuid.uuid4()))
            pole = FakePole(
                pole_id=pole_id,
                job_id=job_id,
                timestamp=pole_data.get('timestamp', ''),
                position=pole_data.get('position', {}),
                detection=pole_data.get('detection', {}),
                metadata=pole_data.get('metadata', {})
            )
            self.poles[job_id][pole_id] = pole
            self.photos[job_id][pole_id] = []
            return pole_id

    def add_photo(self, job_id: str, pole_id: str, photo_id: str) -> bool:
        """Add a photo to a pole."""
        with self._lock:
            if job_id in self.photos and pole_id in self.photos[job_id]:
                self.photos[job_id][pole_id].append(photo_id)
                return True
            return False

    def update_status(self, job_id: str, status: dict):
        """Record status update."""
        with self._lock:
            self.status_updates.append({
                'job_id': job_id,
                'status': status,
                'timestamp': time.time()
            })

    def add_position(self, position: dict):
        """Record position update."""
        with self._lock:
            self.positions.append(position)

    def get_poles(self, job_id: str) -> List[dict]:
        """Get all poles for a job."""
        if job_id not in self.poles:
            return []
        return [p.to_dict() for p in self.poles[job_id].values()]

    def get_stats(self) -> dict:
        """Get server statistics."""
        return {
            'total_jobs': len(self.jobs),
            'total_poles': sum(len(poles) for poles in self.poles.values()),
            'total_status_updates': len(self.status_updates),
            'total_positions': len(self.positions)
        }


# =============================================================================
# HTTP Request Handler
# =============================================================================

class FakeAeroSyncHandler(BaseHTTPRequestHandler):
    """HTTP request handler for fake AeroSync API."""

    # Reference to data store (set by server)
    data_store: DataStore = None

    def log_message(self, format, *args):
        """Suppress default logging."""
        pass

    def _send_json(self, data: dict, status: int = 200):
        """Send JSON response."""
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def _send_error(self, message: str, status: int = 400):
        """Send error response."""
        self._send_json({'error': message}, status)

    def _read_json_body(self) -> Optional[dict]:
        """Read and parse JSON request body."""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                return {}
            body = self.rfile.read(content_length)
            return json.loads(body.decode())
        except Exception:
            return None

    def _parse_path(self) -> tuple:
        """Parse URL path into components."""
        parsed = urlparse(self.path)
        parts = [p for p in parsed.path.split('/') if p]
        return parts, parse_qs(parsed.query)

    def do_OPTIONS(self):
        """Handle CORS preflight."""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type, Authorization')
        self.end_headers()

    def do_GET(self):
        """Handle GET requests."""
        parts, query = self._parse_path()

        # GET /api/jobs/{job_id}
        if len(parts) == 3 and parts[0] == 'api' and parts[1] == 'jobs':
            job_id = parts[2]
            job = self.data_store.get_job(job_id)
            if job:
                self._send_json(job.to_dict())
            else:
                self._send_error(f"Job {job_id} not found", 404)
            return

        # GET /api/jobs/{job_id}/poles
        if len(parts) == 4 and parts[0] == 'api' and parts[1] == 'jobs' and parts[3] == 'poles':
            job_id = parts[2]
            poles = self.data_store.get_poles(job_id)
            self._send_json({'poles': poles})
            return

        # GET /api/stats
        if parts == ['api', 'stats']:
            self._send_json(self.data_store.get_stats())
            return

        self._send_error("Not found", 404)

    def do_POST(self):
        """Handle POST requests."""
        parts, query = self._parse_path()
        body = self._read_json_body()

        # POST /api/jobs/{job_id}/start
        if len(parts) == 4 and parts[0] == 'api' and parts[1] == 'jobs' and parts[3] == 'start':
            job_id = parts[2]
            if self.data_store.start_job(job_id):
                self._send_json({'status': 'started'})
            else:
                self._send_error(f"Job {job_id} not found", 404)
            return

        # POST /api/jobs/{job_id}/poles
        if len(parts) == 4 and parts[0] == 'api' and parts[1] == 'jobs' and parts[3] == 'poles':
            job_id = parts[2]
            if body:
                pole_id = self.data_store.add_pole(job_id, body)
                if pole_id:
                    self._send_json({'pole_id': pole_id})
                    print(f"[AeroSync] Pole reported: {pole_id[:8]}... at "
                          f"({body.get('position', {}).get('latitude', 0):.4f}, "
                          f"{body.get('position', {}).get('longitude', 0):.4f})")
                else:
                    self._send_error(f"Job {job_id} not found", 404)
            else:
                self._send_error("Invalid JSON body")
            return

        # POST /api/jobs/{job_id}/poles/{pole_id}/photo
        if len(parts) == 6 and parts[0] == 'api' and parts[1] == 'jobs' and parts[3] == 'poles' and parts[5] == 'photo':
            job_id = parts[2]
            pole_id = parts[4]
            photo_id = str(uuid.uuid4())
            if self.data_store.add_photo(job_id, pole_id, photo_id):
                self._send_json({'photo_id': photo_id})
                print(f"[AeroSync] Photo uploaded: {photo_id[:8]}... for pole {pole_id[:8]}...")
            else:
                self._send_error("Job or pole not found", 404)
            return

        # POST /api/jobs/{job_id}/complete
        if len(parts) == 4 and parts[0] == 'api' and parts[1] == 'jobs' and parts[3] == 'complete':
            job_id = parts[2]
            if self.data_store.complete_job(job_id, body or {}):
                self._send_json({'status': 'completed'})
                print(f"[AeroSync] Job completed: {job_id}")
            else:
                self._send_error(f"Job {job_id} not found", 404)
            return

        # POST /api/jobs (create job - for testing)
        if parts == ['api', 'jobs'] and body:
            job_id = body.get('job_id', str(uuid.uuid4()))
            name = body.get('name', 'Unnamed Job')
            job = self.data_store.create_job(job_id, name, **{
                k: v for k, v in body.items() if k not in ['job_id', 'name']
            })
            self._send_json(job.to_dict())
            print(f"[AeroSync] Job created: {job_id} - {name}")
            return

        self._send_error("Not found", 404)

    def do_PUT(self):
        """Handle PUT requests."""
        parts, query = self._parse_path()
        body = self._read_json_body()

        # PUT /api/jobs/{job_id}/status
        if len(parts) == 4 and parts[0] == 'api' and parts[1] == 'jobs' and parts[3] == 'status':
            job_id = parts[2]
            if body:
                self.data_store.update_status(job_id, body)
                self._send_json({'status': 'updated'})
            else:
                self._send_error("Invalid JSON body")
            return

        self._send_error("Not found", 404)


# =============================================================================
# Fake Server
# =============================================================================

class FakeAeroSyncServer:
    """Fake AeroSync server for testing."""

    def __init__(self, host: str = '0.0.0.0', port: int = 8080):
        """Initialize fake server.

        Args:
            host: Host to bind to
            port: Port to listen on
        """
        self.host = host
        self.port = port
        self.data_store = DataStore()

        # Set up handler with data store reference
        FakeAeroSyncHandler.data_store = self.data_store

        self._http_server: Optional[HTTPServer] = None
        self._http_thread: Optional[threading.Thread] = None
        self._running = False

    def start(self):
        """Start the server (non-blocking)."""
        if self._running:
            return

        self._running = True
        self._http_server = HTTPServer((self.host, self.port), FakeAeroSyncHandler)

        self._http_thread = threading.Thread(target=self._http_loop, daemon=True)
        self._http_thread.start()

        print(f"[AeroSync] Fake server started on http://{self.host}:{self.port}")

    def _http_loop(self):
        """HTTP server loop."""
        while self._running:
            self._http_server.handle_request()

    def stop(self):
        """Stop the server."""
        self._running = False
        if self._http_server:
            self._http_server.shutdown()
        if self._http_thread:
            self._http_thread.join(timeout=2.0)

    # Convenience methods for testing
    def create_job(self, job_id: str, name: str, **kwargs) -> FakeJob:
        """Create a job directly (for test setup)."""
        return self.data_store.create_job(job_id, name, **kwargs)

    def get_job(self, job_id: str) -> Optional[FakeJob]:
        """Get job by ID."""
        return self.data_store.get_job(job_id)

    def get_poles(self, job_id: str) -> List[dict]:
        """Get all poles for a job."""
        return self.data_store.get_poles(job_id)

    def get_stats(self) -> dict:
        """Get server statistics."""
        return self.data_store.get_stats()

    def get_base_url(self) -> str:
        """Get base URL for this server."""
        return f"http://{self.host}:{self.port}"


# =============================================================================
# CLI Entry Point
# =============================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Fake AeroSync Server')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=8080, help='Port to listen on')
    parser.add_argument('--create-test-job', action='store_true',
                        help='Create a test job on startup')
    args = parser.parse_args()

    # Create and start server
    server = FakeAeroSyncServer(host=args.host, port=args.port)
    server.start()

    # Create test job if requested
    if args.create_test_job:
        server.create_job(
            job_id="test-job-001",
            name="Test Survey Mission",
            stop_conditions={
                'max_poles': 20,
                'max_time_minutes': 15,
                'min_battery_percent': 25.0
            }
        )
        print(f"[AeroSync] Created test job: test-job-001")

    print(f"\n[AeroSync] Server running. Press Ctrl+C to stop.\n")
    print(f"API endpoints:")
    print(f"  GET  /api/jobs/{{job_id}}           - Fetch job")
    print(f"  POST /api/jobs/{{job_id}}/start     - Start job")
    print(f"  POST /api/jobs/{{job_id}}/poles     - Report pole")
    print(f"  POST /api/jobs/{{job_id}}/complete  - Complete job")
    print(f"  GET  /api/stats                    - Server stats")
    print(f"  POST /api/jobs                     - Create job (testing)")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[AeroSync] Shutting down...")
        server.stop()
