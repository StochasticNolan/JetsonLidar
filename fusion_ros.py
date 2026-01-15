#!/usr/bin/env python3
"""
LiDAR + Camera Fusion Viewer
Clean side-by-side visualization with C++-style LiDAR rendering
Now with YOLO pole detection!
"""

import cv2
import numpy as np
import time
import sys
import fcntl
import struct
from pathlib import Path
from collections import deque

# ROS imports
try:
    import rospy
    from sensor_msgs.msg import PointCloud2, Image
    import sensor_msgs.point_cloud2 as pc2
    from cv_bridge import CvBridge
    ROS_AVAILABLE = True
    CV_BRIDGE_AVAILABLE = True
except ImportError as e:
    if 'cv_bridge' in str(e):
        print("WARNING: cv_bridge not available - camera won't be in rosbag")
        import rospy
        from sensor_msgs.msg import PointCloud2
        import sensor_msgs.point_cloud2 as pc2
        ROS_AVAILABLE = True
        CV_BRIDGE_AVAILABLE = False
    else:
        print("ERROR: ROS not available")
        ROS_AVAILABLE = False
        sys.exit(1)

# YOLOv5 imports
YOLO_AVAILABLE = False
try:
    import torch
    sys.path.insert(0, '/home/nvidia/Projects/JetsonLidar/yolov5')
    YOLO_AVAILABLE = True
except ImportError:
    print("WARNING: PyTorch not available - detection disabled")

# LiDAR pole detector
LIDAR_DETECTOR_AVAILABLE = False
try:
    sys.path.insert(0, '/home/nvidia/Projects/JetsonLidar/perception_nav/src')
    from pole_detector import PoleDetector, PoleDetectorConfig
    LIDAR_DETECTOR_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: LiDAR pole detector not available: {e}")

# Sensor fusion
FUSION_AVAILABLE = False
try:
    from fusion import SensorFusion, FusionConfig
    FUSION_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Sensor fusion not available: {e}")

# Line follower (for single-line pole following)
LINE_FOLLOWER_AVAILABLE = False
try:
    from line_follower import LineFollower, LineFollowerConfig
    LINE_FOLLOWER_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Line follower not available: {e}")

# MAVLink guidance (Cube Blue FCU)
GUIDANCE_AVAILABLE = False
try:
    from guidance import MavlinkGuidance, GuidanceConfig, GuidanceState
    GUIDANCE_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: MAVLink guidance not available: {e}")

# Tesla-style smooth controller
SMOOTH_CONTROLLER_AVAILABLE = False
try:
    from smooth_controller import SmoothController, SmoothControllerConfig
    SMOOTH_CONTROLLER_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Smooth controller not available: {e}")

# Capture system (scheduler + bundler + obstacle detection)
CAPTURE_SYSTEM_AVAILABLE = False
try:
    from capture_system import CaptureManager, ObstacleState
    CAPTURE_SYSTEM_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Capture system not available: {e}")

# Model paths (in order of preference - newest first, then smallest/fastest)
MODEL_PATHS = [
    "/home/nvidia/Projects/JetsonLidar/yolov5/runs/train/wyoming_v13_fast/weights/best.pt",  # v13 with demo pole
    "/home/nvidia/Projects/JetsonLidar/runs/train/wyoming_v12_fast/weights/best.pt",  # v12 fast nano
    "/home/nvidia/Projects/JetsonLidar/runs/train/wyoming_v12_orin/weights/best.pt",  # v12 with indoor pole
    "/home/nvidia/Projects/JetsonLidar/runs/train/wyoming_nano_optimized/weights/best.pt",
    "/home/nvidia/Projects/JetsonLidar/runs/train/wyoming_infra_nano/weights/best.pt",
]

# Pole class names from training
POLE_CLASSES = ['Metal h-frame', 'Metal tree pole', 'Single pole',
                'Wood distribution Pole', 'h-frame']

# Colors for each class (BGR)
POLE_COLORS = [
    (0, 255, 0),    # Green
    (255, 128, 0),  # Blue-ish
    (0, 255, 255),  # Yellow
    (255, 0, 255),  # Magenta
    (0, 165, 255),  # Orange
]


class FusionViewer:
    """Clean LiDAR + Camera viewer with C++-style rendering"""

    def __init__(self, lidar_topic="/aeva_0/points", camera_device=0,
                 replay_mode=False, camera_topic="/camera/image_raw"):
        # LiDAR state
        self.lidar_topic = lidar_topic
        self.latest_points = None
        self.lidar_hz = 0.0
        self.last_lidar_time = time.time()

        # Camera state
        self.camera_device = camera_device
        self.cap = None
        self.camera_available = False

        # Replay mode (use ROS camera topic instead of live camera)
        self.replay_mode = replay_mode
        self.camera_topic = camera_topic
        self.camera_subscriber = None
        self.latest_camera_frame = None
        self.camera_frame_time = 0

        # View parameters - slight overhead perspective
        self.scale = 150.0          # pixels per meter (scaled to match camera FOV)
        self.point_size = 1         # single pixel for crisp dense clouds
        self.rotation_x = 15.0      # degrees (positive = overhead looking down)
        self.rotation_y = 180.0     # degrees (rotated to face front)

        # Mouse interaction
        self.mouse_down = False
        self.last_mouse_x = 0
        self.last_mouse_y = 0
        self.mouse_in_lidar = False  # Track which panel mouse is in

        # Performance
        self.fps_history = deque(maxlen=30)
        self.last_time = time.time()
        self.frame_count = 0

        # Window dimensions (same height for both to avoid stretching)
        self.panel_height = 600
        self.lidar_width = 800
        self.camera_width = 800

        # Detection state
        self.detector = None
        self.detection_enabled = True
        self.detection_conf = 0.30  # Confidence threshold (0.0-1.0)
        self.last_detections = []  # Cache detections for display
        self.detection_hz = 0.0
        self.last_detection_time = time.time()

        # Camera exposure (V4L2)
        self.camera_exposure = 1000  # Default exposure value

        # LiDAR pole detection state
        self.lidar_detector = None
        self.lidar_detection_enabled = True
        self.lidar_poles = []
        self.lidar_detection_hz = 0.0
        self.last_lidar_detection_time = time.time()

        # Sensor fusion state
        self.sensor_fusion = None
        self.fusion_enabled = True
        self.fused_poles = []
        self.fusion_hz = 0.0

        # Line follower state
        self.line_follower = None
        self.line_state = None

        # Guidance/navigation state
        self.guidance = None
        self.nav_enabled = False  # OFF by default for safety
        self.fcu_connected = False
        self.fcu_port = '/dev/ttyACM0'  # USB port for CubeOrange+

        # Tesla-style smooth controller
        self.smooth_controller = None
        self.smooth_mode = True  # Use smooth controller vs simple controller

        # Capture system (scheduler + bundler + obstacle detection)
        self.capture_manager = None
        self.obstacle_state = None
        self.last_capture_event = None

        # Training data recording (images + metadata)
        self.recording = False
        self.recording_dir = None
        self.recording_frame_count = 0
        self.recording_start_time = None

        # Rosbag recording for replay
        self.rosbag_process = None
        self.rosbag_recording = False

        # Camera ROS publisher (for rosbag recording)
        self.camera_pub = None
        self.cv_bridge = None
        if CV_BRIDGE_AVAILABLE:
            self.cv_bridge = CvBridge()

    def on_confidence_change(self, val):
        """Trackbar callback for confidence slider"""
        self.detection_conf = val / 100.0
        if self.detector:
            self.detector.conf = self.detection_conf

    def on_exposure_change(self, val):
        """Trackbar callback for exposure slider"""
        # Scale slider value (0-100) to exposure range (100-5000)
        self.camera_exposure = 100 + int(val * 49)  # 0->100, 100->5000
        self.set_camera_exposure(self.camera_exposure)

    def set_camera_exposure(self, exposure_val):
        """Set camera exposure using V4L2 ioctl"""
        try:
            V4L2_CID_EXPOSURE_AUTO = 0x009a0901
            V4L2_CID_EXPOSURE_ABSOLUTE = 0x009a0902
            V4L2_CID_FOCUS_AUTO = 0x009a090c
            VIDIOC_S_CTRL = 0xc008561c
            fd = open('/dev/video0', 'rb+', buffering=0)
            # Set manual exposure mode
            fcntl.ioctl(fd, VIDIOC_S_CTRL, struct.pack('Ii', V4L2_CID_EXPOSURE_AUTO, 1))
            # Set exposure value
            fcntl.ioctl(fd, VIDIOC_S_CTRL, struct.pack('Ii', V4L2_CID_EXPOSURE_ABSOLUTE, exposure_val))
            # Ensure autofocus is enabled
            fcntl.ioctl(fd, VIDIOC_S_CTRL, struct.pack('Ii', V4L2_CID_FOCUS_AUTO, 1))
            fd.close()
        except Exception as e:
            pass  # Silently fail if camera is busy

    def init_ros(self):
        """Initialize ROS subscriber and camera publisher"""
        rospy.init_node('fusion_viewer', anonymous=True)
        self.subscriber = rospy.Subscriber(
            self.lidar_topic,
            PointCloud2,
            self.lidar_callback,
            queue_size=1,
            buff_size=2**24
        )
        print(f"Subscribed to: {self.lidar_topic}")

        # In replay mode, subscribe to camera topic instead of using live camera
        if self.replay_mode and CV_BRIDGE_AVAILABLE:
            self.camera_subscriber = rospy.Subscriber(
                self.camera_topic,
                Image,
                self.camera_callback,
                queue_size=1,
                buff_size=2**24
            )
            print(f"Subscribed to camera: {self.camera_topic}")
            self.camera_available = True  # Mark camera as available via ROS
        elif not self.replay_mode and CV_BRIDGE_AVAILABLE:
            # Create camera publisher for rosbag recording (live mode only)
            self.camera_pub = rospy.Publisher(
                '/camera/image_raw',
                Image,
                queue_size=1
            )
            print("Camera publisher created: /camera/image_raw")

    def init_camera(self):
        """Initialize USB camera (skipped in replay mode)"""
        # Skip camera init in replay mode - we get frames from ROS topic
        if self.replay_mode:
            print("Replay mode: camera will be received from ROS topic")
            return

        # Try different camera backends and devices
        backends = [
            (cv2.CAP_V4L2, "V4L2"),
            (cv2.CAP_ANY, "ANY"),
        ]
        devices = [self.camera_device, 1, 2]

        for dev in devices:
            for backend, backend_name in backends:
                print(f"Trying camera device {dev} with {backend_name} backend...")
                self.cap = cv2.VideoCapture(dev, backend)
                if self.cap.isOpened():
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                    self.cap.set(cv2.CAP_PROP_FPS, 30)

                    # Test read
                    ret, frame = self.cap.read()
                    if ret and frame is not None and frame.size > 0:
                        self.camera_available = True
                        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        print(f"Camera opened: device {dev} ({actual_w}x{actual_h}) with {backend_name}")
                        return
                    else:
                        self.cap.release()
                else:
                    pass

        print("No camera available")
        self.camera_available = False

    def init_detector(self):
        """Initialize YOLO pole detector"""
        if not YOLO_AVAILABLE:
            print("YOLO not available - skipping detector init")
            return

        for model_path in MODEL_PATHS:
            if Path(model_path).exists():
                print(f"Loading YOLO model: {Path(model_path).name}")
                try:
                    self.detector = torch.hub.load(
                        '/home/nvidia/Projects/JetsonLidar/yolov5',
                        'custom',
                        path=model_path,
                        source='local'
                    )
                    self.detector.conf = self.detection_conf
                    print(f"  Model loaded successfully")
                    return
                except Exception as e:
                    print(f"  Failed to load: {e}")
                    continue

        print("No YOLO model found - detection disabled")

    def init_lidar_detector(self):
        """Initialize LiDAR-based pole detector"""
        if not LIDAR_DETECTOR_AVAILABLE:
            print("LiDAR pole detector not available - skipping init")
            return

        try:
            # Configure for real-time performance
            config = PoleDetectorConfig()
            config.downsample_factor = 8      # Aggressive downsampling for speed
            config.max_points = 5000          # Cap points for bounded latency
            config.cluster_eps = 0.5          # Clustering radius
            config.cluster_min_samples = 5
            config.pole_min_height = 1.0      # At least 1m tall
            config.pole_max_radius = 0.5      # Up to 0.5m radius
            config.track_min_hits = 2         # Faster confirmation
            config.max_clusters = 10          # Limit clusters to process

            self.lidar_detector = PoleDetector(config=config)
            print("LiDAR pole detector initialized")
        except Exception as e:
            print(f"Failed to initialize LiDAR detector: {e}")
            self.lidar_detector = None

    def init_sensor_fusion(self):
        """Initialize sensor fusion module"""
        if not FUSION_AVAILABLE:
            print("Sensor fusion not available - skipping init")
            return

        try:
            # Configure fusion with camera intrinsics matching our camera
            config = FusionConfig()
            config.camera_fx = 800.0
            config.camera_fy = 800.0
            config.camera_cx = self.camera_width / 2
            config.camera_cy = self.panel_height / 2

            # Association threshold (pixels)
            config.association_threshold_px = 150.0

            # Confidence weights
            config.weight_lidar = 0.6
            config.weight_camera = 0.4
            config.camera_boost = 0.15

            # Temporal filtering
            config.min_frames_seen = 2
            config.min_fused_confidence = 0.25

            self.sensor_fusion = SensorFusion(config=config)
            print("Sensor fusion initialized")
        except Exception as e:
            print(f"Failed to initialize sensor fusion: {e}")
            self.sensor_fusion = None

    def init_line_follower(self):
        """Initialize line follower for single-line pole navigation"""
        if not LINE_FOLLOWER_AVAILABLE:
            print("Line follower not available - skipping init")
            return

        try:
            config = LineFollowerConfig()
            config.lateral_offset = 10.0      # Fly 10m to the side of poles
            config.fly_on_left = True         # Fly on left side
            config.look_ahead_distance = 20.0
            config.expected_pole_spacing = 75.0  # H-frame typical spacing

            self.line_follower = LineFollower(config=config)
            print("Line follower initialized")
        except Exception as e:
            print(f"Failed to initialize line follower: {e}")
            self.line_follower = None

    def init_guidance(self, port=None):
        """Initialize MAVLink guidance for Cube Blue FCU"""
        if not GUIDANCE_AVAILABLE:
            print("MAVLink guidance not available - skipping init")
            return False

        if port:
            self.fcu_port = port

        try:
            self.guidance = MavlinkGuidance()
            print(f"Connecting to FCU on {self.fcu_port}...")

            if self.guidance.connect(self.fcu_port, baud=57600):
                self.fcu_connected = True
                print(f"FCU connected on {self.fcu_port}")
                return True
            else:
                print(f"Failed to connect to FCU on {self.fcu_port}")
                self.fcu_connected = False
                return False
        except Exception as e:
            print(f"Failed to initialize guidance: {e}")
            self.guidance = None
            self.fcu_connected = False
            return False

    def run_line_following(self):
        """Run line follower on fused poles"""
        if self.line_follower is None:
            return None

        poles = self.fused_poles if self.fused_poles else self.lidar_poles
        if not poles:
            return None

        try:
            self.line_state = self.line_follower.update(poles)
            return self.line_state
        except Exception as e:
            print(f"Line following error: {e}")
            return None

    def run_navigation(self):
        """Run full navigation: line following -> guidance -> FCU"""
        if not self.nav_enabled:
            return

        if self.guidance is None or not self.fcu_connected:
            return

        # Get poles for navigation
        poles = self.fused_poles if self.fused_poles else self.lidar_poles

        # Get current velocity from smooth controller or default
        current_velocity = 0.0
        if self.smooth_controller:
            state = self.smooth_controller.get_state()
            if state:
                current_velocity = state.speed

        # Run capture system (scheduler + obstacle detection)
        if self.capture_manager:
            # Get current lidar points for obstacle detection
            lidar_pts = self.latest_points if self.latest_points is not None else None

            # Update capture system - returns (capture_event, obstacle_state)
            capture_event, obstacle_state = self.capture_manager.update(
                poles or [],
                velocity=current_velocity,
                lidar_points=lidar_pts
            )

            self.obstacle_state = obstacle_state
            self.last_capture_event = capture_event

            # Pass obstacle state to smooth controller
            if self.smooth_controller and obstacle_state:
                self.smooth_controller.update_obstacle_state(
                    obstacle_state.obstacle_detected,
                    obstacle_state.obstacle_distance
                )

            # Pass obstacle state to guidance for emergency stop failsafe
            if self.guidance and obstacle_state:
                self.guidance.update_obstacle(
                    obstacle_state.obstacle_detected,
                    obstacle_state.obstacle_distance
                )

            # Log captures
            if capture_event:
                print(f"CAPTURE: Pole {capture_event.pole_id} at distance {capture_event.distance:.1f}m")

        # Tesla-style smooth controller (100Hz separate thread)
        if self.smooth_mode and self.smooth_controller:
            # Just update the path - smooth controller runs at 100Hz
            if poles:
                self.smooth_controller.update_path(poles)
            return

        # Fallback: Simple line following (runs at perception rate)
        line_state = self.run_line_following()
        if line_state is None or not line_state.valid:
            self.guidance.stop()
            return

        try:
            from guidance import CorridorState
            corridor_state = CorridorState(
                lateral_error=line_state.lateral_error,
                heading_error=line_state.heading_error,
                look_ahead_x=line_state.look_ahead_x,
                look_ahead_y=line_state.look_ahead_y,
                confidence=line_state.confidence,
                num_poles=line_state.num_poles,
                corridor_width=0.0,
            )
            self.guidance.update(corridor_state)
        except Exception as e:
            print(f"Navigation error: {e}")
            self.guidance.stop()

    def init_smooth_controller(self):
        """Initialize Tesla-style smooth controller"""
        if not SMOOTH_CONTROLLER_AVAILABLE:
            print("Smooth controller not available - skipping init")
            return

        try:
            config = SmoothControllerConfig()
            config.lateral_offset = 10.0      # Fly 10m from poles
            config.fly_on_left = True
            config.cruise_speed = 3.0         # 3 m/s cruise
            config.max_speed = 5.0            # 5 m/s max
            config.control_rate_hz = 100.0    # 100Hz control loop

            self.smooth_controller = SmoothController(config)
            print("Smooth controller initialized (100Hz Tesla-style)")
        except Exception as e:
            print(f"Failed to initialize smooth controller: {e}")
            self.smooth_controller = None

    def init_capture_system(self):
        """Initialize capture scheduler + data bundler + obstacle detection"""
        if not CAPTURE_SYSTEM_AVAILABLE:
            print("Capture system not available - skipping init")
            return

        try:
            config = {
                'capture_distance': 10.0,       # Trigger capture at 10m
                'min_capture_interval': 2.0,    # Min 2s between captures
                'approach_velocity': 3.0,       # Default velocity estimate
                'output_dir': 'captures',       # Save to captures/
                'corridor_length': 15.0,        # Obstacle detection range
                'corridor_width': 2.0,          # Corridor half-width
                'altitude_min': -1.0,
                'altitude_max': 2.0,
                'min_obstacle_points': 10,
                'danger_distance': 5.0,
            }
            self.capture_manager = CaptureManager(config)
            self.capture_manager.start()
            print("Capture system initialized (scheduler + bundler + obstacles)")
        except Exception as e:
            print(f"Failed to initialize capture system: {e}")
            self.capture_manager = None

    def toggle_recording(self):
        """Toggle training data recording on/off (includes rosbag for replay)"""
        if self.recording:
            # Stop recording
            self.recording = False
            self.stop_rosbag()
            duration = time.time() - self.recording_start_time if self.recording_start_time else 0
            print(f"\n=== RECORDING STOPPED ===")
            print(f"Saved {self.recording_frame_count} frames in {duration:.1f}s")
            print(f"Location: {self.recording_dir}")
            print(f"")
            print(f"To replay this data:")
            print(f"  rosbag play {self.recording_dir}/*.bag")
            print(f"")
        else:
            # Start recording
            import os
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            self.recording_dir = f"flight_captures/{timestamp}"
            os.makedirs(f"{self.recording_dir}/images", exist_ok=True)
            os.makedirs(f"{self.recording_dir}/lidar", exist_ok=True)

            self.recording = True
            self.recording_frame_count = 0
            self.recording_start_time = time.time()

            # Start rosbag recording for replay capability
            self.start_rosbag()

            print(f"\n=== RECORDING STARTED ===")
            print(f"Saving to: {self.recording_dir}")
            print(f"  - Rosbag: LiDAR + camera topics (for replay)")
            print(f"  - Images: JPG frames (for training)")
            print(f"  - Metadata: JSON with detections")
            print(f"Press 'b' or Herelink C to stop")
            print(f"")

    def start_rosbag(self):
        """Start rosbag recording for LiDAR and camera replay"""
        if self.rosbag_recording:
            return

        import subprocess
        import os

        try:
            # Topics to record
            topics = [
                "/aeva_0/points",  # LiDAR point cloud
            ]

            # Add camera topic if cv_bridge is available
            if CV_BRIDGE_AVAILABLE and self.camera_pub is not None:
                topics.append("/camera/image_raw")

            # Build rosbag command
            bag_path = os.path.join(self.recording_dir, "flight")
            cmd = ["rosbag", "record", "-o", bag_path] + topics

            # Start rosbag as background process
            self.rosbag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid  # Create new process group for clean kill
            )
            self.rosbag_recording = True
            print(f"  Rosbag started (PID: {self.rosbag_process.pid})")
            print(f"  Recording topics: {', '.join(topics)}")

        except Exception as e:
            print(f"  Warning: Could not start rosbag: {e}")
            self.rosbag_process = None
            self.rosbag_recording = False

    def stop_rosbag(self):
        """Stop rosbag recording gracefully"""
        if not self.rosbag_recording or self.rosbag_process is None:
            return

        import os
        import signal

        try:
            # Send SIGINT to rosbag (like Ctrl+C) for clean shutdown
            os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGINT)

            # Wait for rosbag to finish writing
            self.rosbag_process.wait(timeout=5)
            print(f"  Rosbag stopped cleanly")

        except Exception as e:
            # Force kill if needed
            try:
                os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGKILL)
            except:
                pass
            print(f"  Rosbag stopped (forced)")

        self.rosbag_process = None
        self.rosbag_recording = False

    def save_recording_frame(self, frame, points):
        """Save a frame of training data (called from main loop)"""
        if not self.recording or self.recording_dir is None:
            return

        # Save every 3rd frame to avoid too much data (10fps from 30fps)
        if self.recording_frame_count % 3 != 0:
            self.recording_frame_count += 1
            return

        try:
            import os
            timestamp = time.time()
            frame_id = self.recording_frame_count

            # Save image
            if frame is not None:
                img_path = f"{self.recording_dir}/images/frame_{frame_id:06d}.jpg"
                cv2.imwrite(img_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 90])

            # Save LiDAR points as numpy array
            if points is not None and len(points) > 0:
                pts_path = f"{self.recording_dir}/lidar/frame_{frame_id:06d}.npy"
                np.save(pts_path, points.astype(np.float32))

            # Save metadata for this frame
            meta = {
                'frame_id': frame_id,
                'timestamp': timestamp,
                'num_points': len(points) if points is not None else 0,
                'detections': []
            }

            # Add YOLO detections if available
            if hasattr(self, 'yolo_detections') and self.yolo_detections:
                for det in self.yolo_detections:
                    meta['detections'].append({
                        'class': det.get('class', 'unknown'),
                        'confidence': float(det.get('confidence', 0)),
                        'bbox': det.get('bbox', [])
                    })

            # Add LiDAR pole detections if available
            if self.lidar_poles:
                meta['lidar_poles'] = []
                for pole in self.lidar_poles:
                    meta['lidar_poles'].append({
                        'id': pole.id,
                        'x': float(pole.x),
                        'y': float(pole.y),
                        'z': float(pole.z),
                        'confidence': float(pole.confidence)
                    })

            # Save metadata
            import json
            meta_path = f"{self.recording_dir}/lidar/frame_{frame_id:06d}.json"
            with open(meta_path, 'w') as f:
                json.dump(meta, f)

            self.recording_frame_count += 1

        except Exception as e:
            print(f"Recording error: {e}")

    def toggle_navigation(self):
        """Toggle navigation on/off with safety checks"""
        if not GUIDANCE_AVAILABLE:
            print("Guidance not available")
            return

        if self.guidance is None:
            print("Initializing guidance first...")
            if not self.init_guidance():
                print("Could not initialize guidance")
                return

        if not self.fcu_connected:
            print("FCU not connected - cannot enable navigation")
            return

        self.nav_enabled = not self.nav_enabled

        if self.nav_enabled:
            print("=== NAVIGATION ENABLED ===")
            if self.smooth_mode and self.smooth_controller:
                print("Using Tesla-style smooth control (100Hz)")
                self.smooth_controller.set_guidance(self.guidance)
                self.smooth_controller.start()
            else:
                print("Using simple control")
            print("Press 'n' to disable, 'q' to quit")
            print("Flip RC to GUIDED mode to activate")
            self.guidance.start_mission()
        else:
            print("=== NAVIGATION DISABLED ===")
            if self.smooth_controller and self.smooth_controller.is_running():
                self.smooth_controller.stop()
            self.guidance.stop()
            self.guidance.stop_mission()

    def run_fusion(self):
        """Run sensor fusion on LiDAR poles and camera detections"""
        if self.sensor_fusion is None or not self.fusion_enabled:
            return []

        if not self.lidar_poles:
            return []

        try:
            t_start = time.time()
            self.fused_poles = self.sensor_fusion.fuse(
                self.lidar_poles,
                self.last_detections,
                (self.panel_height, self.camera_width)
            )
            dt = time.time() - t_start
            if dt > 0:
                self.fusion_hz = 1.0 / dt
            return self.fused_poles
        except Exception as e:
            print(f"Fusion error: {e}")
            return []

    def run_lidar_detection(self):
        """Run LiDAR pole detection on latest point cloud"""
        if self.lidar_detector is None or not self.lidar_detection_enabled:
            return []

        if self.latest_points is None or len(self.latest_points) == 0:
            return []

        try:
            t_start = time.time()
            self.lidar_poles = self.lidar_detector.detect(self.latest_points)
            dt = time.time() - t_start
            if dt > 0:
                self.lidar_detection_hz = 1.0 / dt
            return self.lidar_poles
        except Exception as e:
            print(f"LiDAR detection error: {e}")
            return []

    def run_detection(self, frame):
        """Run YOLO detection on frame, return list of detections"""
        if self.detector is None or not self.detection_enabled:
            return []

        try:
            start = time.time()
            results = self.detector(frame)
            detections = results.xyxy[0].cpu().numpy()

            # Update detection rate
            dt = time.time() - start
            if dt > 0:
                self.detection_hz = 1.0 / dt

            # Convert to list of dicts
            det_list = []
            for det in detections:
                x1, y1, x2, y2, conf, cls_id = det
                det_list.append({
                    'bbox': (int(x1), int(y1), int(x2), int(y2)),
                    'conf': float(conf),
                    'class_id': int(cls_id),
                    'class_name': POLE_CLASSES[int(cls_id)] if int(cls_id) < len(POLE_CLASSES) else 'unknown'
                })

            self.last_detections = det_list
            return det_list

        except Exception as e:
            print(f"Detection error: {e}")
            return []

    def draw_detections(self, frame, detections):
        """Draw detection boxes on frame"""
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            conf = det['conf']
            cls_id = det['class_id']
            cls_name = det['class_name']

            color = POLE_COLORS[cls_id % len(POLE_COLORS)]
            label = f"{cls_name} {conf:.2f}"

            # Draw box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            # Draw label background
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
            cv2.putText(frame, label, (x1 + 2, y1 - 4),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        return frame

    def lidar_callback(self, msg):
        """ROS callback for point cloud"""
        try:
            now = time.time()
            dt = now - self.last_lidar_time
            if dt > 0:
                self.lidar_hz = 1.0 / dt
            self.last_lidar_time = now

            # Debug: print field names on first message
            if self.latest_points is None:
                field_names = [f.name for f in msg.fields]
                print(f"Point cloud fields: {field_names}")
                print(f"Point cloud size: {msg.width}x{msg.height}")

            # OPTIMIZED: Read points directly as structured numpy array (10-50x faster)
            # pc2.read_points returns a generator - convert to list once then to numpy
            point_list = list(pc2.read_points(msg, skip_nans=True))
            if point_list:
                # Convert to numpy array efficiently
                raw = np.array(point_list, dtype=np.float32)
                if raw.shape[1] >= 3:
                    # Extract fields - handle variable field counts
                    n_fields = raw.shape[1]
                    x = raw[:, 0]
                    y = raw[:, 1]
                    z = raw[:, 2]
                    vel = raw[:, 3] if n_fields > 3 else np.zeros(len(raw), dtype=np.float32)
                    refl = raw[:, 4] if n_fields > 4 else np.zeros(len(raw), dtype=np.float32)
                    self.latest_points = np.column_stack([x, y, z, vel, refl])

        except Exception as e:
            print(f"LiDAR callback error: {e}")
            import traceback
            traceback.print_exc()

    def camera_callback(self, msg):
        """ROS callback for camera image (replay mode)"""
        try:
            if self.cv_bridge is not None:
                # Convert ROS Image to OpenCV format
                self.latest_camera_frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.camera_frame_time = time.time()
        except Exception as e:
            print(f"Camera callback error: {e}")

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse for LiDAR rotation (only in LiDAR panel)"""
        # Check if mouse is in the LiDAR panel (right side)
        self.mouse_in_lidar = x >= self.camera_width

        if event == cv2.EVENT_LBUTTONDOWN:
            if self.mouse_in_lidar:
                # Adjust x to be relative to LiDAR panel
                rel_x = x - self.camera_width
                # Check if clicking front button
                if hasattr(self, 'front_btn'):
                    btn_x, btn_y, btn_w, btn_h = self.front_btn
                    if btn_x <= rel_x <= btn_x + btn_w and btn_y <= y <= btn_y + btn_h:
                        self.rotation_x = 0.0
                        self.rotation_y = 180.0
                        self.scale = 150.0
                        print("Front view (straight-on)")
                        return
                # Check if clicking reset button
                if hasattr(self, 'reset_btn'):
                    btn_x, btn_y, btn_w, btn_h = self.reset_btn
                    if btn_x <= rel_x <= btn_x + btn_w and btn_y <= y <= btn_y + btn_h:
                        self.rotation_x = 15.0
                        self.rotation_y = 180.0
                        self.scale = 150.0
                        print("View reset (overhead)")
                        return
                self.mouse_down = True
                self.last_mouse_x = x
                self.last_mouse_y = y
        elif event == cv2.EVENT_LBUTTONUP:
            self.mouse_down = False
        elif event == cv2.EVENT_MOUSEMOVE and self.mouse_down and self.mouse_in_lidar:
            dx = x - self.last_mouse_x
            dy = y - self.last_mouse_y
            self.rotation_y += dx * 0.5
            self.rotation_x += dy * 0.5
            self.rotation_x = np.clip(self.rotation_x, -89, 89)
            self.rotation_y = self.rotation_y % 360
            self.last_mouse_x = x
            self.last_mouse_y = y
        elif event == cv2.EVENT_MOUSEWHEEL and self.mouse_in_lidar:
            delta = 10 if flags > 0 else -10
            self.scale = np.clip(self.scale + delta, 20.0, 300.0)

    def colorize(self, values):
        """Map values to colors: blue->cyan->green->yellow->red (like C++ viewer)"""
        v_min = np.percentile(values, 5)
        v_max = np.percentile(values, 95)

        if v_max > v_min:
            norm = np.clip((values - v_min) / (v_max - v_min), 0, 1)
        else:
            norm = np.full_like(values, 0.5)

        colors = np.zeros((len(norm), 3), dtype=np.uint8)

        # Vectorized color mapping for speed
        mask1 = norm < 0.25
        mask2 = (norm >= 0.25) & (norm < 0.5)
        mask3 = (norm >= 0.5) & (norm < 0.75)
        mask4 = norm >= 0.75

        # Blue to Cyan
        t = norm[mask1] * 4
        colors[mask1, 0] = 255
        colors[mask1, 1] = (255 * t).astype(np.uint8)
        colors[mask1, 2] = 0

        # Cyan to Green
        t = (norm[mask2] - 0.25) * 4
        colors[mask2, 0] = (255 * (1 - t)).astype(np.uint8)
        colors[mask2, 1] = 255
        colors[mask2, 2] = 0

        # Green to Yellow
        t = (norm[mask3] - 0.5) * 4
        colors[mask3, 0] = 0
        colors[mask3, 1] = 255
        colors[mask3, 2] = (255 * t).astype(np.uint8)

        # Yellow to Red
        t = (norm[mask4] - 0.75) * 4
        colors[mask4, 0] = 0
        colors[mask4, 1] = (255 * (1 - t)).astype(np.uint8)
        colors[mask4, 2] = 255

        return colors

    def render_lidar(self):
        """Render LiDAR with C++-style perspective projection"""
        view = np.zeros((self.panel_height, self.lidar_width, 3), dtype=np.uint8)
        width, height = self.lidar_width, self.panel_height

        if self.latest_points is None or len(self.latest_points) == 0:
            cv2.putText(view, "WAITING FOR LIDAR...",
                       (width//2 - 140, height//2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 165, 255), 2)
            return view

        points = self.latest_points
        point_count = len(points)

        try:
            # Extract coordinates - Aeva uses X=forward, Y=left, Z=up
            # Remap to viewing coords: depth=X, horizontal=Y, vertical=Z
            fwd = -points[:, 0].copy()   # X = forward (negate to flip view direction)
            left = points[:, 1].copy()   # Y = left/right
            up = points[:, 2].copy()     # Z = up/down
            intensity = points[:, 4] if points.shape[1] > 4 else up.copy()

            # Apply 3D rotation for user interaction
            angle_y = np.deg2rad(self.rotation_y)
            cos_y, sin_y = np.cos(angle_y), np.sin(angle_y)
            # Rotate around vertical (Z) axis
            x_rot = left * cos_y - fwd * sin_y
            depth = left * sin_y + fwd * cos_y

            angle_x = np.deg2rad(self.rotation_x)
            cos_x, sin_x = np.cos(angle_x), np.sin(angle_x)
            # Rotate around horizontal axis (tilt up/down)
            y_rot = up * cos_x - depth * sin_x
            z_final = up * sin_x + depth * cos_x

            # Filter points behind camera
            valid = z_final > -5.0
            x_rot = x_rot[valid]
            y_rot = y_rot[valid]
            z_final = z_final[valid]
            intensity = intensity[valid]

            if len(x_rot) == 0:
                cv2.putText(view, "No points in view",
                           (width//2 - 80, height//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
                return view

            # Perspective projection (like C++)
            cam_dist = 10.0
            persp = cam_dist / (z_final + cam_dist)
            px = (width/2 + x_rot * self.scale * persp).astype(np.int32)
            py = (height/2 - y_rot * self.scale * persp).astype(np.int32)

            # Filter to screen bounds
            valid = (px >= 0) & (px < width) & (py >= 0) & (py < height)
            px = px[valid]
            py = py[valid]
            z_final = z_final[valid]
            intensity = intensity[valid]

            if len(px) == 0:
                return view

            # Colorize (skip depth sorting for speed - minor visual impact)
            colors = self.colorize(intensity)

            # Draw points - use vectorized operations for speed
            if self.point_size == 1:
                # Direct pixel access (fastest)
                view[py, px] = colors
            else:
                # For larger points, use vectorized approach with numpy
                for i in range(len(px)):
                    cv2.circle(view, (int(px[i]), int(py[i])), self.point_size,
                              (int(colors[i, 0]), int(colors[i, 1]), int(colors[i, 2])), -1)

            # Run LiDAR pole detection
            self.run_lidar_detection()

            # Run sensor fusion
            self.run_fusion()

            # Run line following (always, for visualization)
            self.run_line_following()

            # Run navigation if enabled (sends commands to FCU)
            self.run_navigation()

            # Check Herelink C button for recording toggle
            if self.guidance and self.guidance.check_button_c_pressed():
                print("Herelink C button pressed!")
                self.toggle_recording()

            # Save recording frame if recording is active
            if self.recording:
                self.save_recording_frame(self.latest_frame, self.latest_points)

            # Draw detected poles (use fused if available, else raw LiDAR)
            poles_to_draw = self.fused_poles if (self.fusion_enabled and self.fused_poles) else self.lidar_poles
            for pole in poles_to_draw:
                # Transform pole position same as point cloud
                pole_fwd = -pole.x
                pole_left = pole.y
                pole_up = pole.z

                # Apply same rotation as points
                pole_x_rot = pole_left * cos_y - pole_fwd * sin_y
                pole_depth = pole_left * sin_y + pole_fwd * cos_y
                pole_y_rot = pole_up * cos_x - pole_depth * sin_x
                pole_z_final = pole_up * sin_x + pole_depth * cos_x

                # Skip if behind camera
                if pole_z_final <= -5.0:
                    continue

                # Project to screen
                pole_persp = cam_dist / (pole_z_final + cam_dist)
                pole_px = int(width/2 + pole_x_rot * self.scale * pole_persp)
                pole_py = int(height/2 - pole_y_rot * self.scale * pole_persp)

                # Skip if outside screen
                if not (0 <= pole_px < width and 0 <= pole_py < height):
                    continue

                # Draw pole marker - size based on radius and perspective
                marker_radius = max(int(pole.radius * self.scale * pole_persp * 2), 8)

                # Get confidence (fused or raw)
                if hasattr(pole, 'fused_confidence'):
                    conf = pole.fused_confidence
                    camera_matched = pole.camera_matched
                else:
                    conf = pole.confidence
                    camera_matched = False

                # Color: green if camera matched, cyan if LiDAR only, red if low confidence
                if camera_matched:
                    conf_color = (0, 255, 0)  # Green - camera matched
                else:
                    conf_color = (
                        int(255 * (1 - conf)),  # B
                        int(255 * conf),        # G
                        int(255 * conf)         # R (cyan)
                    )

                # Draw circle outline (thicker if camera matched)
                thickness = 3 if camera_matched else 2
                cv2.circle(view, (pole_px, pole_py), marker_radius, conf_color, thickness)
                # Draw center dot
                cv2.circle(view, (pole_px, pole_py), 3, (255, 255, 255), -1)

                # Draw pole ID label
                label = f"#{pole.id}"
                if camera_matched:
                    label += " CAM"
                cv2.putText(view, label, (pole_px + marker_radius + 3, pole_py - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

                # Draw info (height, range, confidence)
                range_m = np.sqrt(pole.x**2 + pole.y**2)
                info = f"{range_m:.1f}m c={conf:.0%}"
                cv2.putText(view, info, (pole_px + marker_radius + 3, pole_py + 12),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)

            # Status bar
            cv2.rectangle(view, (0, 0), (width, 70), (30, 30, 30), -1)

            # Title with detection/fusion status
            if self.sensor_fusion is not None and self.fusion_enabled:
                # Fusion mode
                fused_count = len(self.fused_poles)
                cam_matched = sum(1 for p in self.fused_poles if p.camera_matched)
                cv2.putText(view, "LIDAR + FUSION", (15, 28),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(view, f"Fused: {fused_count} ({cam_matched} cam) | {self.lidar_hz:.1f} Hz | Fusion: {self.fusion_hz:.1f} Hz",
                           (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            elif self.lidar_detector is not None:
                det_status = "ON" if self.lidar_detection_enabled else "OFF"
                det_color = (0, 255, 100) if self.lidar_detection_enabled else (100, 100, 100)
                cv2.putText(view, f"LIDAR + POLES [{det_status}]", (15, 28),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, det_color, 2)
                pole_count = len(self.lidar_poles)
                cv2.putText(view, f"Points: {point_count:,} | Poles: {pole_count} | {self.lidar_hz:.1f} Hz | Det: {self.lidar_detection_hz:.1f} Hz",
                           (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            else:
                cv2.putText(view, "LIDAR", (15, 28),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 100), 2)
                cv2.putText(view, f"Points: {point_count:,} | {self.lidar_hz:.1f} Hz",
                           (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(view, f"Rot: {self.rotation_x:.0f}/{self.rotation_y:.0f}",
                       (width - 200, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)

            # Front view button
            self.front_btn = (width - 140, 15, 55, 25)  # x, y, w, h
            btn_x, btn_y, btn_w, btn_h = self.front_btn
            cv2.rectangle(view, (btn_x, btn_y), (btn_x + btn_w, btn_y + btn_h), (80, 80, 80), -1)
            cv2.rectangle(view, (btn_x, btn_y), (btn_x + btn_w, btn_y + btn_h), (120, 120, 120), 1)
            cv2.putText(view, "Front", (btn_x + 5, btn_y + 18),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

            # Reset button (overhead view)
            self.reset_btn = (width - 70, 15, 55, 25)  # x, y, w, h
            btn_x, btn_y, btn_w, btn_h = self.reset_btn
            cv2.rectangle(view, (btn_x, btn_y), (btn_x + btn_w, btn_y + btn_h), (80, 80, 80), -1)
            cv2.rectangle(view, (btn_x, btn_y), (btn_x + btn_w, btn_y + btn_h), (120, 120, 120), 1)
            cv2.putText(view, "Reset", (btn_x + 5, btn_y + 18),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        except Exception as e:
            cv2.putText(view, f"Error: {str(e)[:40]}",
                       (10, height//2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        return view

    def render_camera(self):
        """Render camera feed with YOLO detection"""
        width, height = self.camera_width, self.panel_height

        if not self.camera_available:
            view = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(view, "NO CAMERA",
                       (width//2 - 80, height//2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            # Status bar
            cv2.rectangle(view, (0, 0), (width, 70), (30, 30, 30), -1)
            cv2.putText(view, "CAMERA", (15, 28),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 100, 100), 2)
            return view

        # Get camera frame from appropriate source
        if self.replay_mode:
            # Replay mode: use frame from ROS topic
            if self.latest_camera_frame is None:
                view = np.zeros((height, width, 3), dtype=np.uint8)
                cv2.putText(view, "WAITING FOR CAMERA...",
                           (width//2 - 140, height//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                cv2.putText(view, f"Topic: {self.camera_topic}",
                           (width//2 - 120, height//2 + 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
                return view
            frame = self.latest_camera_frame.copy()
        else:
            # Live mode: read from camera device
            ret, frame = self.cap.read()
            if not ret:
                view = np.zeros((height, width, 3), dtype=np.uint8)
                cv2.putText(view, "CAMERA READ ERROR",
                           (width//2 - 120, height//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                return view

            # Publish camera frame to ROS topic (for rosbag recording)
            if self.rosbag_recording and self.camera_pub is not None and self.cv_bridge is not None:
                try:
                    img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    img_msg.header.stamp = rospy.Time.now()
                    img_msg.header.frame_id = "camera"
                    self.camera_pub.publish(img_msg)
                except Exception as e:
                    pass  # Don't let publishing errors break the main loop

        # Run detection on original frame before resize (better accuracy)
        detections = self.run_detection(frame)

        # Scale detection boxes to panel size
        orig_h, orig_w = frame.shape[:2]
        scale_x = width / orig_w
        scale_y = height / orig_h

        # Resize to panel size
        frame = cv2.resize(frame, (width, height))

        # Scale and draw detections
        if detections:
            scaled_dets = []
            for det in detections:
                x1, y1, x2, y2 = det['bbox']
                scaled_dets.append({
                    'bbox': (int(x1 * scale_x), int(y1 * scale_y),
                             int(x2 * scale_x), int(y2 * scale_y)),
                    'conf': det['conf'],
                    'class_id': det['class_id'],
                    'class_name': det['class_name']
                })
            frame = self.draw_detections(frame, scaled_dets)

        # Draw projected LiDAR poles on camera (fusion visualization)
        if self.fusion_enabled and self.fused_poles:
            for pole in self.fused_poles:
                # Scale projection to panel size
                u = int(pole.image_u * scale_x)
                v = int(pole.image_v * scale_y)

                if 0 <= u < width and 0 <= v < height:
                    # Draw crosshair for LiDAR projection
                    cross_size = 15
                    if pole.camera_matched:
                        color = (0, 255, 0)  # Green if matched
                        # Draw line to matched bbox center if available
                    else:
                        color = (0, 255, 255)  # Yellow if no camera match

                    # Draw crosshair
                    cv2.line(frame, (u - cross_size, v), (u + cross_size, v), color, 2)
                    cv2.line(frame, (u, v - cross_size), (u, v + cross_size), color, 2)

                    # Draw pole ID
                    label = f"L#{pole.id}"
                    cv2.putText(frame, label, (u + 10, v - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Status bar overlay
        cv2.rectangle(frame, (0, 0), (width, 70), (30, 30, 30), -1)

        # Detection status indicator
        if self.detector is not None:
            det_status = "ON" if self.detection_enabled else "OFF"
            det_color = (0, 255, 100) if self.detection_enabled else (100, 100, 100)
            cv2.putText(frame, f"CAMERA + DETECT [{det_status}]", (15, 28),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, det_color, 2)
            det_count = len(self.last_detections)
            cv2.putText(frame, f"Poles: {det_count} | {self.detection_hz:.1f} Hz | Conf: {self.detection_conf:.0%}",
                       (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        else:
            cv2.putText(frame, "CAMERA", (15, 28),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 100), 2)
            cv2.putText(frame, f"{width}x{height}",
                       (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return frame

    def render(self):
        """Render combined view: Camera | LiDAR"""
        # Render both panels (same height, no resizing needed)
        camera_view = self.render_camera()
        lidar_view = self.render_lidar()

        # Combine side by side
        combined = np.hstack([camera_view, lidar_view])

        # FPS calculation
        now = time.time()
        dt = now - self.last_time
        if dt > 0:
            self.fps_history.append(1.0 / dt)
        self.last_time = now
        avg_fps = np.mean(self.fps_history) if self.fps_history else 0

        # Bottom status bar
        h, w = combined.shape[:2]
        cv2.rectangle(combined, (0, h - 30), (w, h), (20, 20, 20), -1)
        fusion_status = "ON" if self.fusion_enabled else "OFF"

        # Navigation status with smooth controller info
        if self.nav_enabled:
            if self.smooth_mode and self.smooth_controller and self.smooth_controller.is_running():
                nav_status = "NAV:SMOOTH"
                nav_color = (0, 255, 0)  # Green
            else:
                nav_status = "NAV:ON"
                nav_color = (0, 255, 0)  # Green
        elif self.fcu_connected:
            nav_status = "NAV:READY"
            nav_color = (0, 255, 255)  # Yellow
        else:
            nav_status = "NAV:OFF"
            nav_color = (150, 150, 150)  # Gray

        # Flight mode from guidance
        mode_info = ""
        mode_color = (150, 150, 150)  # Default gray
        if self.guidance:
            mode_name = self.guidance.get_mode_name()
            is_guided = self.guidance.is_guided_mode()
            mode_color = (0, 255, 0) if is_guided else (0, 165, 255)
            mode_info = f" [{mode_name}]"

        # Line state info (from smooth controller or simple line follower)
        line_info = ""
        if self.smooth_controller and self.smooth_controller.is_running():
            # Get smooth controller state (ControlState dataclass)
            sc_state = self.smooth_controller.get_state()
            if sc_state and sc_state.path_valid:
                line_info = f" | Spd:{sc_state.speed:.1f}m/s Lat:{sc_state.lateral_error:+.1f}m"
            elif sc_state:
                line_info = f" | Spd:{sc_state.speed:.1f}m/s [NO PATH]"
        elif self.line_state and self.line_state.valid:
            line_info = f" | Lat:{self.line_state.lateral_error:+.1f}m Head:{self.line_state.heading_error*57.3:+.0f}deg"

        # Obstacle status
        obs_info = ""
        obs_color = (150, 150, 150)  # Default gray
        if self.obstacle_state and self.obstacle_state.obstacle_detected:
            dist = self.obstacle_state.obstacle_distance
            if dist <= 3.0:  # Emergency stop distance
                obs_info = f" | OBS:{dist:.1f}m STOP!"
                obs_color = (0, 0, 255)  # Red
            elif dist <= 8.0:  # Slow distance
                obs_info = f" | OBS:{dist:.1f}m"
                obs_color = (0, 165, 255)  # Orange
            else:
                obs_info = f" | OBS:{dist:.1f}m"
                obs_color = (0, 255, 255)  # Yellow

        # Recording status
        rec_info = ""
        rec_color = (150, 150, 150)
        if self.recording:
            duration = time.time() - self.recording_start_time if self.recording_start_time else 0
            rec_info = f" | REC:{self.recording_frame_count}f {duration:.0f}s"
            rec_color = (0, 0, 255)  # Red for recording

        cv2.putText(combined, f"FPS:{avg_fps:.0f} | Fus:{fusion_status} | ",
                   (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
        cv2.putText(combined, nav_status,
                   (200, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, nav_color, 1)
        cv2.putText(combined, mode_info,
                   (285, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, mode_color if self.guidance else (150, 150, 150), 1)
        cv2.putText(combined, f"{line_info}{obs_info}{rec_info}",
                   (380, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, rec_color if self.recording else (obs_color if obs_info else (150, 150, 150)), 1)
        cv2.putText(combined, "'b'=rec 'n'=nav 'q'=quit",
                   (w - 220, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        return combined

    def run(self):
        """Main loop"""
        print("\n" + "="*60)
        print("  LIDAR + CAMERA FUSION VIEWER")
        print("  Now with YOLO pole detection!")
        print("="*60)

        print("Initializing ROS...")
        self.init_ros()
        print("ROS initialized.")

        print("Initializing camera...")
        self.init_camera()
        print("Camera initialized.")

        print("Initializing YOLO detector...")
        self.init_detector()
        if self.detector:
            print("YOLO detector ready.")
        else:
            print("YOLO detector not available.")

        print("Initializing LiDAR pole detector...")
        self.init_lidar_detector()
        if self.lidar_detector:
            print("LiDAR pole detector ready.")
        else:
            print("LiDAR pole detector not available.")

        print("Initializing sensor fusion...")
        self.init_sensor_fusion()
        if self.sensor_fusion:
            print("Sensor fusion ready.")
        else:
            print("Sensor fusion not available.")

        print("Initializing line follower...")
        self.init_line_follower()
        if self.line_follower:
            print("Line follower ready.")
        else:
            print("Line follower not available.")

        print("Initializing smooth controller...")
        self.init_smooth_controller()
        if self.smooth_controller:
            print("Smooth controller ready (100Hz Tesla-style)")
        else:
            print("Smooth controller not available - will use simple control")

        print("Initializing capture system...")
        self.init_capture_system()
        if self.capture_manager:
            print("Capture system ready (scheduler + bundler + obstacles)")
        else:
            print("Capture system not available")

        print("\nControls:")
        print("  Mouse drag (on LiDAR) - Rotate view")
        print("  Scroll (on LiDAR)     - Zoom in/out")
        print("  +/-                   - Zoom in/out")
        print("  r                     - Reset view (overhead)")
        print("  f                     - Front view (straight-on)")
        print("  p                     - Change point size (1-5)")
        print("  d                     - Toggle camera detection ON/OFF")
        print("  l                     - Toggle LiDAR pole detection ON/OFF")
        print("  s                     - Toggle sensor fusion ON/OFF")
        print("  t                     - Reset trackers")
        print("  n                     - Toggle NAVIGATION (connects to FCU)")
        print("  c                     - Connect to FCU (without enabling nav)")
        print("  b                     - Toggle training data RECORDING")
        print("  Herelink C button     - Toggle training data RECORDING")
        print("  Slider                - Adjust confidence threshold")
        print("  q/ESC                 - Quit")
        print()

        print("Creating window...")
        cv2.namedWindow('Fusion Viewer', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Fusion Viewer', self.camera_width + self.lidar_width, self.panel_height + 30)
        cv2.setMouseCallback('Fusion Viewer', self.mouse_callback)

        # Create confidence slider (0-100 representing 0.0-1.0)
        cv2.createTrackbar('Confidence %', 'Fusion Viewer',
                          int(self.detection_conf * 100), 100,
                          self.on_confidence_change)

        # Create exposure slider (0-100 maps to 100-5000)
        # Default 1000 = slider position ~18
        initial_exp_slider = int((self.camera_exposure - 100) / 49)
        cv2.createTrackbar('Exposure', 'Fusion Viewer',
                          initial_exp_slider, 100,
                          self.on_exposure_change)
        print("Window created, starting main loop...")

        while not rospy.is_shutdown():
            # Render combined view
            view = self.render()

            if self.frame_count == 0:
                print(f"First frame rendered: {view.shape}")

            cv2.imshow('Fusion Viewer', view)

            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
            elif key == ord('r'):
                self.rotation_x = 15.0
                self.rotation_y = 180.0
                self.scale = 150.0
                print("View reset (overhead)")
            elif key == ord('f'):
                self.rotation_x = 0.0
                self.rotation_y = 180.0
                self.scale = 150.0
                print("Front view (straight-on)")
            elif key in (ord('+'), ord('=')):
                self.scale = min(self.scale + 10.0, 300.0)
                print(f"Zoom: {self.scale:.0f}")
            elif key in (ord('-'), ord('_')):
                self.scale = max(self.scale - 10.0, 20.0)
                print(f"Zoom: {self.scale:.0f}")
            elif key == ord('p'):
                self.point_size = (self.point_size % 5) + 1
                print(f"Point size: {self.point_size}")
            elif key == ord('d'):
                self.detection_enabled = not self.detection_enabled
                status = "ON" if self.detection_enabled else "OFF"
                print(f"Camera detection: {status}")
            elif key == ord('l'):
                self.lidar_detection_enabled = not self.lidar_detection_enabled
                status = "ON" if self.lidar_detection_enabled else "OFF"
                print(f"LiDAR pole detection: {status}")
            elif key == ord('s'):
                self.fusion_enabled = not self.fusion_enabled
                status = "ON" if self.fusion_enabled else "OFF"
                print(f"Sensor fusion: {status}")
            elif key == ord('t'):
                if self.lidar_detector:
                    self.lidar_detector.reset_tracker()
                if self.sensor_fusion:
                    self.sensor_fusion.reset()
                if self.line_follower:
                    self.line_follower.reset()
                print("Trackers reset")
            elif key == ord('n'):
                # Toggle navigation (connect to FCU if needed)
                self.toggle_navigation()
            elif key == ord('c'):
                # Just connect to FCU without enabling nav
                if not self.fcu_connected:
                    print("Connecting to FCU...")
                    self.init_guidance()
                else:
                    print(f"FCU connected: {self.fcu_port}")
                    if self.guidance:
                        mode = self.guidance.get_mode_name()
                        print(f"Flight mode: {mode}")
            elif key == ord('b'):
                # Toggle training data recording
                self.toggle_recording()

            self.frame_count += 1

        # Cleanup
        print("\nShutting down...")

        # Stop smooth controller
        if self.smooth_controller and self.smooth_controller.is_running():
            print("  Stopping smooth controller...")
            self.smooth_controller.stop()

        # Stop capture system
        if self.capture_manager:
            print("  Stopping capture system...")
            self.capture_manager.stop()
            stats = self.capture_manager.get_stats()
            print(f"    Captures saved: {stats['bundler']['saved']}, dropped: {stats['bundler']['dropped']}")

        # Stop rosbag if recording
        if self.rosbag_recording:
            print("  Stopping rosbag recording...")
            self.stop_rosbag()

        # Stop guidance
        if self.guidance:
            print("  Stopping guidance...")
            self.guidance.stop()
            self.guidance.disconnect()

        # Release camera
        if self.cap:
            print("  Releasing camera...")
            self.cap.release()

        cv2.destroyAllWindows()
        print(f"Total frames: {self.frame_count}")
        print("Shutdown complete.")


def check_dependencies():
    """Pre-flight check for required dependencies."""
    print("\n" + "="*60)
    print("  DEPENDENCY CHECK")
    print("="*60)

    errors = []
    warnings = []

    # Critical dependencies
    try:
        import numpy as np
        print(f"  [OK] numpy {np.__version__}")
    except ImportError:
        errors.append("numpy not found - install with: pip install numpy")

    try:
        import cv2
        print(f"  [OK] opencv {cv2.__version__}")
    except ImportError:
        errors.append("opencv not found - install with: pip install opencv-python")

    # ROS (critical)
    if not ROS_AVAILABLE:
        errors.append("ROS not available - source /opt/ros/noetic/setup.bash")
    else:
        print("  [OK] ROS available")

    # Optional but recommended
    try:
        from scipy.interpolate import CubicSpline
        print("  [OK] scipy (spline fitting)")
    except ImportError:
        warnings.append("scipy not found - smooth controller will use linear interpolation")

    try:
        from sklearn.cluster import DBSCAN
        print("  [OK] sklearn (clustering)")
    except ImportError:
        errors.append("sklearn not found - install with: pip install scikit-learn")

    # YOLO (optional)
    if YOLO_AVAILABLE:
        print("  [OK] PyTorch (YOLO)")
    else:
        warnings.append("PyTorch not found - camera detection disabled")

    # MAVLink (optional for guidance)
    if GUIDANCE_AVAILABLE:
        print("  [OK] pymavlink (FCU control)")
    else:
        warnings.append("pymavlink not found - FCU control disabled")

    # Module checks
    if LIDAR_DETECTOR_AVAILABLE:
        print("  [OK] pole_detector module")
    else:
        warnings.append("pole_detector not available - LiDAR detection disabled")

    if FUSION_AVAILABLE:
        print("  [OK] fusion module")
    else:
        warnings.append("fusion module not available")

    if LINE_FOLLOWER_AVAILABLE:
        print("  [OK] line_follower module")
    else:
        warnings.append("line_follower not available")

    if SMOOTH_CONTROLLER_AVAILABLE:
        print("  [OK] smooth_controller module")
    else:
        warnings.append("smooth_controller not available - using simple control")

    print()

    # Print warnings
    for w in warnings:
        print(f"  [WARN] {w}")

    # Print errors and exit if critical
    if errors:
        print()
        for e in errors:
            print(f"  [ERROR] {e}")
        print()
        print("Critical dependencies missing. Please install them and try again.")
        return False

    if warnings:
        print()
    print("  All critical dependencies OK!")
    return True


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='LiDAR + Camera Fusion Viewer')
    parser.add_argument('--lidar-topic', default='/aeva_0/points',
                        help='ROS topic for LiDAR point cloud')
    parser.add_argument('--camera', type=int, default=0,
                        help='Camera device number (default: 0)')
    parser.add_argument('--replay', action='store_true',
                        help='Replay mode: use ROS camera topic instead of live camera')
    parser.add_argument('--camera-topic', default='/camera/image_raw',
                        help='ROS topic for camera in replay mode')
    args = parser.parse_args()

    # Pre-flight dependency check
    if not check_dependencies():
        sys.exit(1)

    print(f"LiDAR topic: {args.lidar_topic}")
    if args.replay:
        print(f"REPLAY MODE: Camera from {args.camera_topic}")
    else:
        print(f"Camera device: {args.camera}")

    viewer = FusionViewer(
        lidar_topic=args.lidar_topic,
        camera_device=args.camera,
        replay_mode=args.replay,
        camera_topic=args.camera_topic
    )
    viewer.run()
