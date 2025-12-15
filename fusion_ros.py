#!/usr/bin/env python3
"""
LiDAR + Camera Fusion Viewer
Clean side-by-side visualization with C++-style LiDAR rendering
"""

import cv2
import numpy as np
import time
import sys
from collections import deque

# ROS imports
try:
    import rospy
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs.point_cloud2 as pc2
    ROS_AVAILABLE = True
except ImportError:
    print("ERROR: ROS not available")
    ROS_AVAILABLE = False
    sys.exit(1)


class FusionViewer:
    """Clean LiDAR + Camera viewer with C++-style rendering"""

    def __init__(self, lidar_topic="/aeva_0/points", camera_device=0):
        # LiDAR state
        self.lidar_topic = lidar_topic
        self.latest_points = None
        self.lidar_hz = 0.0
        self.last_lidar_time = time.time()

        # Camera state
        self.camera_device = camera_device
        self.cap = None
        self.camera_available = False

        # View parameters - slight overhead perspective
        self.scale = 60.0           # pixels per meter (zoomed in)
        self.point_size = 1         # single pixel for crisp dense clouds
        self.rotation_x = -15.0     # degrees (camera overhead looking down)
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

    def init_ros(self):
        """Initialize ROS subscriber"""
        rospy.init_node('fusion_viewer', anonymous=True)
        self.subscriber = rospy.Subscriber(
            self.lidar_topic,
            PointCloud2,
            self.lidar_callback,
            queue_size=1,
            buff_size=2**24
        )
        print(f"Subscribed to: {self.lidar_topic}")

    def init_camera(self):
        """Initialize USB camera"""
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

            # Read all points without filtering by field names
            points = []
            for point in pc2.read_points(msg, skip_nans=True):
                # Point is a tuple of all fields
                if len(point) >= 3:
                    x, y, z = point[0], point[1], point[2]
                    vel = point[3] if len(point) > 3 else 0.0
                    refl = point[4] if len(point) > 4 else 0.0
                    points.append([x, y, z, vel, refl])

            if points:
                self.latest_points = np.array(points, dtype=np.float32)

        except Exception as e:
            print(f"LiDAR callback error: {e}")
            import traceback
            traceback.print_exc()

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse for LiDAR rotation (only in LiDAR panel)"""
        # Check if mouse is in the LiDAR panel (right side)
        self.mouse_in_lidar = x >= self.camera_width

        if event == cv2.EVENT_LBUTTONDOWN:
            if self.mouse_in_lidar:
                # Check if clicking reset button
                if hasattr(self, 'reset_btn'):
                    btn_x, btn_y, btn_w, btn_h = self.reset_btn
                    # Adjust x to be relative to LiDAR panel
                    rel_x = x - self.camera_width
                    if btn_x <= rel_x <= btn_x + btn_w and btn_y <= y <= btn_y + btn_h:
                        self.rotation_x = -15.0
                        self.rotation_y = 180.0
                        self.scale = 60.0
                        print("View reset")
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
            delta = 5 if flags > 0 else -5
            self.scale = np.clip(self.scale + delta, 5.0, 100.0)

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

            # Status bar
            cv2.rectangle(view, (0, 0), (width, 70), (30, 30, 30), -1)
            cv2.putText(view, "LIDAR", (15, 28),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 100), 2)
            cv2.putText(view, f"Points: {point_count:,} | {self.lidar_hz:.1f} Hz",
                       (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(view, f"Rot: {self.rotation_x:.0f}/{self.rotation_y:.0f}",
                       (width - 200, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)

            # Reset button
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
        """Render camera feed"""
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

        ret, frame = self.cap.read()
        if not ret:
            view = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(view, "CAMERA READ ERROR",
                       (width//2 - 120, height//2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return view

        # Resize to panel size
        frame = cv2.resize(frame, (width, height))

        # Status bar overlay
        cv2.rectangle(frame, (0, 0), (width, 70), (30, 30, 30), -1)
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
        cv2.putText(combined, f"Display: {avg_fps:.1f} FPS | Frame: {self.frame_count} | "
                   f"Drag LiDAR to rotate, scroll to zoom | 'r'=reset 'p'=point size 'q'=quit",
                   (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        return combined

    def run(self):
        """Main loop"""
        print("\n" + "="*60)
        print("  LIDAR + CAMERA FUSION VIEWER")
        print("="*60)

        print("Initializing ROS...")
        self.init_ros()
        print("ROS initialized.")

        print("Initializing camera...")
        self.init_camera()
        print("Camera initialized.")

        print("\nControls:")
        print("  Mouse drag (on LiDAR) - Rotate view")
        print("  Scroll (on LiDAR)     - Zoom in/out")
        print("  +/-                   - Zoom in/out")
        print("  r                     - Reset view")
        print("  p                     - Change point size (1-5)")
        print("  q/ESC                 - Quit")
        print()

        print("Creating window...")
        cv2.namedWindow('Fusion Viewer', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Fusion Viewer', self.camera_width + self.lidar_width, self.panel_height + 30)
        cv2.setMouseCallback('Fusion Viewer', self.mouse_callback)
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
                self.rotation_x = -15.0
                self.rotation_y = 180.0
                self.scale = 60.0
                print("View reset")
            elif key in (ord('+'), ord('=')):
                self.scale = min(self.scale + 5.0, 100.0)
                print(f"Zoom: {self.scale:.0f}")
            elif key in (ord('-'), ord('_')):
                self.scale = max(self.scale - 5.0, 5.0)
                print(f"Zoom: {self.scale:.0f}")
            elif key == ord('p'):
                self.point_size = (self.point_size % 5) + 1
                print(f"Point size: {self.point_size}")

            self.frame_count += 1

        # Cleanup
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        print(f"\nTotal frames: {self.frame_count}")


if __name__ == "__main__":
    # Parse optional arguments
    lidar_topic = "/aeva_0/points"
    camera_device = 0

    if len(sys.argv) > 1:
        lidar_topic = sys.argv[1]
    if len(sys.argv) > 2:
        camera_device = int(sys.argv[2])

    print(f"LiDAR topic: {lidar_topic}")
    print(f"Camera device: {camera_device}")

    viewer = FusionViewer(lidar_topic=lidar_topic, camera_device=camera_device)
    viewer.run()
