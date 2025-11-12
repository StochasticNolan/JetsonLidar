#!/usr/bin/env python3
"""
Pole Navigation Fusion
Minimal LiDAR-camera fusion for Jetson deployment
"""

import cv2
import numpy as np
from ultralytics import YOLO
from pathlib import Path
from typing import List, Dict, Tuple
import yaml
import sys


class PoleNavigation:
    def __init__(self, config_path='config/settings.yaml'):
        """Initialize from config file"""
        # Load config
        with open(config_path) as f:
            self.config = yaml.safe_load(f)

        print(f"✓ Loaded config from {config_path}")

        # Camera setup
        cam_dev = self.config['camera']['device']
        self.cap = cv2.VideoCapture(cam_dev)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config['camera']['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config['camera']['height'])
        print(f"✓ Camera opened: device {cam_dev}")

        # YOLO model
        model_path = self.config['yolo']['model']
        if not Path(model_path).exists():
            print(f"ERROR: Model not found: {model_path}")
            sys.exit(1)

        self.yolo = YOLO(model_path)
        print(f"✓ YOLO model loaded: {model_path}")

        # LiDAR setup (optional)
        self.lidar_enabled = self.config['lidar']['enabled']
        if self.lidar_enabled:
            import socket
            self.lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.lidar_sock.bind(('0.0.0.0', self.config['lidar']['port']))
            self.lidar_sock.settimeout(0.1)
            print(f"✓ LiDAR enabled: {self.config['lidar']['ip']}:{self.config['lidar']['port']}")
        else:
            print("○ LiDAR disabled (camera-only mode)")

        # State
        self.frame_count = 0

    def read_lidar(self) -> np.ndarray:
        """Read LiDAR point cloud"""
        if not self.lidar_enabled:
            return np.array([])

        try:
            data, _ = self.lidar_sock.recvfrom(2048)
            num_fields = 5
            point_size = num_fields * 4

            if len(data) % point_size == 0:
                num_points = len(data) // point_size
                points = np.frombuffer(data, dtype=np.float32).reshape(num_points, num_fields)
                return points
        except:
            pass

        return np.array([])

    def detect_poles_camera(self, frame: np.ndarray) -> List[Dict]:
        """YOLO pole detection"""
        results = self.yolo(frame, conf=self.config['yolo']['confidence'], verbose=False)

        detections = []
        for r in results:
            if r.boxes is not None:
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])

                    detections.append({
                        'bbox': (int(x1), int(y1), int(x2), int(y2)),
                        'confidence': conf,
                        'center': ((x1 + x2) / 2, (y1 + y2) / 2),
                        'source': 'camera'
                    })

        return detections

    def detect_poles_lidar(self, points: np.ndarray) -> List[Dict]:
        """LiDAR pole detection via clustering"""
        if len(points) == 0:
            return []

        xyz = points[:, :3]

        # Ground removal
        above_ground = xyz[xyz[:, 1] > 0.2]
        if len(above_ground) < 10:
            return []

        # Clustering
        from sklearn.cluster import DBSCAN
        xz = above_ground[:, [0, 2]]
        clustering = DBSCAN(eps=0.5, min_samples=10).fit(xz)

        detections = []
        for label in set(clustering.labels_) - {-1}:
            cluster = above_ground[clustering.labels_ == label]
            center = cluster.mean(axis=0)
            distances = np.linalg.norm(cluster[:, [0, 2]] - center[[0, 2]], axis=1)
            radius = distances.mean()

            if 0.05 < radius < 0.3:
                detections.append({
                    'position_3d': center,
                    'radius': radius,
                    'source': 'lidar'
                })

        return detections

    def fuse_detections(self, camera_dets: List, lidar_dets: List, frame_shape: Tuple) -> List[Dict]:
        """Fuse camera and LiDAR detections"""
        if self.config['fusion']['prefer_lidar'] and len(lidar_dets) > 0:
            return [{'type': 'pole', 'position': d['position_3d'],
                    'confidence': 0.9, 'source': 'lidar'} for d in lidar_dets]

        # Camera-only fallback
        fused = []
        for det in camera_dets:
            bbox_height = det['bbox'][3] - det['bbox'][1]
            dist = 1000.0 / max(bbox_height, 1)
            cx = det['center'][0]
            x_norm = (cx - frame_shape[1]/2) / (frame_shape[1]/2)
            x = x_norm * dist * 0.5

            fused.append({
                'type': 'pole',
                'position': np.array([x, 0, dist]),
                'confidence': det['confidence'],
                'source': 'camera'
            })

        return fused

    def compute_guidance(self, poles: List[Dict]) -> Dict:
        """Compute guidance from poles"""
        if len(poles) < self.config['fusion']['min_poles']:
            return {'command': 'STOP', 'lateral_velocity': 0.0, 'angular_velocity': 0.0}

        # Separate left/right
        left = [p for p in poles if p['position'][0] < 0]
        right = [p for p in poles if p['position'][0] > 0]

        if len(left) < 1 or len(right) < 1:
            return {'command': 'STOP', 'lateral_velocity': 0.0, 'angular_velocity': 0.0}

        # Centerline
        left_x = np.mean([p['position'][0] for p in left])
        right_x = np.mean([p['position'][0] for p in right])
        centerline_x = (left_x + right_x) / 2

        # Guidance
        lateral_vel = -centerline_x * self.config['guidance']['lateral_gain']
        angular_vel = -centerline_x * self.config['guidance']['heading_gain']

        lateral_vel = np.clip(lateral_vel, -self.config['guidance']['max_lateral_velocity'],
                                           self.config['guidance']['max_lateral_velocity'])
        angular_vel = np.clip(angular_vel, -self.config['guidance']['max_angular_velocity'],
                                           self.config['guidance']['max_angular_velocity'])

        if abs(centerline_x) < 0.3:
            command = 'FORWARD'
        elif centerline_x < 0:
            command = 'MOVE_RIGHT'
        else:
            command = 'MOVE_LEFT'

        return {
            'command': command,
            'lateral_velocity': lateral_vel,
            'angular_velocity': angular_vel,
            'confidence': min(1.0, len(poles) / 4.0)
        }

    def visualize(self, frame: np.ndarray, camera_dets: List,
                  lidar_dets: List, poles: List, guidance: Dict) -> np.ndarray:
        """Draw visualization"""
        vis = frame.copy()

        # Camera detections
        for det in camera_dets:
            x1, y1, x2, y2 = det['bbox']
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Fused poles
        for pole in poles:
            x, y, z = pole['position']
            if z > 0:
                px = int(960 + (x / z) * 1000)
                py = int(540 - (y / z) * 1000)
                if 0 <= px < frame.shape[1] and 0 <= py < frame.shape[0]:
                    color = (255, 0, 0) if pole['source'] == 'lidar' else (0, 165, 255)
                    cv2.circle(vis, (px, py), 10, color, -1)

        # Guidance arrow
        if guidance['command'] != 'STOP':
            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 3

            if guidance['command'] == 'MOVE_LEFT':
                cv2.arrowedLine(vis, (cx, cy), (cx - 150, cy), (0, 165, 255), 15, tipLength=0.3)
            elif guidance['command'] == 'MOVE_RIGHT':
                cv2.arrowedLine(vis, (cx, cy), (cx + 150, cy), (0, 165, 255), 15, tipLength=0.3)
            else:
                cv2.arrowedLine(vis, (cx, cy+50), (cx, cy - 150), (0, 255, 0), 15, tipLength=0.3)

            cv2.putText(vis, guidance['command'], (cx-80, cy+100),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)

        # Status
        status = [
            f"Frame: {self.frame_count}",
            f"Camera: {len(camera_dets)} | LiDAR: {len(lidar_dets)}",
            f"Poles: {len(poles)}",
            f"Cmd: {guidance['command']}",
            f"Conf: {guidance.get('confidence', 0):.2f}"
        ]

        for i, text in enumerate(status):
            cv2.putText(vis, text, (10, 30 + i*30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return vis

    def run(self):
        """Main loop"""
        print("\n=== Pole Navigation Started ===")
        print("Controls: 'q'=quit, 'p'=pause, 's'=screenshot")

        writer = None
        if self.config['output']['save_video']:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            output_path = self.config['output']['video_path']
            Path(output_path).parent.mkdir(parents=True, exist_ok=True)

        paused = False

        while True:
            if not paused:
                ret, frame = self.cap.read()
                if not ret:
                    break

                # Detect
                lidar_points = self.read_lidar()
                camera_dets = self.detect_poles_camera(frame)
                lidar_dets = self.detect_poles_lidar(lidar_points)

                # Fuse
                poles = self.fuse_detections(camera_dets, lidar_dets, frame.shape)

                # Guidance
                guidance = self.compute_guidance(poles)

                # Visualize
                vis = self.visualize(frame, camera_dets, lidar_dets, poles, guidance)

                # Save
                if self.config['output']['save_video']:
                    if writer is None:
                        writer = cv2.VideoWriter(output_path, fourcc, 30,
                                                (vis.shape[1], vis.shape[0]))
                    writer.write(vis)

                self.frame_count += 1

                if self.config['performance']['visualization']:
                    cv2.imshow('Pole Navigation', vis)

            # Keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('p'):
                paused = not paused
            elif key == ord('s') and self.config['output']['save_screenshots']:
                screenshot_dir = Path(self.config['output']['screenshot_dir'])
                screenshot_dir.mkdir(parents=True, exist_ok=True)
                screenshot_path = screenshot_dir / f"screenshot_{self.frame_count:04d}.png"
                cv2.imwrite(str(screenshot_path), vis)
                print(f"Screenshot: {screenshot_path}")

        # Cleanup
        self.cap.release()
        if writer:
            writer.release()
        cv2.destroyAllWindows()

        print(f"\nProcessed {self.frame_count} frames")
        if self.config['output']['save_video']:
            print(f"Video saved: {output_path}")


if __name__ == "__main__":
    nav = PoleNavigation('config/settings.yaml')
    nav.run()
