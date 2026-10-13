#!/usr/bin/env python3
import sys
import os

# Automatically re-execute with virtualenv python if needed (provides YOLO / Torch runtime)
VENV_PYTHON = '/home/hospobot/hospobot_ws/venv/bin/python3'
if sys.executable != VENV_PYTHON and os.path.exists(VENV_PYTHON):
    os.execv(VENV_PYTHON, [VENV_PYTHON] + sys.argv)

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
import math

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from hospobot_interfaces.msg import SemanticObject, SemanticObjectArray
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Point, Vector3, Vector3Stamped

# Try importing YOLO
YOLO_AVAILABLE = False
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except Exception:
    pass


class TrackedObject:
    """Tracks a 3D semantic object across frames in the world frame with robust stationary anchor rejection."""
    def __init__(self, track_id, class_name, x, y, z, current_time=None, frame_id='map'):
        self.id = track_id
        self.class_name = class_name
        self.x = x
        self.y = y
        self.z = z
        self.anchor_x = x
        self.anchor_y = y
        self.is_moving = False
        self.frame_id = frame_id
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.last_seen = current_time if current_time is not None else time.time()
        self.hits = 1
        self.history = [(self.last_seen, x, y)]

    def update(self, x, y, z, current_time):
        dt = current_time - self.last_seen if self.last_seen > 0 else 0.05
        self.last_seen = current_time
        self.history.append((current_time, x, y))
        self.history = [h for h in self.history if (current_time - h[0]) <= 0.30]
        
        dist_from_anchor = math.hypot(x - self.anchor_x, y - self.anchor_y)
        
        # Robust stationary rejection:
        # A stationary human (even during robot weaving or depth noise) stays within 25cm of their anchor.
        # This guarantees velocity remains strictly 0.0 m/s until the human actually takes physical steps!
        if not self.is_moving:
            if dist_from_anchor > 0.25:
                self.is_moving = True
            else:
                self.anchor_x = 0.95 * self.anchor_x + 0.05 * x
                self.anchor_y = 0.95 * self.anchor_y + 0.05 * y
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                self.x = x
                self.y = y
                self.z = z
                self.hits += 1
                return

        # In motion: estimate ground velocity over a 0.15s - 0.45s baseline to eliminate frame differentiation noise
        if len(self.history) >= 2:
            t_old, x_old, y_old = self.history[0]
            dt_hist = current_time - t_old
            if dt_hist > 0.12:
                inst_vx = (x - x_old) / dt_hist
                inst_vy = (y - y_old) / dt_hist
                inst_speed = math.hypot(inst_vx, inst_vy)
                
                # If speed dropped below 0.18 m/s, person has stopped moving!
                if inst_speed < 0.18:
                    self.is_moving = False
                    self.anchor_x = x
                    self.anchor_y = y
                    self.vx = 0.0
                    self.vy = 0.0
                    self.vz = 0.0
                else:
                    if inst_speed > 2.5:
                        scale = 2.5 / inst_speed
                        inst_vx *= scale
                        inst_vy *= scale
                    if self.hits <= 2:
                        self.vx = inst_vx
                        self.vy = inst_vy
                    else:
                        alpha = 0.40
                        self.vx = alpha * inst_vx + (1.0 - alpha) * self.vx
                        self.vy = alpha * inst_vy + (1.0 - alpha) * self.vy

        self.x = x
        self.y = y
        self.z = z
        self.hits += 1


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        self.get_logger().info('Initializing Hospital Object & Human Detector Node...')
        
        self.bridge = CvBridge()
        
        # TF2 Setup for transforming camera coordinates to robot base and odom
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize YOLOv8 if available, with automatic fallback to OpenCV HOG
        self.yolo_model = None
        self.detector_type = "NONE"
        
        model_path = '/home/hospobot/hospobot_ws/yolov8n.pt'
        if YOLO_AVAILABLE and os.path.exists(model_path):
            try:
                self.get_logger().info(f"Loading YOLOv8 model from {model_path}...")
                self.yolo_model = YOLO(model_path)
                self.detector_type = "YOLOv8"
                self.get_logger().info("YOLOv8 detector loaded successfully.")
            except Exception as e:
                self.get_logger().warning(f"Failed to load YOLOv8: {e}. Falling back to OpenCV HOG.")
                self.yolo_model = None
                
        if self.yolo_model is None:
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
            self.face_cascade = cv2.CascadeClassifier(cascade_path) if os.path.exists(cascade_path) else None
            self.detector_type = "HOG+Haar"
            self.get_logger().info("Using OpenCV HOG Pedestrian Detector as primary.")

        # Camera Intrinsics (updated from CameraInfo)
        self.fx = 283.4
        self.fy = 283.3
        self.cx = 321.6
        self.cy = 196.1
        self.intrinsics_calibrated = False

        # Camera Info Subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/oak/stereo/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Approximate Time Synchronizer for RGB and Stereo Depth
        # OAK-D publishes with best_effort (sensor_data QoS)
        self.color_sub = message_filters.Subscriber(
            self,
            Image,
            '/oak/rgb/image_raw',
            qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            '/oak/stereo/image_raw',
            qos_profile=qos_profile_sensor_data
        )
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.15
        )
        self.ts.registerCallback(self.sync_callback)

        # Fallback subscription in case depth is desynchronized
        self.last_depth_image = None
        self.last_depth_frame_id = "oak_rgb_camera_optical_frame"
        self.depth_standalone_sub = self.create_subscription(
            Image,
            '/oak/stereo/image_raw',
            self.depth_standalone_callback,
            qos_profile=qos_profile_sensor_data
        )

        # Publishers
        # 1. Annotated Image view for RViz (user requested)
        self.annotated_image_pub = self.create_publisher(Image, '/oak/annotated_image', 10)
        self.debug_image_pub = self.create_publisher(Image, '/human_detector/debug_image', 10)
        
        # 2. Semantic Objects (Tracked & Raw)
        self.tracked_objects_pub = self.create_publisher(SemanticObjectArray, '/semantic/tracked_objects', 10)
        self.raw_objects_pub = self.create_publisher(SemanticObjectArray, '/semantic/raw_objects', 10)
        self.legacy_objects_pub = self.create_publisher(SemanticObjectArray, '/oakd/semantic_objects', 10)
        
        # 3. Vision Detections (2D)
        self.det_pub = self.create_publisher(Detection2DArray, '/oakd/detections', 10)
        
        # 4. 3D Markers for RViz
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic/tracked_objects_markers', 10)

        # Object Tracking State
        self.tracked_objects = []
        self.next_track_id = 1
        self.distance_match_threshold = 1.5 # meters in odom frame
        self.track_expiry_sec = 0.50 # seconds (clears objects 0.5s after leaving camera frame)
        self.fps_counter = 0
        self.fps_timer = time.time()
        self.current_fps = 0.0

        self.get_logger().info(f"Detector ready [{self.detector_type}]. Publishing annotated camera feed to /oak/annotated_image")

    def camera_info_callback(self, msg: CameraInfo):
        if not self.intrinsics_calibrated:
            if msg.k[0] > 0 and msg.k[4] > 0:
                self.fx = float(msg.k[0])
                self.cx = float(msg.k[2])
                self.fy = float(msg.k[4])
                self.cy = float(msg.k[5])
                self.intrinsics_calibrated = True
                self.get_logger().info(f"Updated camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}")

    def depth_standalone_callback(self, msg: Image):
        try:
            self.last_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.last_depth_frame_id = msg.header.frame_id or "oak_rgb_camera_optical_frame"
        except Exception:
            pass

    def sync_callback(self, color_msg: Image, depth_msg: Image):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            self.last_depth_image = depth_image
            self.last_depth_frame_id = depth_msg.header.frame_id or "oak_rgb_camera_optical_frame"
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion error: {e}")
            return
            
        self.process_frames(color_image, depth_image, color_msg.header)

    def extract_depth_for_box(self, depth_image, x1, y1, x2, y2):
        """Extracts robust median depth (in meters) within the detected bounding box."""
        if depth_image is None:
            return None
            
        ih, iw = depth_image.shape[:2]
        # Crop to box boundaries
        bx1 = max(0, min(x1, iw - 1))
        bx2 = max(0, min(x2, iw - 1))
        by1 = max(0, min(y1, ih - 1))
        by2 = max(0, min(y2, ih - 1))
        
        if bx2 <= bx1 or by2 <= by1:
            return None
            
        # Sample the center 40% region of the bounding box to avoid background bleed
        cx = (bx1 + bx2) // 2
        cy = (by1 + by2) // 2
        rw = max(4, int((bx2 - bx1) * 0.20))
        rh = max(4, int((by2 - by1) * 0.20))
        
        patch_x1 = max(0, cx - rw)
        patch_x2 = min(iw, cx + rw)
        patch_y1 = max(0, cy - rh)
        patch_y2 = min(ih, cy + rh)
        
        patch = depth_image[patch_y1:patch_y2, patch_x1:patch_x2]
        
        # Valid depth mask: non-zero, within 0.25m to 8.0m (OAK-D typical range)
        valid_depths = patch[(patch > 250) & (patch < 8000)]
        
        if len(valid_depths) > 5:
            median_mm = np.median(valid_depths)
            return float(median_mm) / 1000.0 # Return in meters
            
        # Fallback to broader box patch if center was occluded
        broad_patch = depth_image[by1:by2, bx1:bx2]
        valid_broad = broad_patch[(broad_patch > 250) & (broad_patch < 8000)]
        if len(valid_broad) > 10:
            median_mm = np.median(valid_broad)
            return float(median_mm) / 1000.0
            
        return None

    def process_frames(self, color_image, depth_image, header):
        now = time.time()
        
        # FPS Calculation
        self.fps_counter += 1
        if now - self.fps_timer >= 1.0:
            self.current_fps = self.fps_counter / (now - self.fps_timer)
            self.fps_counter = 0
            self.fps_timer = now

        ih, iw = color_image.shape[:2]
        raw_detections = [] # list of dicts: {'box': [x1,y1,x2,y2], 'class': str, 'conf': float}

        # 1. Run Detection
        if self.yolo_model is not None:
            try:
                # Downsample slightly for high inference throughput
                results = self.yolo_model(color_image, imgsz=416, verbose=False, conf=0.35)
                for r in results:
                    for box in r.boxes:
                        cls_id = int(box.cls[0])
                        class_name = self.yolo_model.names[cls_id].upper()
                        conf = float(box.conf[0])
                        
                        # Filter target classes
                        if class_name == 'PERSON':
                            class_name = 'HUMAN'
                        elif class_name in ['BED', 'CHAIR', 'COUCH', 'SOFA']:
                            pass
                        else:
                            continue # Ignore background objects
                            
                        xyxy = box.xyxy[0].cpu().numpy()
                        raw_detections.append({
                            'box': [int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])],
                            'class': class_name,
                            'conf': conf
                        })
            except Exception as e:
                self.get_logger().warn(f"YOLO inference error: {e}")
        else:
            # Classical OpenCV HOG fallback
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            boxes, weights = self.hog.detectMultiScale(gray, winStride=(8, 8), padding=(8, 8), scale=1.05)
            for box, weight in zip(boxes, weights):
                if weight > 0.6:
                    x, y, w, h = box
                    raw_detections.append({
                        'box': [x, y, x + w, y + h],
                        'class': 'HUMAN',
                        'conf': float(min(1.0, weight))
                    })
            if len(raw_detections) == 0 and self.face_cascade is not None:
                faces = self.face_cascade.detectMultiScale(gray, 1.15, 5)
                for (x, y, w, h) in faces:
                    # Estimate full body from face
                    raw_detections.append({
                        'box': [max(0, x - w), max(0, y), min(iw, x + 2 * w), min(ih, y + 4 * h)],
                        'class': 'HUMAN',
                        'conf': 0.65
                    })

        # 2. 3D Position Estimation & TF Transformation
        camera_frame = header.frame_id or "oak_rgb_camera_optical_frame"
        
        # Prepare 3D transformed detections
        projected_objects = []
        
        # Determine fixed world frame for velocity tracking (prefer 'map', fallback to 'odom')
        world_frame = None
        tf_cam_to_world = None
        for candidate in ['map', 'odom']:
            try:
                tf_cam_to_world = self.tf_buffer.lookup_transform(candidate, camera_frame, rclpy.time.Time())
                world_frame = candidate
                break
            except Exception:
                continue
            
        tf_cam_to_base = None
        try:
            tf_cam_to_base = self.tf_buffer.lookup_transform('base_footprint', camera_frame, rclpy.time.Time())
        except Exception:
            pass

        for det in raw_detections:
            x1, y1, x2, y2 = det['box']
            u_center = (x1 + x2) / 2.0
            v_center = (y1 + y2) / 2.0
            box_h = max(1, y2 - y1)
            cls_name = det['class']
            
            # 1. Try stereo depth extraction
            z_stereo = self.extract_depth_for_box(depth_image, x1, y1, x2, y2)
            
            # 2. Geometric pinhole depth fallback
            # (Ensures humans are NEVER lost at 2m - 7m where stereo can have holes/low disparity)
            z_geom = None
            if cls_name == 'HUMAN' and box_h > 15:
                # Standard indoor adult height is ~1.70m
                z_geom = (self.fy * 1.70) / float(box_h)
            elif cls_name in ['BED', 'COUCH', 'SOFA'] and box_h > 15:
                z_geom = (self.fy * 0.85) / float(box_h)
            elif cls_name == 'CHAIR' and box_h > 15:
                z_geom = (self.fy * 0.90) / float(box_h)
                
            # Fuse depth: if stereo is valid and close (<3.0m), prefer stereo;
            # if stereo is absent, noisy, or human is >3m away, seamlessly use geometric depth
            if z_stereo is not None and 0.35 <= z_stereo <= 3.5:
                if z_geom is not None and abs(z_stereo - z_geom) < 2.0:
                    z_depth = 0.7 * z_stereo + 0.3 * z_geom
                else:
                    z_depth = z_stereo
            elif z_geom is not None and 0.4 <= z_geom <= 8.5:
                z_depth = z_geom
            elif z_stereo is not None and 0.25 <= z_stereo <= 8.5:
                z_depth = z_stereo
            else:
                z_depth = None
            
            det_info = {
                'box': [x1, y1, x2, y2],
                'class': det['class'],
                'conf': det['conf'],
                'has_3d': False,
                'rel_pos': None,   # [x, y, z] in base_footprint (robot-relative)
                'world_pos': None, # [x, y, z] in world_frame ('map' or 'odom')
                'distance': 0.0,
                'track': None
            }
            
            if z_depth is not None and z_depth > 0.2:
                # Pinhole projection in camera optical frame
                # Camera Optical Frame: X right, Y down, Z forward
                opt_x = (u_center - self.cx) * z_depth / self.fx
                opt_y = (v_center - self.cy) * z_depth / self.fy
                opt_z = z_depth
                
                pt_optical = PointStamped()
                pt_optical.header.frame_id = camera_frame
                pt_optical.header.stamp = header.stamp
                pt_optical.point.x = opt_x
                pt_optical.point.y = opt_y
                pt_optical.point.z = opt_z
                
                det_info['has_3d'] = True
                det_info['distance'] = float(z_depth)
                
                # Transform to base_footprint (robot-relative)
                if tf_cam_to_base is not None:
                    try:
                        pt_base = tf2_geometry_msgs.do_transform_point(pt_optical, tf_cam_to_base)
                        det_info['rel_pos'] = [pt_base.point.x, pt_base.point.y, pt_base.point.z]
                    except Exception:
                        pass
                
                # Analytic kinematic fallback for base_footprint if TF lookup unavailable
                # Camera mount: x=0.22, y=0.0, z=1.208, pitch down = 20 deg (0.349 rad)
                if det_info['rel_pos'] is None:
                    pitch = 0.34906585
                    cos_p = math.cos(pitch)
                    sin_p = math.sin(pitch)
                    base_x = 0.22 + (opt_z * cos_p - opt_y * sin_p)
                    base_y = -opt_x
                    base_z = 1.20813 - (opt_z * sin_p + opt_y * cos_p)
                    det_info['rel_pos'] = [base_x, base_y, base_z]
                        
                # Transform to world_frame ('map' or 'odom') for true earth-referenced velocity tracking
                if tf_cam_to_world is not None:
                    try:
                        pt_world = tf2_geometry_msgs.do_transform_point(pt_optical, tf_cam_to_world)
                        det_info['world_pos'] = [pt_world.point.x, pt_world.point.y, pt_world.point.z]
                    except Exception:
                        det_info['world_pos'] = None
                else:
                    det_info['world_pos'] = None
                    
            projected_objects.append(det_info)

        # 3. Object Tracking & Velocity Calculation (strictly in world_frame)
        self.update_tracks(projected_objects, now, world_frame)

        # 4. Render Annotated Image for RViz Image View
        annotated_image = self.draw_annotated_feed(color_image, projected_objects)
        
        # Publish Annotated Image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = header
            self.annotated_image_pub.publish(annotated_msg)
            self.debug_image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish annotated image: {e}")

        # 5. Publish Semantic Objects (Tracked & Raw)
        self.publish_semantic_messages(header, world_frame)

    def update_tracks(self, detected_objects, current_time, world_frame):
        """Associates detections with existing tracks and estimates velocity in the world frame."""
        # Clean up stale tracks rapidly (350ms expiry so past locations do not linger)
        self.tracked_objects = [t for t in self.tracked_objects if (current_time - t.last_seen) < self.track_expiry_sec]
        
        if world_frame is None:
            return

        # If world frame switched (e.g. odom -> map), flush tracks to avoid offset jumps
        if not hasattr(self, 'current_tracking_frame') or self.current_tracking_frame != world_frame:
            self.current_tracking_frame = world_frame
            self.tracked_objects.clear()
            
        matched_track_ids = set()
        
        for det in detected_objects:
            if not det['has_3d'] or det['world_pos'] is None:
                continue
                
            wx, wy, wz = det['world_pos']
            best_match = None
            min_dist = self.distance_match_threshold
            
            for track in self.tracked_objects:
                if track.id in matched_track_ids or track.class_name != det['class']:
                    continue
                d = math.hypot(wx - track.x, wy - track.y)
                if d < min_dist:
                    min_dist = d
                    best_match = track
                    
            if best_match is not None:
                best_match.update(wx, wy, wz, current_time)
                matched_track_ids.add(best_match.id)
                det['track'] = best_match
            else:
                new_track = TrackedObject(self.next_track_id, det['class'], wx, wy, wz, current_time=current_time, frame_id=world_frame)
                self.next_track_id += 1
                self.tracked_objects.append(new_track)
                matched_track_ids.add(new_track.id)
                det['track'] = new_track

        # Deduplicate redundant tracks of the same class within 0.7m (removes trailing ghost tracks)
        deduped = []
        for t in self.tracked_objects:
            duplicate = False
            for d in deduped:
                if t.class_name == d.class_name and math.hypot(t.x - d.x, t.y - d.y) < 0.7:
                    duplicate = True
                    break
            if not duplicate:
                deduped.append(t)
        self.tracked_objects = deduped

    def draw_annotated_feed(self, image, detections):
        """Draws bounding boxes, labels, relative coordinates, and velocity arrows on the camera image."""
        vis = image.copy()
        ih, iw = vis.shape[:2]
        
        # Draw HUD Header banner
        cv2.rectangle(vis, (0, 0), (iw, 32), (30, 30, 30), -1)
        cv2.line(vis, (0, 32), (iw, 32), (0, 200, 255), 1)
        
        hud_text = f"Hospobot Perception [{self.detector_type}] | FPS: {self.current_fps:.1f} | Active Tracks: {len(self.tracked_objects)}"
        cv2.putText(vis, hud_text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

        for det in detections:
            x1, y1, x2, y2 = det['box']
            cls_name = det['class']
            conf = det['conf']
            
            # Select color based on class
            if cls_name == 'HUMAN':
                color = (0, 255, 0) # Bright Green
            elif cls_name == 'BED':
                color = (0, 140, 255) # Orange
            elif cls_name == 'CHAIR':
                color = (255, 255, 0) # Cyan
            else:
                color = (200, 200, 200)
                
            # Draw Bounding Box with corner highlights
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            corner_len = min(20, min((x2 - x1) // 4, (y2 - y1) // 4))
            # Top-left corner
            cv2.line(vis, (x1, y1), (x1 + corner_len, y1), color, 3)
            cv2.line(vis, (x1, y1), (x1, y1 + corner_len), color, 3)
            # Bottom-right corner
            cv2.line(vis, (x2, y2), (x2 - corner_len, y2), color, 3)
            cv2.line(vis, (x2, y2), (x2, y2 - corner_len), color, 3)
            
            track = det.get('track', None)
            track_id_str = f" [ID #{track.id}]" if track else ""
            
            # Top Label Banner
            label_top = f"{cls_name}: {int(conf * 100)}%{track_id_str}"
            (tw, th), _ = cv2.getTextSize(label_top, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(vis, (x1, max(32, y1 - th - 8)), (x1 + tw + 10, max(32, y1)), color, -1)
            cv2.putText(vis, label_top, (x1 + 5, max(32, y1) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
            
            # Bottom Info Overlay (Position, Distance & Velocity)
            info_lines = []
            if det['rel_pos'] is not None:
                rx, ry, _ = det['rel_pos']
                info_lines.append(f"Dist: {det['distance']:.2f}m (X: {rx:+.2f}m, Y: {ry:+.2f}m)")
            elif det['has_3d']:
                info_lines.append(f"Dist: {det['distance']:.2f}m")
            else:
                info_lines.append("Depth: Searching...")
                
            if track is not None:
                speed = math.hypot(track.vx, track.vy)
                if speed > 0.05:
                    info_lines.append(f"Speed: {speed:.2f} m/s (vx: {track.vx:+.2f}, vy: {track.vy:+.2f})")
                    
                    # Draw a 2D motion vector arrow from box center
                    cx_box = (x1 + x2) // 2
                    cy_box = (y1 + y2) // 2
                    # Scale velocity for 2D visual arrow: vx is forward/backward, vy is lateral
                    arrow_dx = int(-track.vy * 60) # lateral moves horizontally on screen
                    arrow_dy = int(-track.vx * 30) # forward moves up/down
                    if abs(arrow_dx) > 3 or abs(arrow_dy) > 3:
                        cv2.arrowedLine(vis, (cx_box, cy_box), (cx_box + arrow_dx, cy_box + arrow_dy), (0, 0, 255), 2, tipLength=0.3)
                else:
                    info_lines.append("Speed: 0.00 m/s (Stationary)")

            # Render info lines under the bounding box
            text_y = y2 + 16
            for line in info_lines:
                if text_y < ih - 6:
                    (lw, lh), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
                    cv2.rectangle(vis, (x1, text_y - lh - 2), (x1 + lw + 6, text_y + 2), (20, 20, 20), -1)
                    cv2.putText(vis, line, (x1 + 3, text_y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)
                    text_y += lh + 6

        return vis

    def publish_semantic_messages(self, header, world_frame):
        """Publishes SemanticObjectArray for navigation and costmap layers."""
        array_msg = SemanticObjectArray()
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = world_frame or 'map'
        
        marker_array = MarkerArray()
        
        for track in self.tracked_objects:
            s_obj = SemanticObject()
            s_obj.class_name = track.class_name
            s_obj.tracking_id = track.id
            s_obj.position.x = track.x
            s_obj.position.y = track.y
            s_obj.position.z = track.z
            s_obj.velocity.x = track.vx
            s_obj.velocity.y = track.vy
            s_obj.velocity.z = track.vz
            s_obj.confidence = 1.0
            array_msg.objects.append(s_obj)

            # RViz 3D Box Marker
            cube = Marker()
            cube.header = array_msg.header
            cube.ns = "semantic_tracked_boxes"
            cube.id = track.id
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = track.x
            cube.pose.position.y = track.y
            cube.pose.position.z = 0.85 if track.class_name == 'HUMAN' else 0.4
            
            if track.class_name == 'HUMAN':
                cube.scale.x = 0.5
                cube.scale.y = 0.5
                cube.scale.z = 1.7
                cube.color.r = 0.0
                cube.color.g = 1.0
                cube.color.b = 0.2
                cube.color.a = 0.4
            else:
                cube.scale.x = 0.6
                cube.scale.y = 0.6
                cube.scale.z = 0.6
                cube.color.r = 1.0
                cube.color.g = 0.8
                cube.color.b = 0.0
                cube.color.a = 0.4
                
            cube.lifetime.sec = 1
            marker_array.markers.append(cube)

            # RViz Velocity Arrow Marker
            speed = math.hypot(track.vx, track.vy)
            if speed > 0.15:
                arrow = Marker()
                arrow.header = array_msg.header
                arrow.ns = "semantic_velocity_arrows"
                arrow.id = track.id + 10000
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                
                p_start = Point(x=track.x, y=track.y, z=1.0)
                p_end = Point(x=track.x + track.vx, y=track.y + track.vy, z=1.0)
                arrow.points = [p_start, p_end]
                
                arrow.scale.x = 0.06 # shaft diameter
                arrow.scale.y = 0.12 # head diameter
                arrow.scale.z = 0.15 # head length
                arrow.color.r = 1.0
                arrow.color.g = 0.1
                arrow.color.b = 0.1
                arrow.color.a = 0.9
                arrow.lifetime.sec = 1
                marker_array.markers.append(arrow)

        # Publish semantic objects on all standard topics
        self.tracked_objects_pub.publish(array_msg)
        self.raw_objects_pub.publish(array_msg)
        
        # Only publish static furniture (e.g. BED) to legacy /oakd/semantic_objects
        # Do NOT publish HUMAN to legacy topic to avoid Nav2's 1.2m lethal inflation blocking hallway!
        legacy_msg = SemanticObjectArray()
        legacy_msg.header = array_msg.header
        legacy_msg.objects = [o for o in array_msg.objects if o.class_name != 'HUMAN']
        self.legacy_objects_pub.publish(legacy_msg)
        
        if marker_array.markers:
            self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
