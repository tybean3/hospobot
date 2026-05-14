#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import message_filters

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        
        self.bridge = CvBridge()
        
        # Initialize HOG descriptor for human detection
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        # Initialize Face detector as a fallback
        cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        
        # Initialize ORB for general object feature matching
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Load reference images for object/staff matching
        self.load_reference_images()
        
        # Subscriptions using message_filters to synchronize color and depth
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        
        # ApproximateTimeSynchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)
        
        # Camera Info subscriber to get intrinsic parameters
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        self.camera_info = None
        
        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/human_bbox_marker', 10)
        self.image_pub = self.create_publisher(Image, '/human_detector/debug_image', 10)
        
        self.get_logger().info("Hospital Object & Human detector node started.")
        self.get_logger().info("Training images should be in 'hospobot_ws/object-training/[label]/'")

    def load_reference_images(self):
        self.references = {} # {label: [{'hist': h, 'kp': k, 'des': d}, ...]}
        base_path = '/home/hospobot/hospobot_ws/object-training'
        if not os.path.exists(base_path):
            os.makedirs(base_path, exist_ok=True)
            self.get_logger().info(f"Created empty reference directory at {base_path}")
            return
            
        count = 0
        for label in os.listdir(base_path):
            label_path = os.path.join(base_path, label)
            if not os.path.isdir(label_path):
                continue
            
            self.references[label] = []
            for img_file in os.listdir(label_path):
                if not img_file.lower().endswith(('.png', '.jpg', '.jpeg')):
                    continue
                img_path = os.path.join(label_path, img_file)
                img = cv2.imread(img_path)
                if img is None:
                    continue
                
                # 1. Color Histogram (for clothes/scrubs)
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                hist = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
                cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
                
                # 2. ORB Features (for textured objects like equipment)
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                kp, des = self.orb.detectAndCompute(gray, None)
                
                self.references[label].append({
                    'hist': hist,
                    'kp': kp,
                    'des': des,
                    'img_shape': gray.shape
                })
                count += 1
        
        self.get_logger().info(f"Loaded {count} reference images across {len(self.references)} classes.")

    def match_reference(self, roi):
        if not self.references or roi.size == 0:
            return "Unknown", 0.0
            
        # ROI Color Hist
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        roi_hist = cv2.calcHist([hsv_roi], [0, 1], None, [30, 32], [0, 180, 0, 256])
        cv2.normalize(roi_hist, roi_hist, 0, 1, cv2.NORM_MINMAX)
        
        # ROI ORB Features
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        kp_roi, des_roi = self.orb.detectAndCompute(gray_roi, None)
        
        best_label = "Unknown"
        best_score = -1.0
        
        for label, refs in self.references.items():
            for ref in refs:
                # Color correlation
                color_score = cv2.compareHist(roi_hist, ref['hist'], cv2.HISTCMP_CORREL)
                
                # ORB matching score
                orb_score = 0.0
                if des_roi is not None and ref['des'] is not None:
                    matches = self.bf.match(des_roi, ref['des'])
                    if len(matches) > 10:
                        # Normalize match count by min features
                        orb_score = len(matches) / min(len(des_roi), len(ref['des']))
                
                # Combined score (weighted)
                combined_score = color_score * 0.7 + orb_score * 0.3
                
                if combined_score > best_score:
                    best_score = combined_score
                    best_label = label
                    
        return best_label, best_score

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.destroy_subscription(self.camera_info_sub)

    def sync_callback(self, color_msg, depth_msg):
        if self.camera_info is None:
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
        except Exception as e:
            return
            
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        debug_image = color_image.copy()
        
        # 1. Detect Humans/Faces
        boxes, weights = self.hog.detectMultiScale(gray, winStride=(4,4), padding=(8,8), scale=1.03)
        
        detections = []
        # Process HOG detections
        for i, (box, weight) in enumerate(zip(boxes, weights)):
            if weight < 0.3: continue
            detections.append({'box': box, 'is_face': False, 'type': 'Human'})
            
        # Fallback to Face if no full humans
        if len(detections) == 0:
            faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
            for i, face in enumerate(faces):
                detections.append({'box': face, 'is_face': True, 'type': 'Face'})
        
        for i, det in enumerate(detections):
            x, y, w, h = det['box']
            roi = self.get_shirt_roi(color_image, det['box'], det['is_face'])
            
            # Match
            label, score = self.match_reference(roi)
            # Log score to help user debug their training images
            if score > 0.1:
                self.get_logger().info(f"Det {i} match: {label} (score: {score:.3f})")
            
            # Use a slightly lower threshold for better recall
            final_label = label.upper() if score > 0.35 else "UNKNOWN"
            
            # Depth/3D
            cx, cy = x + w // 2, y + h // 2
            z_val = self.get_depth_at(depth_image, cx, cy)
            
            if z_val > 0:
                real_x, real_y, real_z = self.project_3d(cx, cy, z_val)
                
                # If it's a face, we want the marker to represent the whole person
                if det['is_face']:
                    # Estimate person dimensions based on face size
                    width_3d = w * z_val / self.camera_info.k[0] * 3.0 # Body is wider than face
                    height_3d = h * z_val / self.camera_info.k[4] * 6.0 # Person is ~6x face height
                    # Offset the marker center to the torso (below the face)
                    real_y += height_3d * 0.4
                else:
                    width_3d = w * z_val / self.camera_info.k[0]
                    height_3d = h * z_val / self.camera_info.k[4]
                
                print(f"{det['type']} {i}: {final_label} at {real_z:.2f}m")
                
                # Draw on Debug Image
                color = self.get_color_for_label(final_label)
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), color, 2)
                # Also draw the ROI we are using for matching to show the user what we see
                if det['is_face']:
                    rx, ry, rw, rh = self.get_roi_coords(color_image.shape, det['box'], True)
                    cv2.rectangle(debug_image, (rx, ry), (rx + rw, ry + rh), (255, 0, 255), 1)
                
                cv2.putText(debug_image, f"{final_label} {real_z:.2f}m", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                # Publish Marker
                self.publish_marker(real_x, real_y, real_z, width_3d, height_3d, i, final_label)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8'))

    def get_roi_coords(self, img_shape, box, is_face):
        x, y, w, h = box
        ih, iw = img_shape[:2]
        if is_face:
            # ROI is below the face
            ry, rh, rx, rw = int(y + h * 1.1), int(h * 2.0), int(x - w * 0.5), int(w * 2.0)
        else:
            # ROI is upper torso
            ry, rh, rx, rw = int(y + h * 0.2), int(h * 0.4), int(x + w * 0.1), int(w * 0.8)
        
        rx, ry = max(0, min(rx, iw-1)), max(0, min(ry, ih-1))
        rw, rh = max(1, min(rw, iw-rx)), max(1, min(rh, ih-ry))
        return rx, ry, rw, rh

    def get_shirt_roi(self, image, box, is_face):
        rx, ry, rw, rh = self.get_roi_coords(image.shape, box, is_face)
        return image[ry:ry+rh, rx:rx+rw]

    def get_depth_at(self, depth_image, cx, cy):
        h, w = depth_image.shape
        cx, cy = max(0, min(cx, w-1)), max(0, min(cy, h-1))
        # Use a larger patch for depth to handle noise
        patch = depth_image[max(0, cy-15):min(h, cy+15), max(0, cx-15):min(w, cx+15)]
        valid = patch[patch > 0]
        return np.median(valid) / 1000.0 if len(valid) > 0 else 0.0

    def project_3d(self, u, v, z):
        fx, cx_cam = self.camera_info.k[0], self.camera_info.k[2]
        fy, cy_cam = self.camera_info.k[4], self.camera_info.k[5]
        return (u - cx_cam) * z / fx, (v - cy_cam) * z / fy, z

    def get_color_for_label(self, label):
        if "NURSE" in label: return (255, 255, 0) # Cyan
        if "DOCTOR" in label: return (255, 255, 255) # White
        if "BED" in label: return (0, 165, 255) # Orange
        if "BLACK" in label: return (50, 50, 50) # Dark Gray
        if "WHITE" in label: return (200, 200, 200) # Light Gray
        return (0, 255, 0) # Green

    def publish_marker(self, x, y, z, w, h, id, label):
        marker = Marker()
        # Ensure we use the correct optical frame for alignment
        marker.header.frame_id = "camera_color_optical_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "objects"; marker.id = id; marker.type = Marker.CUBE
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
        marker.scale.x, marker.scale.y, marker.scale.z = w, h, 0.4
        c = self.get_color_for_label(label)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = c[2]/255.0, c[1]/255.0, c[0]/255.0, 0.6
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()

