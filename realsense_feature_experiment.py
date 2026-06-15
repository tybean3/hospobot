#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
import message_filters
from cv_bridge import CvBridge
import math
import numpy as np

class RealsenseFeatureExperiment(Node):
    def __init__(self):
        super().__init__('realsense_feature_experiment')
        
        self.bridge = CvBridge()
        
        # Synchronize Depth and RGB images to prove RGBD superiority
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.color_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.info_callback,
            10
        )
        
        self.marker_pub = self.create_publisher(MarkerArray, '/realsense_features', 10)
        
        self.jump_distance_threshold = 0.25 # meters
        self.min_cluster_size = 5 # pixels
        self.max_depth = 5.0 # meters
        
        self.fx = 380.0
        self.cx = 320.0
        self.fy = 380.0
        self.cy = 240.0
        
        self.get_logger().info("Advanced RGBD Feature Detection Experiment Started!")
        self.get_logger().info("Extracting both 3D Spatial Height AND Semantic Color...")

    def info_callback(self, msg):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]
        self.destroy_subscription(self.camera_info_sub)

    def sync_callback(self, depth_msg, color_msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting images: {e}")
            return
            
        height, width = depth_image.shape
        middle_row = height // 2
        
        depth_slice = depth_image[middle_row-10:middle_row+10, :]
        median_depths = np.median(depth_slice, axis=0)
        
        points = []
        for u in range(width):
            z_mm = median_depths[u]
            if z_mm == 0 or np.isnan(z_mm):
                continue
            z = z_mm / 1000.0
            if z > self.max_depth:
                continue
            
            x = (u - self.cx) * z / self.fx
            points.append({'u': u, 'x': x, 'z': z})
            
        if not points:
            return
            
        clusters = []
        current_cluster = [points[0]]
        
        for i in range(1, len(points)):
            p1 = points[i-1]
            p2 = points[i]
            
            dist = math.sqrt((p1['x']-p2['x'])**2 + (p1['z']-p2['z'])**2)
            
            if dist < self.jump_distance_threshold:
                current_cluster.append(p2)
            else:
                if len(current_cluster) >= self.min_cluster_size:
                    clusters.append(current_cluster)
                current_cluster = [p2]
                
        if len(current_cluster) >= self.min_cluster_size:
            clusters.append(current_cluster)
            
        self.publish_feature_markers(clusters, depth_msg.header, color_image, depth_image, middle_row)

    def publish_feature_markers(self, clusters, header, color_image, depth_image, middle_row):
        marker_array = MarkerArray()
        
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        for i, cluster in enumerate(clusters):
            x_coords = [p['x'] for p in cluster]
            z_coords = [p['z'] for p in cluster]
            u_coords = [p['u'] for p in cluster]
            
            centroid_x = sum(x_coords) / len(cluster)
            centroid_z = sum(z_coords) / len(cluster)
            
            # 1. SEMANTIC CAPABILITY: Extract the exact real-world color of this object!
            # (A 2D Lidar is completely blind to this)
            u_min, u_max = min(u_coords), max(u_coords)
            color_patch = color_image[middle_row-20:middle_row+20, u_min:u_max]
            
            if color_patch.size > 0:
                avg_color = np.mean(color_patch, axis=(0, 1))
                b, g, r = avg_color[0]/255.0, avg_color[1]/255.0, avg_color[2]/255.0
            else:
                r, g, b = 0.5, 0.5, 0.5
                
            # 2. 3D GEOMETRY CAPABILITY: Estimate the physical height of the object!
            # By scanning vertically along the depth column (Lidar only sees a 1D slice)
            center_u = int(sum(u_coords) / len(cluster))
            base_z = centroid_z
            
            top_v = middle_row
            for v in range(middle_row, 0, -5):
                z = depth_image[v, center_u] / 1000.0
                if z == 0 or abs(z - base_z) > 0.3:
                    break
                top_v = v
                
            bottom_v = middle_row
            for v in range(middle_row, depth_image.shape[0]-1, 5):
                z = depth_image[v, center_u] / 1000.0
                if z == 0 or abs(z - base_z) > 0.3:
                    break
                bottom_v = v
                
            # Convert pixels to physical height in meters
            physical_height = abs((bottom_v - top_v) * base_z / self.fy)
            physical_height = max(0.1, physical_height) # At least 10cm visible
            
            marker = Marker()
            marker.header = header
            marker.ns = "realsense_semantic_clusters"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(centroid_x)
            # Center the box vertically
            center_v = (top_v + bottom_v) / 2
            physical_y = (center_v - self.cy) * base_z / self.fy
            marker.pose.position.y = float(physical_y)
            marker.pose.position.z = float(centroid_z)
            
            marker.pose.orientation.w = 1.0
            
            width = max(x_coords) - min(x_coords)
            depth = max(z_coords) - min(z_coords)
            size_x = max(0.1, width)
            size_z = max(0.1, depth)
            
            marker.scale.x = float(size_x)
            marker.scale.y = float(physical_height)
            marker.scale.z = float(size_z)
            
            # Apply the EXACT real-world color to the marker
            marker.color.r = float(r)
            marker.color.g = float(g)
            marker.color.b = float(b)
            marker.color.a = 0.9
            
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RealsenseFeatureExperiment()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
