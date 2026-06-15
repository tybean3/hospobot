#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np

class LidarFeatureExperiment(Node):
    def __init__(self):
        super().__init__('lidar_feature_experiment')
        
        # Subscribe to the 2D lidar scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for RViz visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/lidar_features', 10)
        
        # Parameters for clustering
        self.jump_distance_threshold = 0.25 # meters (distance between consecutive points to be considered a new cluster)
        self.min_cluster_size = 4 # minimum number of points to form a valid feature
        
        self.get_logger().info("Lidar Feature Detection Experiment Started!")
        self.get_logger().info("Listening to /scan and publishing clusters to /lidar_features")

    def scan_callback(self, msg):
        # Convert polar (ranges, angles) to Cartesian (x, y)
        points = []
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue
            
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y, r))
            
        if not points:
            return
            
        # Segment into clusters based on jump distance (Simple Euclidean Distance Clustering)
        clusters = []
        current_cluster = [points[0]]
        
        for i in range(1, len(points)):
            p1 = points[i-1]
            p2 = points[i]
            
            # Euclidean distance between consecutive points
            dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
            
            if dist < self.jump_distance_threshold:
                current_cluster.append(p2)
            else:
                if len(current_cluster) >= self.min_cluster_size:
                    clusters.append(current_cluster)
                current_cluster = [p2]
                
        if len(current_cluster) >= self.min_cluster_size:
            clusters.append(current_cluster)
            
        # Check if the first and last cluster should be merged (since lidar is 360 degrees)
        if len(clusters) > 1:
            p_first = clusters[0][0]
            p_last = clusters[-1][-1]
            if math.sqrt((p_first[0]-p_last[0])**2 + (p_first[1]-p_last[1])**2) < self.jump_distance_threshold:
                clusters[0] = clusters[-1] + clusters[0]
                clusters.pop()

        self.publish_feature_markers(clusters, msg.header)

    def publish_feature_markers(self, clusters, header):
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        for i, cluster in enumerate(clusters):
            # Calculate centroid
            x_coords = [p[0] for p in cluster]
            y_coords = [p[1] for p in cluster]
            
            centroid_x = sum(x_coords) / len(cluster)
            centroid_y = sum(y_coords) / len(cluster)
            
            # Create a cube marker for the cluster centroid
            marker = Marker()
            marker.header = header
            marker.ns = "lidar_clusters"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(centroid_x)
            marker.pose.position.y = float(centroid_y)
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Size of sphere based on cluster spread
            width = max(x_coords) - min(x_coords)
            height = max(y_coords) - min(y_coords)
            size = max(0.1, max(width, height)) # Minimum 10cm sphere
            
            marker.scale.x = float(size)
            marker.scale.y = float(size)
            marker.scale.z = float(size)
            
            # Assign distinct colors based on ID
            np.random.seed(i * 42)
            marker.color.r = float(np.random.rand())
            marker.color.g = float(np.random.rand())
            marker.color.b = float(np.random.rand())
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFeatureExperiment()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
