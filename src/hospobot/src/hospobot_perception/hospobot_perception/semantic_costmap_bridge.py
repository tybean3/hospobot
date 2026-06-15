#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hospobot_interfaces.msg import SemanticObjectArray
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class SemanticCostmapBridge(Node):
    def __init__(self):
        super().__init__('semantic_costmap_bridge')
        self.get_logger().info('Initializing Semantic Costmap Bridge Node...')
        
        self.sub = self.create_subscription(SemanticObjectArray, '/semantic/tracked_objects', self.callback, 10)
        self.pub = self.create_publisher(PointCloud2, '/semantic/obstacle_cloud', 10)
        
        self.default_radius = 0.3
        self.points_per_cluster = 16 # Number of points to approximate the circle
        
        # Load the Semantic Database
        self.load_database()

    def load_database(self):
        self.db = {}
        try:
            pkg_share = get_package_share_directory('hospobot_perception')
            db_path = os.path.join(pkg_share, 'config', 'semantic_objects_db.yaml')
            with open(db_path, 'r') as f:
                self.db = yaml.safe_load(f)
            self.get_logger().info(f"Loaded Semantic Object Database from {db_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load semantic DB: {e}. Using safe defaults.")

    def callback(self, msg):
        cloud_points = []
        
        semantic_classes = self.db.get('semantic_classes', {})
        
        for obj in msg.objects:
            # Fetch object rules from DB
            obj_rules = semantic_classes.get(obj.class_name, {})
            costmap_rules = obj_rules.get('costmap_injection', {})
            behavior_rules = obj_rules.get('behavior_rules', {})
            
            base_radius = costmap_rules.get('base_radius', self.default_radius)
            inflation_factor = costmap_rules.get('velocity_inflation_factor', 1.0)
            velocity_threshold = behavior_rules.get('approach_velocity_threshold', 1.0)
            forces_reroute = costmap_rules.get('forces_reroute', False)
            
            # Inflate radius if object is moving faster than its threshold
            speed = np.sqrt(obj.velocity.x**2 + obj.velocity.y**2)
            if speed > velocity_threshold:
                base_radius *= inflation_factor
                
            # If the object explicitly forces a reroute, make the radius massive to block the global costmap
            if forces_reroute:
                base_radius = max(base_radius, 2.5) # 2.5m radius = 5m wide obstacle
                
            # Generate a cluster of points around the center to force costmap inflation
            for i in range(self.points_per_cluster):
                angle = i * (2.0 * np.pi / self.points_per_cluster)
                px = obj.position.x + base_radius * np.cos(angle)
                py = obj.position.y + base_radius * np.sin(angle)
                pz = obj.position.z # Usually ignored by 2D costmaps, but keep it accurate
                cloud_points.append([px, py, pz])
                
            # Add the center point as well
            cloud_points.append([obj.position.x, obj.position.y, obj.position.z])
            
        # Density-based crowd detection (if 3 or more objects are close to each other)
        # We calculate pairwise distances and find clusters
        if len(msg.objects) >= 3:
            for i, obj1 in enumerate(msg.objects):
                close_neighbors = []
                for j, obj2 in enumerate(msg.objects):
                    if i != j:
                        dist = np.sqrt((obj1.position.x - obj2.position.x)**2 + (obj1.position.y - obj2.position.y)**2)
                        if dist < 2.0: # 2 meters
                            close_neighbors.append(obj2)
                
                if len(close_neighbors) >= 2: # obj1 + 2 neighbors = 3 objects
                    # Crowd detected, block the hallway at obj1's position
                    base_radius = 2.5
                    for k in range(self.points_per_cluster):
                        angle = k * (2.0 * np.pi / self.points_per_cluster)
                        px = obj1.position.x + base_radius * np.cos(angle)
                        py = obj1.position.y + base_radius * np.sin(angle)
                        cloud_points.append([px, py, obj1.position.z])
                    cloud_points.append([obj1.position.x, obj1.position.y, obj1.position.z])
                    break # Only need to generate one massive obstacle for the crowd
            
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
        
        if not cloud_points:
            # Publish empty cloud to clear old obstacles
            cloud_msg = pc2.create_cloud_xyz32(header, [])
        else:
            cloud_msg = pc2.create_cloud_xyz32(header, cloud_points)
            
        self.pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticCostmapBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
