#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hospobot_interfaces.msg import SemanticObject, SemanticObjectArray
import numpy as np
import time

class TrackedObject:
    def __init__(self, obj_id, class_name, x, y, z):
        self.id = obj_id
        self.class_name = class_name
        self.x = x
        self.y = y
        self.z = z
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.last_seen = time.time()
        self.decay_time = 1.5 # seconds

    def update(self, x, y, z, dt):
        alpha = 0.6 # Simple smoothing factor
        self.vx = (x - self.x) / dt * alpha + self.vx * (1 - alpha)
        self.vy = (y - self.y) / dt * alpha + self.vy * (1 - alpha)
        self.vz = (z - self.z) / dt * alpha + self.vz * (1 - alpha)
        self.x = x
        self.y = y
        self.z = z
        self.last_seen = time.time()
        
    def predict(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

    def is_alive(self):
        return (time.time() - self.last_seen) < self.decay_time

class SemanticTrackerNode(Node):
    def __init__(self):
        super().__init__('semantic_tracker_node')
        self.get_logger().info('Initializing Semantic Tracker Node...')
        
        self.sub = self.create_subscription(SemanticObjectArray, '/semantic/raw_objects', self.callback, 10)
        self.pub = self.create_publisher(SemanticObjectArray, '/semantic/tracked_objects', 10)
        
        self.tracked_objects = []
        self.next_id = 1
        self.distance_threshold = 1.5 # meters
        self.last_update_time = time.time()
        
        # Publisher loop for decay and prediction
        self.timer = self.create_timer(0.1, self.publish_tracked_objects)

    def callback(self, msg):
        current_time = time.time()
        dt = current_time - self.last_update_time
        if dt <= 0:
            dt = 0.01
            
        self.last_update_time = current_time

        # Predict current tracked objects
        for t_obj in self.tracked_objects:
            t_obj.predict(dt)

        # Match new detections to existing tracks
        matched_indices = set()
        
        for new_obj in msg.objects:
            best_match_idx = -1
            min_dist = self.distance_threshold
            
            for i, t_obj in enumerate(self.tracked_objects):
                if i in matched_indices or t_obj.class_name != new_obj.class_name:
                    continue
                    
                dist = np.sqrt((new_obj.position.x - t_obj.x)**2 + (new_obj.position.y - t_obj.y)**2 + (new_obj.position.z - t_obj.z)**2)
                if dist < min_dist:
                    min_dist = dist
                    best_match_idx = i
                    
            if best_match_idx != -1:
                # Update existing
                self.tracked_objects[best_match_idx].update(new_obj.position.x, new_obj.position.y, new_obj.position.z, dt)
                matched_indices.add(best_match_idx)
            else:
                # Create new
                new_track = TrackedObject(self.next_id, new_obj.class_name, new_obj.position.x, new_obj.position.y, new_obj.position.z)
                self.tracked_objects.append(new_track)
                self.next_id += 1

    def publish_tracked_objects(self):
        # Cull decayed objects
        self.tracked_objects = [obj for obj in self.tracked_objects if obj.is_alive()]
        
        msg = SemanticObjectArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        
        for t_obj in self.tracked_objects:
            s_obj = SemanticObject()
            s_obj.class_name = t_obj.class_name
            s_obj.tracking_id = t_obj.id
            s_obj.position.x = t_obj.x
            s_obj.position.y = t_obj.y
            s_obj.position.z = t_obj.z
            s_obj.velocity.x = t_obj.vx
            s_obj.velocity.y = t_obj.vy
            s_obj.velocity.z = t_obj.vz
            s_obj.confidence = 1.0 # Assume confident if tracked
            msg.objects.append(s_obj)
            
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
