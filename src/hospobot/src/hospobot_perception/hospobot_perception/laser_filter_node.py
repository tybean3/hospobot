#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserFilterNode(Node):
    def __init__(self):
        super().__init__('laser_filter_node')
        
        # Subscribe to raw scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data)
            
        # Publish filtered scan
        self.publisher = self.create_publisher(
            LaserScan, 
            '/scan', 
            rclpy.qos.qos_profile_sensor_data)
            
        self.get_logger().info('Laser Filter Node initialized. Blocking poles.')

    def scan_callback(self, msg):
        filtered_msg = msg
        # Convert tuple to list to modify ranges
        ranges = list(msg.ranges)
        
        for i in range(len(ranges)):
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad) % 360.0
            
            # The Lidar is mounted backwards! 0 degrees is the user (robot rear).
            # 1. Front Wedge (User + Front Poles): 303 to 57 degrees
            # 2. Rear Left Pole (135): 123 to 147 degrees
            # 3. Rear Right Pole (225): 213 to 237 degrees
            
            blocked = False
            
            # Front Wedge (User + Front Poles)
            if angle_deg >= 303.0 or angle_deg <= 57.0:
                blocked = True
            # Rear Left Pole (135)
            elif 123.0 <= angle_deg <= 147.0:
                blocked = True
            # Rear Right Pole (225)
            elif 213.0 <= angle_deg <= 237.0:
                blocked = True
                
            if blocked:
                ranges[i] = float('nan') # Set to NaN so SLAM completely ignores it (does not clear space)
                
        filtered_msg.ranges = ranges
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
