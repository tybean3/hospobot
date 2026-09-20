#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserFilterNode(Node):
    def __init__(self):
        super().__init__('laser_filter_node')
        
        self.declare_parameter('flip_angles', False)
        self.flip_angles = self.get_parameter('flip_angles').get_parameter_value().bool_value
        
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
            
        mode_str = "FLIPPED 180° (Operator In Front / Mapping)" if self.flip_angles else "STANDARD (Operator At Rear / Driving)"
        self.get_logger().info(f'Laser Filter Node initialized. Mode: {mode_str}')

    def scan_callback(self, msg):
        filtered_msg = msg
        # Convert tuple to list to modify ranges
        ranges = list(msg.ranges)
        
        # Re-check parameter in case updated dynamically
        flip = self.get_parameter('flip_angles').get_parameter_value().bool_value
        
        for i in range(len(ranges)):
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad) % 360.0
            
            blocked = False
            
            if not flip:
                # Standard mode (Lidar is mounted backwards; 0 degrees is user at robot rear)
                # 1. Rear Wedge (User + Rear Poles): 303 to 57 degrees
                # 2. Left Pole (135): 123 to 147 degrees
                # 3. Right Pole (225): 213 to 237 degrees
                if angle_deg >= 303.0 or angle_deg <= 57.0:
                    blocked = True
                elif 123.0 <= angle_deg <= 147.0:
                    blocked = True
                elif 213.0 <= angle_deg <= 237.0:
                    blocked = True
            else:
                # Mapping mode: angles rotated by 180 degrees.
                # Operator stands in front of the robot (180 degrees) looking at the screen.
                # 1. Front Wedge (User in front): 123 to 237 degrees (180 +/- 57 deg)
                # 2. Pole 1 (315 deg): 303 to 327 degrees (135 + 180 = 315 +/- 12 deg)
                # 3. Pole 2 (45 deg): 33 to 57 degrees (225 + 180 = 45 +/- 12 deg)
                if 123.0 <= angle_deg <= 237.0:
                    blocked = True
                elif 303.0 <= angle_deg <= 327.0:
                    blocked = True
                elif 33.0 <= angle_deg <= 57.0:
                    blocked = True
                
            # Chassis self-reflection and floor bump strike rejection:
            # Any reading closer than 0.22m is robot chassis/cables/floor glare
            r = ranges[i]
            if not math.isnan(r) and (r < 0.22 or r > 12.0):
                blocked = True
                
            if blocked:
                ranges[i] = float('nan') # Set to NaN so SLAM completely ignores it (does not clear space)
                
        filtered_msg.ranges = ranges
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
