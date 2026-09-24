#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserFilterNode(Node):
    def __init__(self):
        super().__init__('laser_filter_node')
        
        self.declare_parameter('flip_angles', False)
        self.declare_parameter('nav_mode', 'mapping')
        self.flip_angles = self.get_parameter('flip_angles').get_parameter_value().bool_value
        self.nav_mode = self.get_parameter('nav_mode').get_parameter_value().string_value
        
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
            
        mode_str = f"{'FLIPPED 180°' if self.flip_angles else 'STANDARD'} | Nav Mode: {self.nav_mode.upper()}"
        self.get_logger().info(f'Laser Filter Node initialized. Mode: {mode_str}')

    def scan_callback(self, msg):
        filtered_msg = msg
        # Convert tuple to list to modify ranges
        ranges = list(msg.ranges)
        
        # Re-check parameters in case updated dynamically
        flip = self.get_parameter('flip_angles').get_parameter_value().bool_value
        current_nav_mode = self.get_parameter('nav_mode').get_parameter_value().string_value
        
        is_localization = (current_nav_mode == 'localization')

        for i in range(len(ranges)):
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad) % 360.0
            
            blocked = False
            
            if is_localization:
                # Autonomous Navigation / Localization Mode:
                # Robot navigates autonomously with no operator walking behind it.
                # The rear 66° opening (327° to 33°) is unblocked and used by AMCL.
                # Only the 4 structural chassis poles are blocked (each ~24° wide):
                # 1. Rear Left Pole (45°): 33° to 57°
                # 2. Front Left Pole (135°): 123° to 147°
                # 3. Front Right Pole (225°): 213° to 237°
                # 4. Rear Right Pole (315°): 303° to 327°
                if (33.0 <= angle_deg <= 57.0) or \
                   (123.0 <= angle_deg <= 147.0) or \
                   (213.0 <= angle_deg <= 237.0) or \
                   (303.0 <= angle_deg <= 327.0):
                    blocked = True
            elif not flip:
                # Standard Mapping Mode:
                # Operator walks behind the robot (at 0° / 360° rear).
                # 1. Rear Wedge (User + Rear Poles): 303° to 57° (114° blocked)
                # 2. Front Left Pole (135°): 123° to 147°
                # 3. Front Right Pole (225°): 213° to 237°
                if angle_deg >= 303.0 or angle_deg <= 57.0:
                    blocked = True
                elif 123.0 <= angle_deg <= 147.0:
                    blocked = True
                elif 213.0 <= angle_deg <= 237.0:
                    blocked = True
            else:
                # Flipped Mapping Mode:
                # Operator stands in front of the robot (180°).
                # 1. Front Wedge (User in front): 123° to 237°
                # 2. Rear Right Pole: 303° to 327°
                # 3. Rear Left Pole: 33° to 57°
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
