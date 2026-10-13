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
        self.declare_parameter('nav_fov_deg', 120.0)
        self.declare_parameter('filter_pillars', True)
        self.flip_angles = self.get_parameter('flip_angles').get_parameter_value().bool_value
        self.nav_mode = self.get_parameter('nav_mode').get_parameter_value().string_value
        
        # Subscribe to raw scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data)
            
        # Publish filtered scan (for localization / AMCL / SLAM)
        self.publisher = self.create_publisher(
            LaserScan, 
            '/scan', 
            rclpy.qos.qos_profile_sensor_data)

        # Publish navigation scan for obstacle avoidance (front, left, right only)
        self.nav_publisher = self.create_publisher(
            LaserScan,
            '/scan_nav',
            rclpy.qos.qos_profile_sensor_data)
            
        mode_str = f"{'FLIPPED 180°' if self.flip_angles else 'STANDARD'} | Nav Mode: {self.nav_mode.upper()}"
        self.get_logger().info(f'Laser Filter Node initialized. Mode: {mode_str}')

    def scan_callback(self, msg):
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max

        nav_msg = LaserScan()
        nav_msg.header = msg.header
        nav_msg.angle_min = msg.angle_min
        nav_msg.angle_max = msg.angle_max
        nav_msg.angle_increment = msg.angle_increment
        nav_msg.time_increment = msg.time_increment
        nav_msg.scan_time = msg.scan_time
        nav_msg.range_min = msg.range_min
        nav_msg.range_max = msg.range_max

        # Convert tuples to lists to modify ranges
        ranges = list(msg.ranges)
        nav_ranges = list(msg.ranges)
        
        # Re-check parameters in case updated dynamically
        flip = self.get_parameter('flip_angles').get_parameter_value().bool_value
        current_nav_mode = self.get_parameter('nav_mode').get_parameter_value().string_value
        nav_fov_deg = float(self.get_parameter('nav_fov_deg').get_parameter_value().double_value)
        filter_pillars = self.get_parameter('filter_pillars').get_parameter_value().bool_value
        
        is_localization = (current_nav_mode == 'localization')

        half_fov = nav_fov_deg / 2.0  # 60.0 degrees for 120° FOV
        min_front_deg = (180.0 - half_fov) % 360.0  # 120.0° in standard mode
        max_front_deg = (180.0 + half_fov) % 360.0  # 240.0° in standard mode

        for i in range(len(ranges)):
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad) % 360.0
            
            # --- 1. General scan filtering (used by /scan for AMCL / SLAM) ---
            blocked_general = False
            
            if is_localization:
                # Autonomous Navigation / Localization Mode:
                # Robot navigates autonomously. Rear 66° opening (327° to 33°)
                # is unblocked and used by AMCL map matching.
                # Only the 4 structural chassis poles are blocked (each ~24° wide):
                # 1. Rear Left Pole (45°): 33° to 57°
                # 2. Front Left Pole (135°): 123° to 147°
                # 3. Front Right Pole (225°): 213° to 237°
                # 4. Rear Right Pole (315°): 303° to 327°
                if (33.0 <= angle_deg <= 57.0) or \
                   (123.0 <= angle_deg <= 147.0) or \
                   (213.0 <= angle_deg <= 237.0) or \
                   (303.0 <= angle_deg <= 327.0):
                    blocked_general = True
            elif not flip:
                # Standard Mapping Mode:
                # Operator walks behind the robot (at 0° / 360° rear).
                # 1. Rear Wedge (User + Rear Poles): 303° to 57° (114° blocked)
                # 2. Front Left Pole (135°): 123° to 147°
                # 3. Front Right Pole (225°): 213° to 237°
                if angle_deg >= 303.0 or angle_deg <= 57.0:
                    blocked_general = True
                elif 123.0 <= angle_deg <= 147.0:
                    blocked_general = True
                elif 213.0 <= angle_deg <= 237.0:
                    blocked_general = True
            else:
                # Flipped Mapping Mode:
                # Operator stands in front of the robot (180°).
                if 123.0 <= angle_deg <= 237.0:
                    blocked_general = True
                elif 303.0 <= angle_deg <= 327.0:
                    blocked_general = True
                elif 33.0 <= angle_deg <= 57.0:
                    blocked_general = True
                
            # Chassis self-reflection and floor bump strike rejection:
            r = ranges[i]
            if not math.isnan(r) and (r < 0.22 or r > 12.0):
                blocked_general = True
                
            if blocked_general:
                ranges[i] = float('nan')

            # --- 2. Obstacle avoidance scan filtering (/scan_nav) ---
            # Restricted to ONLY the FRONT 120° FOV (120° to 240°, centered at 180°),
            # strictly excluding the structural chassis pillars and chassis reflections.
            blocked_nav = False
            if not flip:
                # Standard orientation (180° = direct robot front heading)
                # Front 120° arc: [120.0°, 240.0°]
                if angle_deg < min_front_deg or angle_deg > max_front_deg:
                    blocked_nav = True
                elif filter_pillars:
                    # Exclude Front-Left Pillar (centered at 135°: 123° to 147°)
                    if 123.0 <= angle_deg <= 147.0:
                        blocked_nav = True
                    # Exclude Front-Right Pillar (centered at 225°: 213° to 237°)
                    elif 213.0 <= angle_deg <= 237.0:
                        blocked_nav = True
            else:
                # Flipped orientation (0° / 360° = direct robot front heading)
                # Front 120° arc: [300.0°, 360.0°] U [0.0°, 60.0°]
                if half_fov < angle_deg < (360.0 - half_fov):
                    blocked_nav = True
                elif filter_pillars:
                    if (303.0 <= angle_deg <= 327.0) or (33.0 <= angle_deg <= 57.0):
                        blocked_nav = True

            nr = nav_ranges[i]
            if not math.isnan(nr) and (nr < 0.22 or nr > 12.0):
                blocked_nav = True

            if blocked_nav:
                nav_ranges[i] = float('nan')
                
        filtered_msg.ranges = ranges
        if len(msg.intensities) > 0:
            filtered_msg.intensities = list(msg.intensities)
        self.publisher.publish(filtered_msg)

        nav_msg.ranges = nav_ranges
        if len(msg.intensities) > 0:
            nav_msg.intensities = list(msg.intensities)
        self.nav_publisher.publish(nav_msg)

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
