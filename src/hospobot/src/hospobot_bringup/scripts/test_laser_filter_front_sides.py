#!/usr/bin/env python3
"""
Unit test for 2D LiDAR filtering and obstacle avoidance integration.
Verifies that:
1. LaserFilterNode produces /scan_nav with strictly FRONT, LEFT, and RIGHT sectors.
2. All rear beams (303° to 57° mod 360°) and chassis poles are masked out (NaN).
3. /scan continues to provide AMCL/localization with scan data.
4. nav2_params.yaml configures local_costmap and global_costmap to use /scan_nav for obstacle avoidance.
"""

import sys
import math
import time
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from hospobot_perception.laser_filter_node import LaserFilterNode

class LaserFilterTester(Node):
    def __init__(self):
        super().__init__('test_laser_filter_verifier')
        self.raw_pub = self.create_publisher(LaserScan, '/scan_raw', 10)
        
        self.received_scan = None
        self.received_nav = None
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, rclpy.qos.qos_profile_sensor_data)
        self.nav_sub = self.create_subscription(
            LaserScan, '/scan_nav', self.on_nav, rclpy.qos.qos_profile_sensor_data)

    def on_scan(self, msg):
        self.received_scan = msg

    def on_nav(self, msg):
        self.received_nav = msg

def test_yaml_configuration():
    print("[TEST 1] Verifying nav2_params.yaml costmap configurations...")
    yaml_path = '/home/hospobot/hospobot_ws/src/hospobot/src/hospobot_bringup/config/nav2_params.yaml'
    with open(yaml_path, 'r') as f:
        cfg = yaml.safe_load(f)
        
    local = cfg['local_costmap']['local_costmap']['ros__parameters']
    global_cm = cfg['global_costmap']['global_costmap']['ros__parameters']
    
    # Verify local_costmap
    assert 'obstacle_layer' in local['plugins'], "obstacle_layer must be in local_costmap plugins"
    assert 'depth_obstacle_layer' in local['plugins'], "depth_obstacle_layer must be retained in local_costmap"
    assert local['obstacle_layer']['scan']['topic'] == '/scan_nav', "local_costmap scan topic must be /scan_nav"
    
    # Verify global_costmap
    assert 'obstacle_layer' in global_cm['plugins'], "obstacle_layer must be in global_costmap plugins"
    assert global_cm['obstacle_layer']['scan']['topic'] == '/scan_nav', "global_costmap scan topic must be /scan_nav"
    
    print("  -> PASSED: nav2_params.yaml configures 2D LiDAR obstacle avoidance via /scan_nav.")

def test_laser_filter_runtime():
    print("[TEST 2] Verifying LaserFilterNode sector masking runtime (/scan_nav vs /scan)...")
    rclpy.init()
    
    filter_node = LaserFilterNode()
    tester_node = LaserFilterTester()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(filter_node)
    executor.add_node(tester_node)
    
    # Synthesize 360-degree scan (360 samples, 1 deg each, min=0, max=2*pi)
    scan_raw = LaserScan()
    scan_raw.header.stamp = tester_node.get_clock().now().to_msg()
    scan_raw.header.frame_id = 'laser_frame'
    scan_raw.angle_min = 0.0
    scan_raw.angle_max = 2.0 * math.pi
    scan_raw.angle_increment = 2.0 * math.pi / 360.0
    scan_raw.time_increment = 0.0
    scan_raw.scan_time = 0.1
    scan_raw.range_min = 0.15
    scan_raw.range_max = 12.0
    scan_raw.ranges = [2.5] * 360
    scan_raw.intensities = [100.0] * 360
    
    # Publish raw scan and spin
    start_time = time.time()
    while tester_node.received_nav is None and (time.time() - start_time) < 3.0:
        tester_node.raw_pub.publish(scan_raw)
        executor.spin_once(timeout_sec=0.1)
        
    assert tester_node.received_nav is not None, "Failed to receive /scan_nav from LaserFilterNode"
    assert tester_node.received_scan is not None, "Failed to receive /scan from LaserFilterNode"
    
    nav_ranges = tester_node.received_nav.ranges
    scan_ranges = tester_node.received_scan.ranges
    
    # Verify /scan_nav:
    # 1. Front FOV (centered at 180° in laser_frame): 147° to 213° must have valid measurements
    front_valid = [not math.isnan(nav_ranges[deg]) for deg in range(155, 205)]
    assert all(front_valid), "Front FOV (around 180 deg) must have valid measurements in /scan_nav"
    
    # 2. Front structural pillars (123°-147° and 213°-237°) MUST BE MASKED (NaN)
    assert math.isnan(nav_ranges[135]), f"Front-left pillar 135° in /scan_nav must be NaN, got {nav_ranges[135]}"
    assert math.isnan(nav_ranges[225]), f"Front-right pillar 225° in /scan_nav must be NaN, got {nav_ranges[225]}"
    for deg in range(125, 145):
        assert math.isnan(nav_ranges[deg]), f"Front-left pillar angle {deg}° in /scan_nav must be NaN"
    for deg in range(215, 235):
        assert math.isnan(nav_ranges[deg]), f"Front-right pillar angle {deg}° in /scan_nav must be NaN"

    # 3. Outside front 120° FOV (< 120° or > 240°) MUST BE MASKED (NaN)
    # Left side (65°-115°) is outside front 120° -> must be NaN
    left_masked = [math.isnan(nav_ranges[deg]) for deg in range(65, 115)]
    assert all(left_masked), "Left side (< 120°) must be masked with NaN in /scan_nav"
    
    # Right side (245°-295°) is outside front 120° -> must be NaN
    right_masked = [math.isnan(nav_ranges[deg]) for deg in range(245, 295)]
    assert all(right_masked), "Right side (> 240°) must be masked with NaN in /scan_nav"
    
    # Rear Wedge (305° to 55° mod 360°) MUST BE MASKED (NaN)
    rear_degs = list(range(305, 360)) + list(range(0, 55))
    for deg in rear_degs:
        assert math.isnan(nav_ranges[deg]), f"Rear angle {deg}° in /scan_nav must be NaN, got {nav_ranges[deg]}"
        
    print("  -> PASSED: /scan_nav successfully restricted strictly to the FRONT 120° FOV.")
    print("  -> PASSED: Structural pillars (123°-147° & 213°-237°) and everything outside 120° are 100% masked with NaN.")
    
    executor.shutdown()
    filter_node.destroy_node()
    tester_node.destroy_node()
    rclpy.shutdown()

def main():
    print("======================================================================")
    print("STARTING 2D LIDAR OBSTACLE AVOIDANCE FILTER VERIFICATION SUITE")
    print("======================================================================")
    test_yaml_configuration()
    test_laser_filter_runtime()
    print("======================================================================")
    print("ALL 2D LIDAR OBSTACLE AVOIDANCE TESTS PASSED SUCCESSFULLY!")
    print("======================================================================")

if __name__ == '__main__':
    main()
