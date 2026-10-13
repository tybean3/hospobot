#!/usr/bin/env python3
"""
Comprehensive Test Suite for Navigation Recovery Node (Cloud AI & Multi-Modal Sensor Fusion).
Tests:
1. Node initialization, parameters, and sensor subscription wiring.
2. Multi-modal sensor ingestion and clearance profiling.
3. Abort detection via /navigate_to_pose/_action/status (STATUS_ABORTED).
4. Cloud AI / Intelligent Reasoner scene diagnosis & escape maneuver synthesis.
5. Safe physical extrication maneuver execution (/cmd_vel) & costmap flushing.
6. Automatic mission resumption (re-dispatching target goal to /goal_pose).
7. Observability (status JSON, RViz billboard & arrow markers, persistent logging).
"""

import os
import sys
import time
import math
import json
import rclpy
from rclpy.node import Node

from action_msgs.msg import GoalStatusArray, GoalStatus, GoalInfo
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_msgs.srv import ClearEntireCostmap

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from navigation_recovery_node import NavigationRecoveryNode
from hospobot_interfaces.msg import SemanticObjectArray, SemanticObject


class RecoveryTestHarness(Node):
    def __init__(self):
        super().__init__('recovery_test_harness')
        
        # Publishers to simulate robot sensors & Nav2 action server
        self.pub_status = self.create_publisher(GoalStatusArray, '/navigate_to_pose/_action/status', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_scan_nav = self.create_publisher(LaserScan, '/scan_nav', 10)
        self.pub_scan_raw = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_image = self.create_publisher(Image, '/oak/rgb/image_raw', 10)
        self.pub_semantics = self.create_publisher(SemanticObjectArray, '/oakd/semantic_objects', 10)

        # Service servers for costmap clearing mocks
        self.srv_clear_global = self.create_service(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap', self._on_clear_global)
        self.srv_clear_local = self.create_service(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap', self._on_clear_local)

        # Subscribers to capture recovery outputs
        self.received_cmd_vels = []
        self.received_resumed_goals = []
        self.received_status_msgs = []
        self.received_markers = []
        self.global_costmap_cleared = False
        self.local_costmap_cleared = False

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal_pose, 10)
        self.create_subscription(String, '/navigation_recovery/status', self._on_status, 10)
        self.create_subscription(MarkerArray, '/navigation_recovery/markers', self._on_markers, 10)

    def _on_clear_global(self, req, res):
        self.global_costmap_cleared = True
        return res

    def _on_clear_local(self, req, res):
        self.local_costmap_cleared = True
        return res

    def _on_cmd_vel(self, msg: Twist):
        self.received_cmd_vels.append((time.time(), msg.linear.x, msg.angular.z))

    def _on_goal_pose(self, msg: PoseStamped):
        self.received_resumed_goals.append(msg)

    def _on_status(self, msg: String):
        try:
            self.received_status_msgs.append(json.loads(msg.data))
        except Exception:
            pass

    def _on_markers(self, msg: MarkerArray):
        self.received_markers.append(msg)

    def send_simulated_sensors(self, front_dist=0.45, left_dist=0.70, right_dist=1.85, rear_dist=2.5, has_bed=True):
        now = self.get_clock().now().to_msg()

        # 1. Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = 2.0
        odom.pose.pose.position.y = 0.5
        odom.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(odom)

        # 2. 2D LiDAR /scan_nav
        scan_nav = LaserScan()
        scan_nav.header.stamp = now
        scan_nav.header.frame_id = 'laser_frame'
        scan_nav.angle_min = 0.0
        scan_nav.angle_max = 2.0 * math.pi
        scan_nav.angle_increment = 2.0 * math.pi / 360.0
        scan_nav.range_min = 0.15
        scan_nav.range_max = 12.0
        ranges = [float('nan')] * 360
        # Left (60°-120°): left_dist
        for deg in range(60, 121):
            ranges[deg] = left_dist
        # Front (150°-210°): front_dist
        for deg in range(150, 211):
            ranges[deg] = front_dist
        # Right (240°-300°): right_dist
        for deg in range(240, 301):
            ranges[deg] = right_dist
        scan_nav.ranges = ranges
        self.pub_scan_nav.publish(scan_nav)

        # 3. 2D LiDAR raw /scan (includes rear)
        scan_raw = LaserScan()
        scan_raw.header.stamp = now
        scan_raw.header.frame_id = 'laser_frame'
        scan_raw.angle_min = 0.0
        scan_raw.angle_max = 2.0 * math.pi
        scan_raw.angle_increment = 2.0 * math.pi / 360.0
        scan_raw.range_min = 0.15
        scan_raw.range_max = 12.0
        raw_ranges = list(ranges)
        # Rear (330°-30°)
        for deg in list(range(330, 360)) + list(range(0, 31)):
            raw_ranges[deg] = rear_dist
        scan_raw.ranges = raw_ranges
        self.pub_scan_raw.publish(scan_raw)

        # 4. Camera Image (RGB)
        img = Image()
        img.header.stamp = now
        img.header.frame_id = 'oakd_rgb_camera_optical_frame'
        img.height = 120
        img.width = 160
        img.encoding = 'rgb8'
        img.step = 160 * 3
        # Right corridor is bright/open, left is darker/blocked
        pixel_row = bytearray()
        for col in range(160):
            val = 220 if col > 90 else 50
            pixel_row.extend([val, val, val])
        img.data = bytes(pixel_row * 120)
        self.pub_image.publish(img)

        # 5. Semantic detections
        sem_array = SemanticObjectArray()
        sem_array.header.stamp = now
        if has_bed:
            bed = SemanticObject()
            bed.class_name = 'bed'
            bed.position.x = 0.5
            bed.position.y = 0.1
            bed.confidence = 0.92
            sem_array.objects.append(bed)
        self.pub_semantics.publish(sem_array)


def test_recovery_lifecycle():
    print("======================================================================")
    print("STARTING NAVIGATION RECOVERY NODE VERIFICATION TEST SUITE")
    print("======================================================================")
    rclpy.init()

    recovery_node = NavigationRecoveryNode()
    harness = RecoveryTestHarness()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(recovery_node)
    executor.add_node(harness)

    # ---------------------------------------------------------
    # TEST 1: Mission Goal Caching
    # ---------------------------------------------------------
    print("[TEST 1] Setting target mission goal...")
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = harness.get_clock().now().to_msg()
    goal.pose.position.x = 8.5
    goal.pose.position.y = 1.2
    goal.pose.orientation.w = 1.0
    harness.pub_goal.publish(goal)

    for _ in range(10):
        executor.spin_once(timeout_sec=0.05)

    assert recovery_node.target_goal_pose is not None, "Target goal was not cached by recovery node"
    assert recovery_node.target_goal_pose.pose.position.x == 8.5
    assert recovery_node.state == "IDLE"
    print("  -> PASSED: Mission goal successfully cached in recovery node.")

    # ---------------------------------------------------------
    # TEST 2: Multi-Modal Sensor Ingestion & Clearance Profiling
    # ---------------------------------------------------------
    print("[TEST 2] Publishing sensor streams (Front blocked at 0.45m by Bed, Right flank open at 1.85m)...")
    harness.send_simulated_sensors(front_dist=0.45, left_dist=0.70, right_dist=1.85, rear_dist=2.5, has_bed=True)

    for _ in range(15):
        executor.spin_once(timeout_sec=0.05)

    clearances = recovery_node._profile_sensor_clearances()
    assert abs(clearances['front_min'] - 0.45) < 0.05, f"Front clearance error: {clearances['front_min']}"
    assert abs(clearances['right_min'] - 1.85) < 0.05, f"Right clearance error: {clearances['right_min']}"
    assert len(recovery_node.latest_semantics) >= 1, "Semantic bed detection not received"
    print(f"  -> Sensor clearances verified: Front={clearances['front_min']:.2f}m, Right={clearances['right_min']:.2f}m, Semantics={[s['class'] for s in recovery_node.latest_semantics]}")
    print("  -> PASSED: Multi-modal sensors ingested and profiled accurately.")

    # ---------------------------------------------------------
    # TEST 3: Abort Detection & AI Diagnosis Synthesis
    # ---------------------------------------------------------
    print("[TEST 3] Simulating Nav2 STATUS_ABORTED event on /navigate_to_pose/_action/status...")
    status_array = GoalStatusArray()
    status_item = GoalStatus()
    status_item.status = GoalStatus.STATUS_ABORTED
    info = GoalInfo()
    info.goal_id.uuid = [1] * 16
    status_item.goal_info = info
    status_array.status_list.append(status_item)
    harness.pub_status.publish(status_array)

    # Spin to allow abort detection and AI diagnosis
    start_t = time.time()
    while recovery_node.state in ["IDLE", "ABORT_DETECTED"] and (time.time() - start_t) < 2.0:
        executor.spin_once(timeout_sec=0.05)

    assert recovery_node.state in ["AI_DIAGNOSING", "EXECUTING_ESCAPE"], f"Expected active recovery, got: {recovery_node.state}"
    assert recovery_node.recovery_attempt_count == 1
    print(f"  -> State transitioned to: {recovery_node.state}")

    # Spin further for AI diagnosis execution
    for _ in range(10):
        executor.spin_once(timeout_sec=0.05)

    assert "PIVOT_RIGHT" in recovery_node.last_escape_maneuver, f"Expected PIVOT_RIGHT, got: {recovery_node.last_escape_maneuver}"
    assert "Hospital Bed" in recovery_node.last_diagnosis or "Obstacle" in recovery_node.last_diagnosis
    print(f"  -> AI Diagnosis: '{recovery_node.last_diagnosis}'")
    print(f"  -> Synthesized Maneuver: '{recovery_node.last_escape_maneuver}'")
    print("  -> PASSED: AI accurately diagnosed bed blockage and synthesized safe rightward pivot without blind reverse.")

    # ---------------------------------------------------------
    # TEST 4: Physical Escape Execution & Costmap Flushing
    # ---------------------------------------------------------
    print("[TEST 4] Validating safe physical extrication maneuver execution...")
    # Step through escape execution
    start_esc = time.time()
    while len(harness.received_cmd_vels) < 3 and (time.time() - start_esc) < 2.5:
        executor.spin_once(timeout_sec=0.05)

    assert harness.global_costmap_cleared, "global_costmap clear service must be invoked"
    assert harness.local_costmap_cleared, "local_costmap clear service must be invoked"
    assert len(harness.received_cmd_vels) > 0, "No velocity commands were published to /cmd_vel"

    # Verify pivot commands were issued (rotates in place so camera faces corridor) and NO blind reverse
    pivot_cmds = [v for v in harness.received_cmd_vels if abs(v[2]) > 0.05]
    assert len(pivot_cmds) > 0, "No pivot command issued during escape"
    reverse_cmds = [v for v in harness.received_cmd_vels if v[1] < -0.01]
    assert len(reverse_cmds) == 0, f"Dangerous blind reverse detected: {reverse_cmds}"
    print(f"  -> Pivot rotation commands verified: {len(pivot_cmds)} commands (Wz ≈ {pivot_cmds[0][2]:.2f} rad/s, zero blind reverse)")
    print("  -> PASSED: Physical escape trajectory executed safely and costmaps successfully flushed.")

    # ---------------------------------------------------------
    # TEST 5: Automatic Mission Goal Re-Dispatch
    # ---------------------------------------------------------
    print("[TEST 5] Checking automatic mission goal re-dispatch...")
    # Accelerate escape timer to complete maneuver
    recovery_node.escape_start_time = time.time() - 10.0
    start_resume = time.time()
    while recovery_node.state != "RESUMING_NAV" and (time.time() - start_resume) < 3.0:
        executor.spin_once(timeout_sec=0.05)

    assert recovery_node.state == "RESUMING_NAV", f"Expected RESUMING_NAV, got: {recovery_node.state}"
    assert len(harness.received_resumed_goals) > 0, "Original mission goal was not re-published to /goal_pose"
    resumed = harness.received_resumed_goals[-1]
    assert resumed.pose.position.x == 8.5
    assert resumed.pose.position.y == 1.2
    print(f"  -> Re-dispatched goal coordinates: ({resumed.pose.position.x}, {resumed.pose.position.y})")
    print("  -> PASSED: Robot automatically re-dispatched mission goal rather than stopping!")

    # ---------------------------------------------------------
    # TEST 6: Observability (Status JSON, RViz 3D Markers & Logging)
    # ---------------------------------------------------------
    print("[TEST 6] Validating observability (Status JSON, 3D Markers, Log file)...")
    assert len(harness.received_status_msgs) > 0, "No status JSON published"
    latest_st = harness.received_status_msgs[-1]
    assert latest_st['attempt_count'] == 1
    assert latest_st['target_goal_cached'] is True

    assert len(harness.received_markers) > 0, "No RViz 3D markers published"
    markers = harness.received_markers[-1].markers
    assert len(markers) >= 1
    assert any(m.ns == 'recovery_hud' for m in markers), "AI Billboard marker missing"

    log_path = recovery_node.log_file_path
    assert os.path.exists(log_path), f"Log file {log_path} does not exist"
    with open(log_path, 'r') as f:
        log_content = f.read()
    assert "[RECOVERY_INITIATED]" in log_content
    assert "[AI_RECOVERY_DECISION]" in log_content
    assert "[RESUME_GOAL]" in log_content
    print("  -> Status JSON and RViz 3D markers verified.")
    print(f"  -> Persistent recovery log confirmed at: {log_path}")
    print("  -> PASSED: All observability outputs functional.")

    executor.shutdown()
    recovery_node.destroy_node()
    harness.destroy_node()
    rclpy.shutdown()

    print("======================================================================")
    print("ALL 6/6 NAVIGATION RECOVERY TESTS PASSED SUCCESSFULLY!")
    print("======================================================================")


if __name__ == '__main__':
    test_recovery_lifecycle()
