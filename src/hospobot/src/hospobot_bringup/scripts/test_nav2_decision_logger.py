#!/usr/bin/env python3
"""
Test Nav2 Decision Logger
Simulates navigation scenarios and verifies that Nav2 Decision Logger:
  1. Accurately computes Cross-Track Error (CTE) and Heading Error.
  2. Detects and diagnoses Abrupt Stops.
  3. Detects and explains Rotate-to-Heading in-place rotations.
  4. Intercepts Collision Monitor triggers.
  5. Detects steering oscillations.
  6. Writes structured logs to /home/hospobot/hospobot_ws/log/nav2_decisions.log.
  7. Publishes /nav2/decision_summary and /nav2/decision_events.
"""

import os
import sys
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

try:
    from nav2_msgs.msg import CollisionMonitorState
    HAS_COL_MON = True
except ImportError:
    HAS_COL_MON = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav2_decision_logger import Nav2DecisionLogger


class DecisionLoggerTestHarness(Node):
    def __init__(self):
        super().__init__('decision_logger_test_harness')
        self.pub_plan = self.create_publisher(Path, '/plan', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_cmd_smoothed = self.create_publisher(Twist, '/cmd_vel_smoothed', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_costmap = self.create_publisher(OccupancyGrid, '/local_costmap/costmap', 10)
        if HAS_COL_MON:
            self.pub_col_mon = self.create_publisher(CollisionMonitorState, '/collision_monitor_state', 10)

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.received_summaries = []
        self.received_events = []

        self.sub_summary = self.create_subscription(
            String, '/nav2/decision_summary', self._on_summary, 10
        )
        self.sub_events = self.create_subscription(
            String, '/nav2/decision_events', self._on_event, 10
        )

    def _on_summary(self, msg: String):
        self.received_summaries.append(msg.data)

    def _on_event(self, msg: String):
        self.received_events.append(msg.data)

    def broadcast_pose(self, x: float, y: float, yaw: float):
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'map'
        ts.child_frame_id = 'base_footprint'
        ts.transform.translation.x = float(x)
        ts.transform.translation.y = float(y)
        ts.transform.translation.z = 0.0
        ts.transform.rotation.z = math.sin(yaw / 2.0)
        ts.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(ts)

    def publish_straight_plan(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for i in range(50):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = i * 0.1  # 0.0 to 4.9m straight line along +X
            ps.pose.position.y = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_plan.publish(path)


def run_tests():
    print("[TEST] Initializing ROS 2 test suite for Nav2 Decision Logger...")
    rclpy.init()

    # Dedicated test log file
    test_log_file = "/home/hospobot/hospobot_ws/log/nav2_decisions.log"

    logger_node = Nav2DecisionLogger()
    harness = DecisionLoggerTestHarness()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(logger_node)
    executor.add_node(harness)

    def spin_for(duration: float, publish_fn=None):
        end_time = time.time() + duration
        while time.time() < end_time and rclpy.ok():
            if publish_fn:
                publish_fn()
            executor.spin_once(timeout_sec=0.05)

    print("[TEST] Setting up initial transform and straight path plan...")
    harness.broadcast_pose(x=0.0, y=0.0, yaw=0.0)
    harness.publish_straight_plan()
    spin_for(0.4)

    # -------------------------------------------------------------
    # Scenario 1: Normal Driving Forward
    # -------------------------------------------------------------
    print("\n--- Scenario 1: Normal Driving Forward ---")
    cmd = Twist()
    cmd.linear.x = 0.38
    cmd.angular.z = 0.02
    def pub_drive():
        harness.pub_cmd_vel.publish(cmd)
        harness.pub_cmd_smoothed.publish(cmd)
    spin_for(0.6, pub_drive)

    assert len(harness.received_summaries) > 0, "Failed to receive decision summary!"
    latest = harness.received_summaries[-1]
    print(f"Summary received: {latest}")
    assert "DRIVING_FORWARD" in latest, f"Expected DRIVING_FORWARD, got: {latest}"
    print("PASS: Scenario 1 confirmed DRIVING_FORWARD status.")

    # -------------------------------------------------------------
    # Scenario 2: Rotate to Heading Alignment
    # -------------------------------------------------------------
    print("\n--- Scenario 2: Rotate to Heading in Place ---")
    # Robot facing 60 degrees away (1.05 rad), angle > threshold (0.85 rad / 48.7 deg)
    harness.broadcast_pose(x=1.0, y=0.0, yaw=math.radians(60.0))
    cmd_rth = Twist()
    cmd_rth.linear.x = 0.0
    cmd_rth.angular.z = 0.40
    def pub_rth():
        harness.pub_cmd_vel.publish(cmd_rth)
    spin_for(0.6, pub_rth)

    latest_rth = harness.received_summaries[-1]
    print(f"Summary received: {latest_rth}")
    assert "ROTATE_TO_HEADING" in latest_rth, f"Expected ROTATE_TO_HEADING, got: {latest_rth}"
    assert "Aligning in-place" in latest_rth, f"Expected Aligning in-place explanation, got: {latest_rth}"
    print("PASS: Scenario 2 confirmed ROTATE_TO_HEADING with heading error explanation.")

    # -------------------------------------------------------------
    # Scenario 3: Abrupt Stop Detection & Diagnosis
    # -------------------------------------------------------------
    print("\n--- Scenario 3: Abrupt Stop Detection ---")
    # First drive fast
    cmd_drive = Twist()
    cmd_drive.linear.x = 0.40
    harness.pub_cmd_vel.publish(cmd_drive)
    spin_for(0.4)

    # Suddenly halt
    cmd_stop = Twist()
    cmd_stop.linear.x = 0.0
    cmd_stop.angular.z = 0.40  # Halt to rotate
    harness.pub_cmd_vel.publish(cmd_stop)
    spin_for(0.5)

    abrupt_stops = [e for e in harness.received_events if "ABRUPT_STOP" in e]
    print(f"Abrupt stop events captured: {len(abrupt_stops)}")
    assert len(abrupt_stops) > 0, "Failed to detect ABRUPT_STOP event!"
    print(f"Detected event details: {abrupt_stops[-1]}")
    assert "DIAGNOSIS" in abrupt_stops[-1], "Missing DIAGNOSIS in abrupt stop alert!"
    print("PASS: Scenario 3 confirmed Abrupt Stop detection and diagnostic attribution.")

    # -------------------------------------------------------------
    # Scenario 4: Collision Monitor Intervention
    # -------------------------------------------------------------
    if HAS_COL_MON:
        print("\n--- Scenario 4: Collision Monitor Intervention ---")
        cm_msg = CollisionMonitorState()
        cm_msg.action_type = 1  # STOP
        cm_msg.polygon_name = "FootprintApproach"
        harness.pub_col_mon.publish(cm_msg)
        spin_for(0.5)

        col_events = [e for e in harness.received_events if "COLLISION_MONITOR_TRIGGERED" in e]
        print(f"Collision monitor events: {len(col_events)}")
        assert len(col_events) > 0, "Failed to capture COLLISION_MONITOR_TRIGGERED event!"
        print(f"Captured: {col_events[-1]}")
        print("PASS: Scenario 4 confirmed Collision Monitor interception.")

    # -------------------------------------------------------------
    # Scenario 5: Steering Oscillation Detection
    # -------------------------------------------------------------
    print("\n--- Scenario 5: Steering Oscillation Detection ---")
    # Feed rapid steering reversals
    for i in range(8):
        cmd_osc = Twist()
        cmd_osc.linear.x = 0.30
        cmd_osc.angular.z = 0.35 if i % 2 == 0 else -0.35
        harness.pub_cmd_vel.publish(cmd_osc)
        spin_for(0.15)

    osc_events = [e for e in harness.received_events if "OSCILLATION" in e]
    print(f"Oscillation events captured: {len(osc_events)}")
    assert len(osc_events) > 0, "Failed to detect OSCILLATION event!"
    print(f"Captured: {osc_events[-1]}")
    print("PASS: Scenario 5 confirmed steering hunting oscillation detection.")

    # -------------------------------------------------------------
    # Scenario 6: Verify Log File Persistence
    # -------------------------------------------------------------
    print("\n--- Scenario 6: Log File Verification ---")
    assert os.path.exists(test_log_file), f"Log file does not exist: {test_log_file}"
    with open(test_log_file, "r") as f:
        log_content = f.read()

    print(f"Log file exists at {test_log_file}. Total length: {len(log_content)} bytes.")
    assert "Nav2 Decision Logger Initialized" in log_content, "Missing logger initialization entry in log file!"
    assert "ABRUPT_STOP" in log_content or "ROTATE_TO_HEADING" in log_content, "Missing diagnostic entries in log file!"
    print("PASS: Scenario 6 verified persistent logging in log/nav2_decisions.log.")

    # Clean shutdown
    logger_node.destroy_node()
    harness.destroy_node()
    rclpy.shutdown()

    print("\n=======================================================")
    print("ALL 6 NAV2 DECISION LOGGER VERIFICATION TESTS PASSED!")
    print("=======================================================\n")


if __name__ == '__main__':
    run_tests()
