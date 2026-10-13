#!/usr/bin/env python3
"""
Test local path plan publishing and /nav_markers visualization
Verifies that:
  1. /transformed_global_plan and /local_plan are actively populated.
  2. /nav_markers contains both the global plan (Marker 300) and the local trajectory (Marker 400, 401).
  3. /nav_markers contains the lookahead carrot target (Marker 120) and connector (Marker 121).
"""

import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from tf2_ros import StaticTransformBroadcaster

import sys
sys.path.insert(0, '/home/hospobot/hospobot_ws/src/hospobot/src/hospobot_perception/hospobot_perception')
from nav_visualizer_node import NavVisualizerNode


class VisualizerHarness(Node):
    def __init__(self):
        super().__init__('visualizer_harness')
        self.pub_plan = self.create_publisher(Path, '/plan', 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.received_local_plan = []
        self.received_transformed_plan = []
        self.received_markers = []

        self.sub_local = self.create_subscription(
            Path, '/local_plan', lambda msg: self.received_local_plan.append(msg), 10
        )
        self.sub_transformed = self.create_subscription(
            Path, '/transformed_global_plan', lambda msg: self.received_transformed_plan.append(msg), 10
        )
        self.sub_markers = self.create_subscription(
            MarkerArray, '/nav_markers', lambda msg: self.received_markers.append(msg), 10
        )

    def broadcast_pose(self, x=0.0, y=0.0, yaw=0.0):
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'map'
        ts.child_frame_id = 'base_footprint'
        ts.transform.translation.x = float(x)
        ts.transform.translation.y = float(y)
        ts.transform.translation.z = 0.0
        ts.transform.rotation.w = math.cos(yaw / 2.0)
        ts.transform.rotation.z = math.sin(yaw / 2.0)
        self.tf_broadcaster.sendTransform(ts)

    def publish_plan(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for i in range(40):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = i * 0.1
            ps.pose.position.y = 0.2 * math.sin(i * 0.2)
            path.poses.append(ps)
        self.pub_plan.publish(path)


def test_local_plan():
    print("[TEST] Initializing ROS 2 test harness for local path plan...")
    rclpy.init()

    vis_node = NavVisualizerNode()
    harness = VisualizerHarness()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(vis_node)
    executor.add_node(harness)

    harness.broadcast_pose(x=0.0, y=0.0, yaw=0.0)
    for _ in range(5):
        harness.publish_plan()
        end_t = time.time() + 0.1
        while time.time() < end_t and rclpy.ok():
            executor.spin_once(timeout_sec=0.02)

    # Let visualizer cycle run
    end_t = time.time() + 0.6
    while time.time() < end_t and rclpy.ok():
        harness.broadcast_pose(x=0.0, y=0.0, yaw=0.0)
        executor.spin_once(timeout_sec=0.05)

    print(f"Received /local_plan messages: {len(harness.received_local_plan)}")
    print(f"Received /transformed_global_plan messages: {len(harness.received_transformed_plan)}")
    print(f"Received /nav_markers messages: {len(harness.received_markers)}")

    assert len(harness.received_local_plan) > 0, "No /local_plan messages received!"
    assert len(harness.received_transformed_plan) > 0, "No /transformed_global_plan messages received!"

    latest_local = harness.received_local_plan[-1]
    latest_trans = harness.received_transformed_plan[-1]

    print(f"/local_plan poses count: {len(latest_local.poses)}")
    print(f"/transformed_global_plan poses count: {len(latest_trans.poses)}")
    assert len(latest_local.poses) > 5, "Local plan has insufficient poses!"
    assert len(latest_trans.poses) > 5, "Transformed global plan has insufficient poses!"

    assert len(harness.received_markers) > 0, "No /nav_markers received!"
    latest_markers = harness.received_markers[-1]
    marker_namespaces = {m.ns: m for m in latest_markers.markers}
    print(f"Marker namespaces published: {list(marker_namespaces.keys())}")

    assert 'global_path_line' in marker_namespaces, "Missing global_path_line marker!"
    assert 'local_path_line' in marker_namespaces, "Missing local_path_line marker!"
    assert 'local_path_waypoints' in marker_namespaces, "Missing local_path_waypoints marker!"
    assert 'lookahead_target_sphere' in marker_namespaces, "Missing lookahead_target_sphere marker!"
    assert 'lookahead_connector_line' in marker_namespaces, "Missing lookahead_connector_line marker!"

    local_line = marker_namespaces['local_path_line']
    print(f"Local path line points count: {len(local_line.points)}")
    assert len(local_line.points) > 5, "Local path line marker is empty!"

    vis_node.destroy_node()
    harness.destroy_node()
    rclpy.shutdown()
    print("\n=======================================================")
    print("LOCAL PATH PLAN & VISUALIZATION VERIFICATION PASSED!")
    print("=======================================================\n")


if __name__ == '__main__':
    test_local_plan()
