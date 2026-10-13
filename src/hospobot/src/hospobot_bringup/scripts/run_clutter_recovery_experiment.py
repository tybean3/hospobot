#!/usr/bin/env python3
"""
run_clutter_recovery_experiment.py

Automated runner for the Clutter & Recovery experiment.
Reads configuration from saved_experiment_pose.json, sets the initial pose estimate
on AMCL, waits for localization convergence, and dispatches the goal pose (39.0, -38.0).
"""

import sys
import os
import json
import time
import math
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_srvs.srv import Empty as EmptySrv

DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(__file__), "..", "config", "saved_experiment_pose.json"
)

class ClutterRecoveryExperimentRunner(Node):
    def __init__(self, config_path):
        super().__init__('clutter_recovery_experiment_runner')
        
        # Load config
        self.config_path = os.path.abspath(config_path)
        self.load_config()

        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.get_logger().info(f"Loaded experiment configuration from: {self.config_path}")
        self.get_logger().info(f"Map: {self.map_name}")
        self.get_logger().info(f"Initial Pose: ({self.init_x:.3f}, {self.init_y:.3f}, yaw={self.init_yaw:.3f} rad)")
        self.get_logger().info(f"Goal Pose: ({self.goal_x:.3f}, {self.goal_y:.3f}, yaw={self.goal_yaw:.3f} rad)")

    def load_config(self):
        self.map_name = "Building6-Floor1"
        self.init_x = 33.226
        self.init_y = -30.233
        self.init_yaw = -0.641
        self.goal_x = 39.0
        self.goal_y = -38.0
        self.goal_yaw = -0.641

        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, 'r') as f:
                    data = json.load(f)
                self.map_name = data.get("map", self.map_name)
                ip = data.get("initial_pose", {})
                self.init_x = float(ip.get("x", self.init_x))
                self.init_y = float(ip.get("y", self.init_y))
                self.init_yaw = float(ip.get("yaw_rad", self.init_yaw))

                exp = data.get("experiment_clutter_and_recovery", {})
                gp = exp.get("goal_pose", {})
                self.goal_x = float(gp.get("x", self.goal_x))
                self.goal_y = float(gp.get("y", self.goal_y))
                self.goal_yaw = float(gp.get("yaw_rad", self.goal_yaw))
            except Exception as e:
                self.get_logger().error(f"Error parsing json config: {e}")

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(self.init_x)
        msg.pose.pose.position.y = float(self.init_y)
        msg.pose.pose.position.z = 0.0

        half_yaw = float(self.init_yaw) * 0.5
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        cov = [0.0] * 36
        cov[0] = 0.08
        cov[7] = 0.08
        cov[35] = 0.04
        msg.pose.covariance = cov

        self.initialpose_pub.publish(msg)
        self.get_logger().info(f"Published initial pose estimate: ({self.init_x:.3f}, {self.init_y:.3f}, yaw={self.init_yaw:.3f} rad)")

        # Request nomotion updates to converge particle filter
        try:
            client = self.create_client(EmptySrv, '/request_nomotion_update')
            if client.wait_for_service(timeout_sec=0.5):
                for _ in range(3):
                    req = EmptySrv.Request()
                    client.call_async(req)
                    time.sleep(0.05)
                self.get_logger().info("Triggered AMCL nomotion particle filter convergence updates.")
        except Exception as e:
            self.get_logger().warn(f"Nomotion update request skipped: {e}")

    def publish_goal_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(self.goal_x)
        msg.pose.position.y = float(self.goal_y)
        msg.pose.position.z = 0.0

        half_yaw = float(self.goal_yaw) * 0.5
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)

        self.goal_pub.publish(msg)
        self.get_logger().info(f"Published navigation goal pose: ({self.goal_x:.3f}, {self.goal_y:.3f}, yaw={self.goal_yaw:.3f} rad)")

def main():
    parser = argparse.ArgumentParser(description="Run Clutter & Recovery Navigation Experiment")
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH, help="Path to saved_experiment_pose.json")
    parser.add_argument("--skip-wait", action="store_true", help="Skip startup wait delay")
    args = parser.parse_args()

    rclpy.init()
    node = ClutterRecoveryExperimentRunner(args.config)

    print("\n=======================================================")
    print("   HOSPOBOT EXPERIMENT RUNNER: Clutter & Recovery     ")
    print("=======================================================")
    print(f"Map:          {node.map_name}")
    print(f"Initial Pose: ({node.init_x:.3f}, {node.init_y:.3f}, yaw={node.init_yaw:.3f} rad)")
    print(f"Goal Pose:    ({node.goal_x:.3f}, {node.goal_y:.3f}, yaw={node.goal_yaw:.3f} rad)")
    print("=======================================================\n")

    if not args.skip_wait:
        print("[1/3] Waiting 3 seconds for Nav2 & AMCL stack stabilization...")
        time.sleep(3.0)

    print("[2/3] Setting Initial Pose Estimate...")
    node.publish_initial_pose()
    time.sleep(1.0)
    node.publish_initial_pose()

    print("[3/3] Waiting 10 seconds for map and initial pose to start up properly...")
    for remaining in range(10, 0, -1):
        print(f"       Dispatching goal in {remaining}s...   ", end="\r", flush=True)
        time.sleep(1.0)
    print("\n[✓] Dispatching Goal Pose (39.0, -38.0)...")
    node.publish_goal_pose()
    time.sleep(1.0)
    node.publish_goal_pose()

    print("\nExperiment dispatched successfully! Robot is navigating to (39.0, -38.0).\n")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
