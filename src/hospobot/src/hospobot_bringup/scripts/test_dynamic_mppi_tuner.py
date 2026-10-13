#!/usr/bin/env python3
"""
Test Suite: Dynamic Intelligent MPPI Scene Tuner Verification
Verifies behavioral logic:
  1. Clear scene -> Nominal delivery speeds (vx_max = 0.40 m/s, cost_weight = 3.81).
  2. Crowded state (>= 2 people) -> Reduced max velocity (0.16 m/s), increased obstacle cost weight (8.5).
  3. Heavy equipment (e.g., bed) -> Reduced max velocity (0.18 m/s), increased obstacle cost weight (8.0).
  4. Combined congestion (>= 2 people + bed) -> Critical caution crawl (vx_max = 0.12 m/s, cost_weight = 10.0).
  5. Object clearing hysteresis -> Verifies 0.8s hold before reverting to nominal speed.
  6. Parameter dispatch -> Validates SetParameters requests for FollowPath.vx_max, FollowPath.max_vel_x,
     FollowPath.CostCritic.cost_weight, and FollowPath.ObstaclesCost.cost_weight.
"""

import os
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import SetParametersResult

from hospobot_interfaces.msg import SemanticObjectArray, SemanticObject

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dynamic_mppi_tuner import DynamicMppiTuner


class MockServerNode(Node):
    """Mocks controller_server and velocity_smoother SetParameters services."""
    def __init__(self):
        super().__init__('mock_server_node')
        self.received_controller_params = {}
        self.received_smoother_params = {}

        self.srv_controller = self.create_service(
            SetParameters,
            '/controller_server/set_parameters',
            self._cb_controller_set_params
        )
        self.srv_smoother = self.create_service(
            SetParameters,
            '/velocity_smoother/set_parameters',
            self._cb_smoother_set_params
        )

    def _cb_controller_set_params(self, request, response):
        for p in request.parameters:
            self.received_controller_params[p.name] = p.value.double_value
            response.results.append(SetParametersResult(successful=True, reason="OK"))
        return response

    def _cb_smoother_set_params(self, request, response):
        for p in request.parameters:
            self.received_smoother_params[p.name] = list(p.value.double_array_value)
            response.results.append(SetParametersResult(successful=True, reason="OK"))
        return response


class TestHarness:
    def __init__(self):
        rclpy.init()
        self.mock_node = MockServerNode()
        self.tuner_node = DynamicMppiTuner()

        # Disable TF lookup dependency during unit testing (robot at 0,0,0)
        self.tuner_node.robot_x = 0.0
        self.tuner_node.robot_y = 0.0
        self.tuner_node.robot_yaw = 0.0
        self.tuner_node.has_pose = True
        self.tuner_node._update_robot_pose = lambda: None  # mock pose update

        # Publisher for semantic objects
        self.obj_pub = self.mock_node.create_publisher(
            SemanticObjectArray,
            '/semantic/tracked_objects',
            10
        )

        # State monitoring
        self.tuning_states = []
        self.events = []
        self.mock_node.create_subscription(
            String,
            '/mppi/scene_tuning_state',
            lambda msg: self.tuning_states.append(msg.data),
            10
        )
        self.mock_node.create_subscription(
            String,
            '/nav2/decision_events',
            lambda msg: self.events.append(msg.data),
            10
        )

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.mock_node)
        self.executor.add_node(self.tuner_node)

    def spin_duration(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            self.executor.spin_once(timeout_sec=0.05)

    def publish_objects(self, obj_list, frame_id='map'):
        msg = SemanticObjectArray()
        msg.header.stamp = self.mock_node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        for idx, (cname, x, y, vx, vy) in enumerate(obj_list):
            obj = SemanticObject()
            obj.class_name = cname
            obj.tracking_id = idx + 1
            obj.position = Point(x=float(x), y=float(y), z=0.5)
            obj.velocity = Vector3(x=float(vx), y=float(vy), z=0.0)
            obj.confidence = 0.95
            msg.objects.append(obj)
        self.obj_pub.publish(msg)

    def shutdown(self):
        self.mock_node.destroy_node()
        self.tuner_node.destroy_node()
        rclpy.shutdown()


def run_tests():
    print("=" * 70)
    print("STARTING DYNAMIC INTELLIGENT MPPI TUNER TEST SUITE")
    print("=" * 70)

    harness = TestHarness()
    passes = 0
    total = 6

    try:
        # TEST 1: Initial Clear State
        print("\n[TEST 1] Initial Clear Corridor State...")
        harness.spin_duration(0.5)
        mode = harness.tuner_node.current_mode
        t_vx = harness.tuner_node.target_vx_max
        t_cost = harness.tuner_node.target_cost_weight
        print(f"  Mode: {mode} | Target Vx: {t_vx:.2f} m/s | Target Cost: {t_cost:.2f}")
        assert mode == "CLEAR_DELIVERY", f"Expected CLEAR_DELIVERY, got {mode}"
        assert abs(t_vx - 0.40) < 1e-3, f"Expected 0.40, got {t_vx}"
        assert abs(t_cost - 3.81) < 1e-3, f"Expected 3.81, got {t_cost}"
        passes += 1
        print("  --> PASS: Clear corridor defaults to nominal delivery velocity.")

        # TEST 2: Crowded State (>= 2 people)
        print("\n[TEST 2] Crowded State Detection (3 people in corridor)...")
        for _ in range(8):
            harness.publish_objects([
                ('person', 1.5, 0.3, -0.2, 0.0),
                ('person', 2.0, -0.4, 0.0, 0.0),
                ('human', 2.5, 0.1, -0.1, 0.0)
            ])
            harness.spin_duration(0.1)

        mode = harness.tuner_node.current_mode
        t_vx = harness.tuner_node.target_vx_max
        t_cost = harness.tuner_node.target_cost_weight
        active_vx = harness.tuner_node.active_vx_max
        print(f"  Mode: {mode} | Target Vx: {t_vx:.2f} m/s | Target Cost: {t_cost:.2f} | Active Vx: {active_vx:.2f}")
        assert mode == "CROWDED_STATE", f"Expected CROWDED_STATE, got {mode}"
        assert t_vx <= 0.18, f"Expected vx <= 0.18, got {t_vx}"
        assert t_cost >= 8.0, f"Expected cost_weight >= 8.0, got {t_cost}"
        passes += 1
        print("  --> PASS: Crowded state successfully triggers velocity suppression and obstacle cost increase.")

        # TEST 3: Heavy Hospital Equipment (Bed detected)
        print("\n[TEST 3] Heavy Hospital Equipment State (Bed detected)...")
        for _ in range(8):
            harness.publish_objects([
                ('bed', 1.8, 0.2, 0.0, 0.0)
            ])
            harness.spin_duration(0.1)

        mode = harness.tuner_node.current_mode
        t_vx = harness.tuner_node.target_vx_max
        t_cost = harness.tuner_node.target_cost_weight
        print(f"  Mode: {mode} | Target Vx: {t_vx:.2f} m/s | Target Cost: {t_cost:.2f}")
        assert mode == "HEAVY_EQUIPMENT_CAUTION", f"Expected HEAVY_EQUIPMENT_CAUTION, got {mode}"
        assert t_vx <= 0.20, f"Expected vx <= 0.20, got {t_vx}"
        assert t_cost >= 7.5, f"Expected cost_weight >= 7.5, got {t_cost}"
        passes += 1
        print("  --> PASS: Heavy hospital equipment forces wide-clearance cautious sampling.")

        # TEST 4: Critical Congestion (2 people + Bed)
        print("\n[TEST 4] Critical Congestion (2 people + Bed)...")
        for _ in range(8):
            harness.publish_objects([
                ('bed', 2.0, 0.0, 0.0, 0.0),
                ('patient', 1.4, 0.5, 0.0, 0.0),
                ('doctor', 1.7, -0.4, 0.0, 0.0)
            ])
            harness.spin_duration(0.1)

        mode = harness.tuner_node.current_mode
        t_vx = harness.tuner_node.target_vx_max
        t_cost = harness.tuner_node.target_cost_weight
        print(f"  Mode: {mode} | Target Vx: {t_vx:.2f} m/s | Target Cost: {t_cost:.2f}")
        assert mode == "CRITICAL_CAUTION_CONGESTION", f"Expected CRITICAL_CAUTION_CONGESTION, got {mode}"
        assert t_vx <= 0.14, f"Expected vx <= 0.14, got {t_vx}"
        assert t_cost >= 9.5, f"Expected cost_weight >= 9.5, got {t_cost}"
        passes += 1
        print("  --> PASS: Critical congestion enforces minimum crawl speed and max clearance weight.")

        # TEST 5: Parameter Dispatch to Controller & Velocity Smoother
        print("\n[TEST 5] Validating Parameter Service Dispatch...")
        ctrl_params = harness.mock_node.received_controller_params
        smooth_params = harness.mock_node.received_smoother_params
        print(f"  Received Controller Params: {list(ctrl_params.keys())}")
        print(f"  Received Smoother Params: {list(smooth_params.keys())}")
        assert "FollowPath.vx_max" in ctrl_params, "Missing FollowPath.vx_max"
        assert "FollowPath.max_vel_x" in ctrl_params, "Missing FollowPath.max_vel_x alias"
        assert "FollowPath.CostCritic.cost_weight" in ctrl_params, "Missing CostCritic.cost_weight"
        assert "FollowPath.ObstaclesCost.cost_weight" in ctrl_params, "Missing ObstaclesCost.cost_weight alias"
        assert "max_velocity" in smooth_params, "Missing smoother max_velocity"
        passes += 1
        print("  --> PASS: Controller server and velocity smoother successfully receive tuned parameters.")

        # TEST 6: Scene Clearing & Hysteresis Delay
        print("\n[TEST 6] Scene Clearing and Hysteresis Reversion...")
        harness.publish_objects([])
        
        harness.spin_duration(0.2)
        immed_mode = harness.tuner_node.current_mode
        print(f"  Mode at t=0.2s: {immed_mode} (Expected caution held by hysteresis)")
        assert immed_mode != "CLEAR_DELIVERY", "Hysteresis failed: reverted prematurely"

        harness.spin_duration(2.0)
        final_mode = harness.tuner_node.current_mode
        final_vx = harness.tuner_node.active_vx_max
        final_cost = harness.tuner_node.active_cost_weight
        print(f"  Mode at t=2.2s: {final_mode} | Active Vx: {final_vx:.2f} m/s | Active Cost: {final_cost:.2f}")
        assert final_mode == "CLEAR_DELIVERY", f"Expected CLEAR_DELIVERY, got {final_mode}"
        assert final_vx > 0.35, f"Expected vx > 0.35 m/s, got {final_vx}"
        assert final_cost < 4.5, f"Expected cost < 4.5, got {final_cost}"
        passes += 1
        print("  --> PASS: Hysteresis correctly holds caution then smoothly reverts to delivery speed.")

        print("\n" + "=" * 70)
        print(f"ALL {passes}/{total} MPPI SCENE TUNER TESTS PASSED SUCCESSFULLY!")
        print("=" * 70)
        return True

    finally:
        harness.shutdown()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
