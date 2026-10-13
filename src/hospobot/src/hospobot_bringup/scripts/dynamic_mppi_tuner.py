#!/usr/bin/env python3
"""
Dynamic Intelligent MPPI Tuner for Hospobot
Author: Hospobot Team

Dynamically tunes Nav2 MPPI controller and velocity smoother parameters in real-time
based on high-level semantic scene understanding and inherent object properties:
  - Vulnerable dynamic agents (Pedestrians, Patients, Crowds >= 2 people)
  - Heavy, rigid, high-value hospital equipment (Beds, Wheelchairs, Gurneys)
  - Clear, unobstructed delivery corridors

Behavioral Logic:
  - If a "crowded" state (>= 2 people) or heavy equipment (bed, etc.) is detected:
    Dynamically reduces FollowPath.vx_max / max_vel_x and increases
    FollowPath.CostCritic.cost_weight (and ObstaclesCost.cost_weight)
    to force conservative, cautious trajectory sampling.
  - When the scene clears, smoothly reverts parameters to normal delivery speeds.
  - Incorporates inherent properties (agent mobility, intent velocity, structural fragility).
  - Smooth parameter slewing with hysteresis to prevent oscillation or chattering.

Outputs:
  - Dynamic parameter updates to /controller_server and /velocity_smoother
  - Diagnostic topic: /mppi/scene_tuning_state (std_msgs/String @ 5 Hz)
  - Event alerts to /nav2/decision_events on state changes
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import tf2_ros

try:
    from hospobot_interfaces.msg import SemanticObjectArray, SemanticObject
    HAS_SEMANTIC_MSG = True
except ImportError:
    HAS_SEMANTIC_MSG = False


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


class DynamicMppiTuner(Node):
    def __init__(self):
        super().__init__('dynamic_mppi_tuner')

        # Parameters
        self.declare_parameter('controller_node_name', 'controller_server')
        self.declare_parameter('smoother_node_name', 'velocity_smoother')
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('detection_horizon_m', 3.8)  # max range to consider objects
        self.declare_parameter('revert_delay_s', 0.8)  # hysteresis before reverting to fast delivery

        # Nominal parameters (Clear Corridor)
        self.nominal_vx_max = 0.40
        self.nominal_cost_weight = 3.81
        self.nominal_repulsion_weight = 1.2
        self.nominal_path_align_weight = 12.0

        # Tuning state
        self.current_mode = "CLEAR_DELIVERY"
        self.prev_mode = "CLEAR_DELIVERY"
        self.mode_reason = "Unobstructed corridor, operating at standard medical delivery speed."

        # Active parameter values (smoothly slewed)
        self.active_vx_max = self.nominal_vx_max
        self.active_cost_weight = self.nominal_cost_weight
        self.active_repulsion_weight = self.nominal_repulsion_weight
        self.active_path_align_weight = self.nominal_path_align_weight

        # Target parameter values
        self.target_vx_max = self.nominal_vx_max
        self.target_cost_weight = self.nominal_cost_weight
        self.target_repulsion_weight = self.nominal_repulsion_weight
        self.target_path_align_weight = self.nominal_path_align_weight

        # Last parameter set values (to avoid redundant service calls)
        self.last_sent_vx = -1.0
        self.last_sent_cost_w = -1.0
        self.last_sent_align_w = -1.0

        # Object tracking cache
        self.latest_objects = []
        self.last_semantic_time = 0.0
        self.clear_state_start_time = time.time()

        # TF & Robot Pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_vx = 0.0
        self.has_pose = False

        # Service Clients
        self.controller_param_client = self.create_client(
            SetParameters, f"/{self.get_parameter('controller_node_name').value}/set_parameters"
        )
        self.smoother_param_client = self.create_client(
            SetParameters, f"/{self.get_parameter('smoother_node_name').value}/set_parameters"
        )

        # Publishers
        self.pub_tuning_state = self.create_publisher(String, '/mppi/scene_tuning_state', 10)
        self.pub_decision_events = self.create_publisher(String, '/nav2/decision_events', 10)

        # Subscriptions
        if HAS_SEMANTIC_MSG:
            self.create_subscription(
                SemanticObjectArray, '/semantic/tracked_objects', self._cb_tracked_objects, 10
            )
            self.create_subscription(
                SemanticObjectArray, '/oakd/semantic_objects', self._cb_tracked_objects, 10
            )

        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)

        # Periodic Loop
        timer_period = 1.0 / self.get_parameter('update_rate').value
        self.timer = self.create_timer(timer_period, self._tuning_cycle)

        self.get_logger().info("Dynamic Intelligent MPPI Tuner initialized.")
        self.get_logger().info(f"Nominal Delivery Vx_max: {self.nominal_vx_max:.2f} m/s | CostWeight: {self.nominal_cost_weight:.2f}")

    def _cb_tracked_objects(self, msg: SemanticObjectArray):
        self.latest_objects = list(msg.objects)
        self.latest_frame_id = msg.header.frame_id if msg.header.frame_id else 'map'
        self.last_semantic_time = time.time()

    def _cb_odom(self, msg: Odometry):
        self.robot_vx = msg.twist.twist.linear.x

    def _update_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=rclpy.duration.Duration(seconds=0.05)
            )
            self.robot_x = trans.transform.translation.x
            self.robot_y = trans.transform.translation.y
            q = trans.transform.rotation
            self.robot_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            self.has_pose = True
        except Exception:
            try:
                trans = self.tf_buffer.lookup_transform(
                    'odom', 'base_footprint', Time(), timeout=rclpy.duration.Duration(seconds=0.05)
                )
                self.robot_x = trans.transform.translation.x
                self.robot_y = trans.transform.translation.y
                q = trans.transform.rotation
                self.robot_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
                self.has_pose = True
            except Exception:
                self.has_pose = False

    def _analyze_scene(self):
        """
        Evaluates inherent properties of obstacles in the interaction horizon:
          - Human vulnerability, agency, motion velocity
          - Heavy hospital equipment inertia, rigidity, fragility
        """
        now = time.time()
        horizon_m = self.get_parameter('detection_horizon_m').value

        # Check if perception stream is active
        if now - self.last_semantic_time > 0.8:
            # No recent objects detected -> clear scene
            return "CLEAR_DELIVERY", "No detected obstacles in immediate vicinity. Path clear for delivery.", [], []

        people_in_horizon = []
        equipment_in_horizon = []

        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        is_robot_frame = getattr(self, 'latest_frame_id', 'map') in ['base_footprint', 'base_link', 'oak_d_frame', 'camera_link']

        for obj in self.latest_objects:
            cname = obj.class_name.lower()
            ox = obj.position.x
            oy = obj.position.y

            if is_robot_frame:
                dist = math.hypot(ox, oy)
                forward_x = ox
                lateral_y = oy
                dx = ox
                dy = oy
            else:
                dx = ox - self.robot_x
                dy = oy - self.robot_y
                dist = math.hypot(dx, dy)
                forward_x = cos_yaw * dx + sin_yaw * dy
                lateral_y = -sin_yaw * dx + cos_yaw * dy

            if dist > horizon_m:
                continue

            # Consider objects in front (forward_x > -0.5m) and within corridor (lateral_y < 2.5m)
            if forward_x < -0.5 or abs(lateral_y) > 2.5:
                continue

            # Approaching rate analysis
            vx = getattr(obj.velocity, 'x', 0.0)
            vy = getattr(obj.velocity, 'y', 0.0)
            if is_robot_frame:
                v_approach = -vx
            else:
                v_approach = - (dx * vx + dy * vy) / max(dist, 0.1)

            obj_summary = {
                'id': obj.tracking_id,
                'class': cname,
                'dist': dist,
                'forward': forward_x,
                'approaching': v_approach > 0.15,
                'speed': math.hypot(vx, vy)
            }

            if any(k in cname for k in ['person', 'human', 'patient', 'doctor', 'nurse', 'pedestrian', 'visitor']):
                people_in_horizon.append(obj_summary)
            elif any(k in cname for k in ['bed', 'wheelchair', 'gurney', 'stretcher', 'cart', 'medical_cart', 'equipment', 'couch', 'sofa', 'chair', 'trolley', 'iv_pole']):
                equipment_in_horizon.append(obj_summary)

        num_people = len(people_in_horizon)
        num_equipment = len(equipment_in_horizon)

        # Inherent Scene Understanding Decision Matrix
        if num_people >= 2 and num_equipment >= 1:
            mode = "CRITICAL_CAUTION_CONGESTION"
            reason = (
                f"High-density scene detected: {num_people} humans and {num_equipment} heavy hospital assets. "
                f"Severe spatial constraint and vulnerability. Enforcing minimum crawl speed and max clearance."
            )
        elif num_people >= 2:
            approaching_str = " (pedestrians approaching)" if any(p['approaching'] for p in people_in_horizon) else ""
            mode = "CROWDED_STATE"
            reason = (
                f"Crowded corridor state: {num_people} people detected within {horizon_m:.1f}m{approaching_str}. "
                f"High human agency & movement unpredictability. Suppressing velocity and expanding trajectory safety weight."
            )
        elif num_equipment >= 1:
            eq_types = ", ".join(set(e['class'] for e in equipment_in_horizon))
            mode = "HEAVY_EQUIPMENT_CAUTION"
            reason = (
                f"Heavy hospital asset detected: {eq_types} ({num_equipment} item(s)). "
                f"Large physical mass, rigid structure, potential patient payload. Forcing wide clearance sampling."
            )
        elif num_people == 1:
            p = people_in_horizon[0]
            if p['approaching']:
                mode = "HUMAN_APPROACHING_ALERT"
                reason = f"Single human approaching at {p['speed']:.1f} m/s (dist: {p['dist']:.1f}m). Cautious passing mode."
            else:
                mode = "SINGLE_HUMAN_PROXIMITY"
                reason = f"Single human detected nearby (dist: {p['dist']:.1f}m). Moderate polite deceleration."
        else:
            mode = "CLEAR_DELIVERY"
            reason = "Corridor clear of pedestrians and medical equipment. Swift normal delivery speed enabled."

        return mode, reason, people_in_horizon, equipment_in_horizon

    def _determine_target_parameters(self, mode: str):
        """Map inherent scene mode to optimal MPPI sampling parameters."""
        if mode == "CRITICAL_CAUTION_CONGESTION":
            target_vx = 0.12
            target_cost_w = 10.0
            target_repulsion_w = 4.0
            target_align_w = 18.0
        elif mode == "CROWDED_STATE":
            target_vx = 0.16
            target_cost_w = 8.5
            target_repulsion_w = 3.5
            target_align_w = 16.0
        elif mode == "HEAVY_EQUIPMENT_CAUTION":
            target_vx = 0.18
            target_cost_w = 8.0
            target_repulsion_w = 3.2
            target_align_w = 14.0
        elif mode == "HUMAN_APPROACHING_ALERT":
            target_vx = 0.20
            target_cost_w = 6.5
            target_repulsion_w = 2.5
            target_align_w = 14.0
        elif mode == "SINGLE_HUMAN_PROXIMITY":
            target_vx = 0.26
            target_cost_w = 5.2
            target_repulsion_w = 2.0
            target_align_w = 13.0
        else:  # CLEAR_DELIVERY
            target_vx = self.nominal_vx_max
            target_cost_w = self.nominal_cost_weight
            target_repulsion_w = self.nominal_repulsion_weight
            target_align_w = self.nominal_path_align_weight

        return target_vx, target_cost_w, target_repulsion_w, target_align_w

    def _tuning_cycle(self):
        self._update_robot_pose()
        now = time.time()

        detected_mode, reason, people, equipment = self._analyze_scene()

        # Apply Hysteresis: Require scene to remain clear for revert_delay_s before reverting to fast delivery
        revert_delay = self.get_parameter('revert_delay_s').value
        if detected_mode == "CLEAR_DELIVERY":
            if self.current_mode != "CLEAR_DELIVERY":
                if (now - self.clear_state_start_time) < revert_delay:
                    # Maintain cautious mode until delay passes
                    effective_mode = self.current_mode
                else:
                    effective_mode = "CLEAR_DELIVERY"
                    self.current_mode = "CLEAR_DELIVERY"
            else:
                effective_mode = "CLEAR_DELIVERY"
        else:
            # Immediately enter cautious mode upon hazard detection
            self.clear_state_start_time = now
            effective_mode = detected_mode
            self.current_mode = detected_mode

        self.mode_reason = reason

        # Determine target parameters
        t_vx, t_cost_w, t_repulsion_w, t_align_w = self._determine_target_parameters(effective_mode)
        self.target_vx_max = t_vx
        self.target_cost_weight = t_cost_w
        self.target_repulsion_weight = t_repulsion_w
        self.target_path_align_weight = t_align_w

        # Smooth parameter slewing (Exponential filter at 10Hz)
        alpha = 0.25
        self.active_vx_max += alpha * (self.target_vx_max - self.active_vx_max)
        self.active_cost_weight += alpha * (self.target_cost_weight - self.active_cost_weight)
        self.active_repulsion_weight += alpha * (self.target_repulsion_weight - self.active_repulsion_weight)
        self.active_path_align_weight += alpha * (self.target_path_align_weight - self.active_path_align_weight)

        # Notify on state transition
        if effective_mode != self.prev_mode:
            event_str = (
                f"[MPPI_TUNER_TRANSITION] {self.prev_mode} -> {effective_mode} | "
                f"Target Vx: {self.target_vx_max:.2f} m/s | Target CostWeight: {self.target_cost_weight:.1f} | "
                f"Inherent Reason: {reason}"
            )
            self.get_logger().info(event_str)
            self.pub_decision_events.publish(String(data=event_str))
            self.prev_mode = effective_mode

        # Check if parameters need to be dispatched to controller_server
        should_send = (
            abs(self.active_vx_max - self.last_sent_vx) > 0.02 or
            abs(self.active_cost_weight - self.last_sent_cost_w) > 0.3 or
            abs(self.active_path_align_weight - self.last_sent_align_w) > 0.5
        )

        if should_send:
            self._dispatch_parameters()

        # Publish diagnostic state summary (5Hz)
        if not hasattr(self, '_diag_count'):
            self._diag_count = 0
        self._diag_count += 1
        if self._diag_count % 2 == 0:
            summary = (
                f"[MPPI_TUNING] MODE: {effective_mode} | "
                f"Vx_max: {self.active_vx_max:.2f} m/s (Target: {self.target_vx_max:.2f}) | "
                f"CostWeight: {self.active_cost_weight:.2f} | AlignWeight: {self.active_path_align_weight:.1f} | "
                f"People: {len(people)} | Equipment: {len(equipment)} | "
                f"Reason: {reason}"
            )
            self.pub_tuning_state.publish(String(data=summary))

    def _dispatch_parameters(self):
        """Send SetParameters request to controller_server and velocity_smoother."""
        self.last_sent_vx = self.active_vx_max
        self.last_sent_cost_w = self.active_cost_weight
        self.last_sent_align_w = self.active_path_align_weight

        # 1. Controller Server Parameters
        if self.controller_param_client.service_is_ready():
            req = SetParameters.Request()

            # Set vx_max / max_vel_x
            p_vx = Parameter()
            p_vx.name = "FollowPath.vx_max"
            p_vx.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(self.active_vx_max))
            req.parameters.append(p_vx)

            # Compatibility alias max_vel_x
            p_vx_alias = Parameter()
            p_vx_alias.name = "FollowPath.max_vel_x"
            p_vx_alias.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(self.active_vx_max))
            req.parameters.append(p_vx_alias)

            # Set CostCritic cost_weight
            p_cost = Parameter()
            p_cost.name = "FollowPath.CostCritic.cost_weight"
            p_cost.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(self.active_cost_weight))
            req.parameters.append(p_cost)

            # Compatibility alias ObstaclesCost.cost_weight
            p_obs_cost = Parameter()
            p_obs_cost.name = "FollowPath.ObstaclesCost.cost_weight"
            p_obs_cost.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(self.active_cost_weight))
            req.parameters.append(p_obs_cost)

            # Set ObstaclesCritic repulsion_weight
            p_rep = Parameter()
            p_rep.name = "FollowPath.ObstaclesCritic.repulsion_weight"
            p_rep.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(self.active_repulsion_weight))
            req.parameters.append(p_rep)

            # Set PathAlignCritic cost_weight
            p_align = Parameter()
            p_align.name = "FollowPath.PathAlignCritic.cost_weight"
            p_align.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(self.active_path_align_weight))
            req.parameters.append(p_align)

            future = self.controller_param_client.call_async(req)
            future.add_done_callback(self._on_param_response)

        # 2. Velocity Smoother max_velocity
        if self.smoother_param_client.service_is_ready():
            req_smooth = SetParameters.Request()
            p_sm = Parameter()
            p_sm.name = "max_velocity"
            p_sm.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=[float(self.active_vx_max), 0.0, 0.52]
            )
            req_smooth.parameters.append(p_sm)
            future_sm = self.smoother_param_client.call_async(req_smooth)
            future_sm.add_done_callback(self._on_param_response)

    def _on_param_response(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().debug(f"Service call result exception: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DynamicMppiTuner()
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
