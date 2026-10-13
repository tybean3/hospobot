#!/usr/bin/env python3
"""
Nav2 Decision Logger for Hospobot
Author: Hospobot Team

Monitors and logs real-time Nav2 navigation decisions with human-readable diagnostic explanations.
Helps diagnose and eliminate issues such as:
  - Abrupt stops (linear velocity dropped to 0.0)
  - Random or sudden in-place rotations (rotate_to_heading triggered)
  - Path-tracking oscillations (steering hunting, cross-track zig-zagging)
  - Collision monitor interventions (FootprintApproach slowdown/stop)
  - Costmap inflation / obstacle proximity halts
  - Behavior tree recovery triggers (Spin, BackUp, VLM interventions)

Outputs:
  1. Dedicated Rotating Log File: /home/hospobot/hospobot_ws/log/nav2_decisions.log
  2. Live Summary ROS Topic: /nav2/decision_summary (std_msgs/String @ 5 Hz)
  3. Live Event Alert ROS Topic: /nav2/decision_events (std_msgs/String on state change)
"""

import os
import sys
import math
import time
import collections
import logging
from logging.handlers import RotatingFileHandler

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, Odometry, OccupancyGrid
import tf2_ros

try:
    from nav2_msgs.msg import CollisionMonitorState
    HAS_COLLISION_MONITOR_MSG = True
except ImportError:
    HAS_COLLISION_MONITOR_MSG = False

try:
    from nav2_msgs.msg import BehaviorTreeLog
    HAS_BT_LOG_MSG = True
except ImportError:
    HAS_BT_LOG_MSG = False

try:
    from rcl_interfaces.msg import Log as RosoutLog
    HAS_ROSOUT_MSG = True
except ImportError:
    HAS_ROSOUT_MSG = False


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw angle from quaternion."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


class Nav2DecisionLogger(Node):
    def __init__(self):
        super().__init__('nav2_decision_logger')

        # Parameters
        self.declare_parameter('log_file_path', '/home/hospobot/hospobot_ws/log/nav2_decisions.log')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate', 5.0)  # Hz
        self.declare_parameter('rotate_to_heading_threshold_deg', 48.7)  # 0.85 rad
        self.declare_parameter('lookahead_dist', 0.65)  # meters

        self.log_file_path = self.get_parameter('log_file_path').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.rth_threshold_deg = float(self.get_parameter('rotate_to_heading_threshold_deg').value)
        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)

        # Setup file logger
        self._setup_file_logger()

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.pub_summary = self.create_publisher(String, '/nav2/decision_summary', 10)
        self.pub_events = self.create_publisher(String, '/nav2/decision_events', 10)

        # Internal state caches
        self.latest_plan = None           # List of (x, y)
        self.latest_plan_frame = self.map_frame
        self.latest_cmd_vel = Twist()     # Final cmd_vel (after collision monitor)
        self.latest_cmd_smoothed = Twist()# cmd_vel_smoothed (smoother output)
        self.latest_cmd_nav = Twist()     # raw controller output
        self.last_cmd_vel_time = 0.0

        self.latest_odom = None
        self.col_monitor_action = 0       # 0: DO_NOTHING, 1: STOP, 2: SLOWDOWN, 3: APPROACH, 4: LIMIT
        self.col_monitor_polygon = ""
        self.col_monitor_time = 0.0

        self.bt_active_node = "IDLE"
        self.bt_recent_events = collections.deque(maxlen=10)

        self.costmap_data = None
        self.costmap_info = None

        # Diagnostic tracking
        self.prev_linear_vel = 0.0
        self.prev_angular_vel = 0.0
        self.prev_state_name = "IDLE"
        self.last_stop_alert_time = 0.0
        self.last_oscillation_alert_time = 0.0

        # Oscillation window: timestamps and angular velocities
        self.angular_vel_history = collections.deque(maxlen=30)  # ~1.5s at 20Hz
        self.cross_track_history = collections.deque(maxlen=30)

        # Subscriptions
        self.create_subscription(Path, '/plan', self._cb_plan, 10)
        self.create_subscription(Path, '/transformed_global_plan', self._cb_transformed_plan, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self._cb_cmd_vel_smoothed, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self._cb_cmd_vel_nav, 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._cb_costmap, 10)

        if HAS_COLLISION_MONITOR_MSG:
            self.create_subscription(CollisionMonitorState, '/collision_monitor_state', self._cb_collision_monitor, 10)

        if HAS_BT_LOG_MSG:
            self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self._cb_bt_log, 10)

        if HAS_ROSOUT_MSG:
            self.create_subscription(RosoutLog, '/rosout', self._cb_rosout, 20)

        # Dynamic MPPI scene tuning state
        self.mppi_scene_state = ""
        self.create_subscription(String, '/mppi/scene_tuning_state', self._cb_mppi_tuning, 10)

        # Periodic decision analysis timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self._decision_cycle)

        self.file_logger.info("==================================================================")
        self.file_logger.info("Nav2 Decision Logger Initialized. Active and Monitoring.")
        self.file_logger.info(f"Target log file: {self.log_file_path}")
        self.file_logger.info(f"Rotate-to-heading threshold: {self.rth_threshold_deg:.1f}°")
        self.file_logger.info(f"Lookahead distance: {self.lookahead_dist:.2f}m")
        self.file_logger.info("==================================================================")
        self.get_logger().info(f"Nav2 Decision Logger started. Writing to {self.log_file_path}")

    def _setup_file_logger(self):
        """Configure rotating file handler for clean, persistent logging."""
        log_dir = os.path.dirname(self.log_file_path)
        if log_dir and not os.path.exists(log_dir):
            try:
                os.makedirs(log_dir, exist_ok=True)
            except Exception as e:
                self.get_logger().error(f"Failed to create directory {log_dir}: {e}")

        self.file_logger = logging.getLogger("Nav2DecisionLogger")
        self.file_logger.setLevel(logging.INFO)
        self.file_logger.propagate = False

        # Clear existing handlers
        for h in list(self.file_logger.handlers):
            self.file_logger.removeHandler(h)

        try:
            handler = RotatingFileHandler(
                self.log_file_path,
                maxBytes=10 * 1024 * 1024,  # 10 MB per file
                backupCount=5,
                encoding='utf-8'
            )
            formatter = logging.Formatter(
                fmt='%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S'
            )
            handler.setFormatter(formatter)
            self.file_logger.addHandler(handler)
        except Exception as e:
            self.get_logger().error(f"Failed to create file logger at {self.log_file_path}: {e}")
            # Fallback to console logger handler
            self.file_logger.addHandler(logging.StreamHandler(sys.stdout))

    # --- Topic Callbacks ---
    def _cb_plan(self, msg: Path):
        if msg.poses:
            self.latest_plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.latest_plan_frame = msg.header.frame_id or self.map_frame
            self._log_event(f"[PLAN_UPDATED] New path with {len(msg.poses)} waypoints received in frame '{self.latest_plan_frame}'.")

    def _cb_transformed_plan(self, msg: Path):
        # Transformed plan is the controller's active slice
        if msg.poses:
            self.latest_controller_plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _cb_cmd_vel(self, msg: Twist):
        self.latest_cmd_vel = msg
        self.last_cmd_vel_time = time.time()
        self.angular_vel_history.append((self.last_cmd_vel_time, msg.angular.z))

    def _cb_cmd_vel_smoothed(self, msg: Twist):
        self.latest_cmd_smoothed = msg

    def _cb_cmd_vel_nav(self, msg: Twist):
        self.latest_cmd_nav = msg

    def _cb_odom(self, msg: Odometry):
        self.latest_odom = msg

    def _cb_costmap(self, msg: OccupancyGrid):
        self.costmap_data = msg.data
        self.costmap_info = msg.info

    def _cb_collision_monitor(self, msg):
        prev_act = self.col_monitor_action
        self.col_monitor_action = msg.action_type
        self.col_monitor_polygon = msg.polygon_name
        self.col_monitor_time = time.time()

        action_names = {0: "DO_NOTHING", 1: "STOP", 2: "SLOWDOWN", 3: "APPROACH", 4: "LIMIT"}
        act_name = action_names.get(msg.action_type, f"UNKNOWN({msg.action_type})")

        if prev_act != msg.action_type and msg.action_type != 0:
            self._log_event(
                f"[COLLISION_MONITOR_TRIGGERED] Action={act_name} | Polygon='{msg.polygon_name}' | "
                f"Collision Monitor is overriding velocity!"
            )

    def _cb_mppi_tuning(self, msg: String):
        self.mppi_scene_state = msg.data

    def _cb_bt_log(self, msg):
        for event in msg.event_log:
            node_name = event.node_name
            curr_status = event.current_status
            prev_status = event.previous_status

            # Track primary execution nodes
            if curr_status == "RUNNING" and node_name not in ["MainTree", "NavigateRecovery", "NavigateWithReplanning", "PipelineSequence", "RateController"]:
                self.bt_active_node = node_name

            # Log significant status transitions
            if curr_status in ["FAILURE", "SUCCESS"] or "Recovery" in node_name or node_name in ["VLMSemanticIntervention", "BackUp", "Spin", "Wait"]:
                log_entry = f"[BT_EVENT] Node '{node_name}': {prev_status} -> {curr_status}"
                self.bt_recent_events.append(log_entry)
                if curr_status == "FAILURE" or "Recovery" in node_name:
                    self._log_event(log_entry)

    def _cb_rosout(self, msg):
        # Filter for Nav2 nodes
        nav2_nodes = [
            "controller_server", "planner_server", "bt_navigator",
            "behavior_server", "collision_monitor", "velocity_smoother"
        ]
        if any(node in msg.name for node in nav2_nodes):
            # Log WARN (30) and ERROR (40)
            if msg.level >= 30:
                lvl = "WARN" if msg.level == 30 else "ERROR"
                self._log_event(f"[NAV2_{lvl}] [{msg.name}] {msg.msg}")
            # Also catch specific diagnostic INFO logs
            elif any(k in msg.msg.lower() for k in ["collision detected", "rotate to heading", "aborted", "failed"]):
                self._log_event(f"[NAV2_INFO] [{msg.name}] {msg.msg}")

    # --- Geometry & Diagnostic Helpers ---
    def _get_robot_pose(self):
        """Lookup robot pose in the map (or odom) frame."""
        target_frame = self.latest_plan_frame if self.latest_plan_frame else self.map_frame
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame,
                self.base_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.08)
            )
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            return tx, ty, yaw
        except Exception:
            return None

    def _analyze_path_tracking(self, rx: float, ry: float, ryaw: float):
        """
        Compute:
          - Nearest point index
          - Cross-track error (signed meters)
          - Lookahead carrot position (x, y)
          - Angle to carrot (heading error in degrees)
          - Distance to goal (meters)
        """
        if not self.latest_plan or len(self.latest_plan) < 2:
            return None

        # 1. Find nearest waypoint
        min_dist_sq = float('inf')
        closest_idx = 0
        for i, (px, py) in enumerate(self.latest_plan):
            dsq = (px - rx) ** 2 + (py - ry) ** 2
            if dsq < min_dist_sq:
                min_dist_sq = dsq
                closest_idx = i

        cx, cy = self.latest_plan[closest_idx]
        raw_cte = math.sqrt(min_dist_sq)

        # Determine sign of CTE relative to path direction
        next_idx = min(len(self.latest_plan) - 1, closest_idx + 1)
        prev_idx = max(0, closest_idx - 1)
        p_dx = self.latest_plan[next_idx][0] - self.latest_plan[prev_idx][0]
        p_dy = self.latest_plan[next_idx][1] - self.latest_plan[prev_idx][1]
        path_angle = math.atan2(p_dy, p_dx)

        # Cross product (p_dx * (ry - cy) - p_dy * (rx - cx))
        cross_prod = p_dx * (ry - cy) - p_dy * (rx - cx)
        signed_cte = raw_cte if cross_prod >= 0 else -raw_cte

        # 2. Lookahead carrot point
        accum_dist = 0.0
        lookahead_idx = closest_idx
        for i in range(closest_idx, len(self.latest_plan) - 1):
            p1 = self.latest_plan[i]
            p2 = self.latest_plan[i + 1]
            seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            accum_dist += seg_len
            if accum_dist >= self.lookahead_dist:
                lookahead_idx = i + 1
                break
        else:
            lookahead_idx = len(self.latest_plan) - 1

        carrot_x, carrot_y = self.latest_plan[lookahead_idx]

        # 3. Angle to carrot (Heading Error)
        angle_to_carrot = math.atan2(carrot_y - ry, carrot_x - rx)
        heading_err_rad = normalize_angle(angle_to_carrot - ryaw)
        heading_err_deg = math.degrees(heading_err_rad)

        # 4. Remaining distance to goal along path
        dist_to_goal = 0.0
        for i in range(closest_idx, len(self.latest_plan) - 1):
            p1 = self.latest_plan[i]
            p2 = self.latest_plan[i + 1]
            dist_to_goal += math.hypot(p2[0] - p1[0], p2[1] - p1[1])

        # Record CTE for oscillation analysis
        self.cross_track_history.append((time.time(), signed_cte))

        return {
            'closest_idx': closest_idx,
            'cte': signed_cte,
            'abs_cte': raw_cte,
            'carrot': (carrot_x, carrot_y),
            'heading_err_deg': heading_err_deg,
            'dist_to_goal': dist_to_goal,
            'path_angle_deg': math.degrees(path_angle)
        }

    def _check_costmap_at_point(self, x: float, y: float) -> int:
        """Check cost at (x, y) in local costmap (-1 if out of bounds)."""
        if not self.costmap_data or not self.costmap_info:
            return -1
        res = self.costmap_info.resolution
        origin_x = self.costmap_info.origin.position.x
        origin_y = self.costmap_info.origin.position.y
        w = self.costmap_info.width
        h = self.costmap_info.height

        mx = int((x - origin_x) / res)
        my = int((y - origin_y) / res)
        if 0 <= mx < w and 0 <= my < h:
            idx = my * w + mx
            if 0 <= idx < len(self.costmap_data):
                return self.costmap_data[idx]
        return -1

    def _detect_oscillation(self) -> tuple[bool, str]:
        """
        Detect rapid steering reversals or cross-track weaving:
        If angular velocity changes sign 3+ times in the last 1.5 seconds
        with high magnitude, or if CTE alternates signs rapidly.
        """
        now = time.time()
        # Look at entries from the last 1.5 seconds
        recent_w = [w for t, w in self.angular_vel_history if now - t <= 1.5 and abs(w) >= 0.12]
        if len(recent_w) >= 5:
            sign_flips = 0
            for i in range(len(recent_w) - 1):
                if (recent_w[i] > 0 and recent_w[i + 1] < 0) or (recent_w[i] < 0 and recent_w[i + 1] > 0):
                    sign_flips += 1
            if sign_flips >= 3:
                freq = sign_flips / 1.5
                return True, f"Steering hunting detected ({sign_flips} reversals in 1.5s, ~{freq:.1f}Hz)"

        return False, ""

    def _diagnose_abrupt_stop(self, tracking_info: dict, cur_vx: float, cur_wz: float) -> str:
        """Determine exactly why the robot came to an abrupt stop."""
        reasons = []

        # 1. Rotate to heading check
        if tracking_info:
            herr = abs(tracking_info['heading_err_deg'])
            if herr >= self.rth_threshold_deg:
                reasons.append(
                    f"ROTATE_TO_HEADING: Heading error ({herr:.1f}°) exceeds threshold ({self.rth_threshold_deg:.1f}°). "
                    f"RPPC controller halts linear velocity (v=0.0) to align in-place with angular_vel={cur_wz:+.2f} rad/s."
                )

        # 2. Collision monitor check
        if self.col_monitor_action == 1:  # STOP
            reasons.append(
                f"COLLISION_MONITOR_STOP: Collision monitor triggered on polygon '{self.col_monitor_polygon}'. "
                f"Stopping robot immediately to avoid collision."
            )
        elif self.col_monitor_action == 3:  # APPROACH
            reasons.append(
                f"COLLISION_MONITOR_APPROACH: Collision monitor approach limit active on polygon '{self.col_monitor_polygon}'."
            )

        # 3. Local costmap check along carrot
        if tracking_info:
            cx, cy = tracking_info['carrot']
            cost = self._check_costmap_at_point(cx, cy)
            if cost >= 253:  # Lethal / Inscribed
                reasons.append(
                    f"OBSTACLE_ON_PATH: Lookahead carrot is located in high-cost obstacle cell (cost={cost}). "
                    f"RPPC collision detection halted forward motion."
                )

        # 4. Behavior Tree Recovery
        if "Recovery" in self.bt_active_node or self.bt_active_node in ["Spin", "BackUp", "Wait", "VLMSemanticIntervention"]:
            reasons.append(
                f"BT_RECOVERY: Behavior Tree is executing recovery action '{self.bt_active_node}'."
            )

        # 5. Goal Arrival
        if tracking_info and tracking_info['dist_to_goal'] < 0.25:
            reasons.append(
                f"GOAL_REACHED: Distance to goal is {tracking_info['dist_to_goal']:.2f}m (< 0.25m tolerance)."
            )

        if not reasons:
            reasons.append(
                f"CONTROLLER_ZERO_VELOCITY: Controller commanded linear_vel=0.0 while w={cur_wz:+.2f} rad/s. "
                f"Check controller status or costmap clearance."
            )

        return " | ".join(reasons)

    # --- Main Decision Cycle (Runs at 5 Hz) ---
    def _decision_cycle(self):
        pose = self._get_robot_pose()
        now = time.time()

        is_cmd_recent = (now - self.last_cmd_vel_time) < 0.5
        cur_vx = self.latest_cmd_vel.linear.x if is_cmd_recent else 0.0
        cur_wz = self.latest_cmd_vel.angular.z if is_cmd_recent else 0.0
        smooth_vx = self.latest_cmd_smoothed.linear.x if is_cmd_recent else 0.0

        if pose is None:
            # TF not ready
            summary = "[NAV2_DECISION] Waiting for TF transform (map -> base_footprint)..."
            self.pub_summary.publish(String(data=summary))
            return

        rx, ry, ryaw = pose
        tracking = self._analyze_path_tracking(rx, ry, ryaw)

        # State Classification
        decision_state = "IDLE"
        state_reason = ""

        if not self.latest_plan:
            decision_state = "NO_PLAN"
            state_reason = "No active global plan received."
        elif tracking and tracking['dist_to_goal'] < 0.25 and abs(cur_vx) < 0.03:
            decision_state = "GOAL_REACHED"
            state_reason = f"Within goal tolerance (dist={tracking['dist_to_goal']:.2f}m)"
        elif "Recovery" in self.bt_active_node or self.bt_active_node in ["Spin", "BackUp", "Wait", "VLMSemanticIntervention"]:
            decision_state = f"RECOVERY_{self.bt_active_node.upper()}"
            state_reason = f"BT executing recovery: {self.bt_active_node}"
        elif self.col_monitor_action == 1:
            decision_state = "COLLISION_STOP"
            state_reason = f"Collision monitor '{self.col_monitor_polygon}' stopped robot"
        elif abs(cur_vx) < 0.03 and abs(cur_wz) >= 0.05:
            turn_dir = "CCW_LEFT" if cur_wz > 0 else "CW_RIGHT"
            decision_state = "ROTATE_TO_HEADING"
            herr = tracking['heading_err_deg'] if tracking else 0.0
            state_reason = (
                f"Aligning in-place ({turn_dir}, w={cur_wz:+.2f}r/s). "
                f"HeadingErr: {herr:+.1f}° vs Thresh: {self.rth_threshold_deg:.1f}°"
            )
        elif cur_vx >= 0.03:
            decision_state = "DRIVING_FORWARD"
            state_reason = f"Tracking path at {cur_vx:.2f} m/s (smoothed: {smooth_vx:.2f} m/s)"
        else:
            decision_state = "STOPPED"
            state_reason = "Linear and angular velocities are zero."

        # Detect Abrupt Stop Event
        if self.prev_linear_vel > 0.15 and cur_vx < 0.03 and (now - self.last_stop_alert_time) > 1.0:
            self.last_stop_alert_time = now
            stop_reason = self._diagnose_abrupt_stop(tracking, cur_vx, cur_wz)
            self._log_event(
                f"[EVENT: ABRUPT_STOP] Robot halted from vx={self.prev_linear_vel:.2f} m/s to {cur_vx:.2f} m/s! "
                f"DIAGNOSIS: {stop_reason}"
            )

        # Detect Oscillation Event
        is_oscillating, osc_reason = self._detect_oscillation()
        if is_oscillating and (now - self.last_oscillation_alert_time) > 2.0:
            self.last_oscillation_alert_time = now
            cte_str = f"CTE: {tracking['cte']:+.2f}m" if tracking else ""
            self._log_event(
                f"[EVENT: OSCILLATION] Robot is oscillating around the path! {osc_reason} | {cte_str} | "
                f"Current Wz={cur_wz:+.2f} rad/s"
            )

        # Detect State Transitions
        if decision_state != self.prev_state_name:
            self._log_event(f"[STATE_TRANSITION] {self.prev_state_name} -> {decision_state} | {state_reason}")
            self.prev_state_name = decision_state

        # Update historical speeds
        self.prev_linear_vel = cur_vx
        self.prev_angular_vel = cur_wz

        # Compose Human-Readable Live Summary
        cte_str = f"CTE: {tracking['cte']:+.2f}m" if tracking else "CTE: N/A"
        herr_str = f"HeadErr: {tracking['heading_err_deg']:+.1f}°" if tracking else "HeadErr: N/A"
        goal_str = f"DistGoal: {tracking['dist_to_goal']:.2f}m" if tracking else "DistGoal: N/A"
        col_str = "ColMon: OK" if self.col_monitor_action == 0 else f"ColMon: ACT_{self.col_monitor_action}"
        mppi_str = f" | {self.mppi_scene_state}" if self.mppi_scene_state else ""

        summary_str = (
            f"[{decision_state}] Vx={cur_vx:+.2f}m/s Wz={cur_wz:+.2f}r/s | "
            f"{cte_str} | {herr_str} | {goal_str} | {col_str} | BT: {self.bt_active_node}{mppi_str} | "
            f"Why: {state_reason}"
        )

        # Publish to ROS topic
        self.pub_summary.publish(String(data=summary_str))

        # Periodic log file heartbeat (every ~2 seconds = every 10 cycles)
        if not hasattr(self, '_cycle_count'):
            self._cycle_count = 0
        self._cycle_count += 1
        if self._cycle_count % 10 == 0:
            self.file_logger.info(summary_str)

    def _log_event(self, event_msg: str):
        """Log significant decision event to file, ROS events topic, and console."""
        self.file_logger.warning(event_msg)
        self.get_logger().info(event_msg)
        self.pub_events.publish(String(data=event_msg))


def main(args=None):
    rclpy.init(args=args)
    node = Nav2DecisionLogger()
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
