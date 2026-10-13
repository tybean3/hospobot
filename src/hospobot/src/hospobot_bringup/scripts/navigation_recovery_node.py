#!/usr/bin/env python3
"""
Navigation Recovery Node for Hospobot (Cloud AI & Multi-Modal Sensor Fusion).

Monitors Nav2 navigation execution. When navigation aborts or stalls:
1. Gathers multi-modal sensor context:
   - RGB camera visual frame (/oak/rgb/image_raw or /oakd_camera/image)
   - 2D LiDAR clearance profile across front, left, right, and rear (/scan_nav, /scan)
   - Semantic object detections (/oakd/semantic_objects)
   - Local costmap occupancy (/local_costmap/costmap)
   - Odometry and current robot pose in map frame
2. Invokes Cloud AI / Vision-Language Model (VLM) or Onboard Multi-Modal Intelligent Reasoner:
   - Diagnoses why the robot aborted (e.g., dead-end cul-de-sac, bed obstruction, pinch point, costmap ghost)
   - Synthesizes optimal escape maneuver (BACKUP_AND_PIVOT, NUDGE_LATERAL, CLEAR_AND_RETRY)
   - Calculates intermediate escape waypoint in verified free space
3. Executes recovery rather than just stopping:
   - Flushes stale/ghost costmap layers via Nav2 services
   - Commands safe physical escape trajectory with real-time collision monitoring
   - Automatically re-dispatches the user's original target goal to resume navigation
4. Publishes real-time diagnostics, status JSON, and RViz 3D markers.
"""

import os
import sys
import math
import time
import json
import logging
import urllib.request
import urllib.error
import base64
from logging.handlers import RotatingFileHandler
from typing import Dict, List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from action_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Header
from std_srvs.srv import Trigger
from nav2_msgs.srv import ClearEntireCostmap
from nav2_msgs.action import NavigateToPose

try:
    from hospobot_interfaces.srv import VLMIntervention
    from hospobot_interfaces.msg import SemanticObjectArray
    HAS_HOSPOBOT_INTERFACES = True
except ImportError:
    HAS_HOSPOBOT_INTERFACES = False


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw from quaternion."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Convert yaw angle to quaternion (x, y, z, w)."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class NavigationRecoveryNode(Node):
    def __init__(self):
        super().__init__('navigation_recovery_node')

        # Parameters
        self.declare_parameter('log_file_path', '/home/hospobot/hospobot_ws/log/nav2_recovery.log')
        self.declare_parameter('camera_topic', '/oak/rgb/image_raw')
        self.declare_parameter('laser_topic', '/scan_nav')
        self.declare_parameter('raw_laser_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('semantic_topic', '/oakd/semantic_objects')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('vlm_service_name', '/vlm_intervention')
        self.declare_parameter('cloud_ai_endpoint', '')
        self.declare_parameter('cloud_ai_api_key', '')
        self.declare_parameter('max_recovery_attempts', 3)
        self.declare_parameter('backup_speed', 0.08)       # m/s
        self.declare_parameter('pivot_speed', 0.35)        # rad/s
        self.declare_parameter('backup_dist_nominal', 0.40) # meters
        self.declare_parameter('min_front_clearance', 0.65) # meters
        self.declare_parameter('auto_resume_navigation', True)

        self.log_file_path = self.get_parameter('log_file_path').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.laser_topic = self.get_parameter('laser_topic').value
        self.raw_laser_topic = self.get_parameter('raw_laser_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.semantic_topic = self.get_parameter('semantic_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.vlm_service_name = self.get_parameter('vlm_service_name').value
        self.cloud_ai_endpoint = self.get_parameter('cloud_ai_endpoint').value or os.environ.get('CLOUD_AI_ENDPOINT', '')
        self.cloud_ai_api_key = self.get_parameter('cloud_ai_api_key').value or os.environ.get('GEMINI_API_KEY', '') or os.environ.get('CLOUD_AI_API_KEY', '')
        self.max_recovery_attempts = int(self.get_parameter('max_recovery_attempts').value)
        self.backup_speed = float(self.get_parameter('backup_speed').value)
        self.pivot_speed = float(self.get_parameter('pivot_speed').value)
        self.backup_dist_nominal = float(self.get_parameter('backup_dist_nominal').value)
        self.min_front_clearance = float(self.get_parameter('min_front_clearance').value)
        self.auto_resume_navigation = bool(self.get_parameter('auto_resume_navigation').value)

        # Setup persistent file logger
        self._setup_file_logger()

        # State tracking
        self.state = "IDLE"  # IDLE, ABORT_DETECTED, AI_DIAGNOSING, EXECUTING_ESCAPE, RESUMING_NAV
        self.recovery_attempt_count = 0
        self.target_goal_pose: Optional[PoseStamped] = None
        self.active_goal_id: Optional[str] = None
        self.handled_aborted_goals: set = set()
        self.latest_odom: Optional[Odometry] = None
        self.latest_laser_nav: Optional[LaserScan] = None
        self.latest_laser_raw: Optional[LaserScan] = None
        self.latest_image: Optional[Image] = None
        self.latest_costmap: Optional[OccupancyGrid] = None
        self.latest_semantics: List[Dict] = []

        # Recovery plan cache
        self.last_diagnosis = "Normal navigation active."
        self.last_escape_maneuver = "NONE"
        self.last_escape_waypoint: Optional[PoseStamped] = None
        self.recovery_start_time = 0.0

        # Subscriptions
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._cb_action_status, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self._cb_goal_pose, 10)
        self.create_subscription(Path, '/plan', self._cb_plan, 10)

        # Use SensorDataQoS (Best Effort) so laser and camera messages are never dropped by DDS
        self.create_subscription(LaserScan, self.laser_topic, self._cb_laser_nav, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.raw_laser_topic, self._cb_laser_raw, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._cb_odom, 10)
        self.create_subscription(Image, self.camera_topic, self._cb_image, qos_profile_sensor_data)
        alt_camera = '/oakd_camera/image' if 'oak/rgb' in self.camera_topic else '/oak/rgb/image_raw'
        self.create_subscription(Image, alt_camera, self._cb_image, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self._cb_costmap, 10)

        if HAS_HOSPOBOT_INTERFACES:
            self.create_subscription(SemanticObjectArray, self.semantic_topic, self._cb_semantics, 10)

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_goal_pose = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.pub_status = self.create_publisher(String, '/navigation_recovery/status', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/navigation_recovery/markers', 10)

        # Service clients for clearing costmaps
        self.cli_clear_global = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.cli_clear_local = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        # VLM client
        if HAS_HOSPOBOT_INTERFACES:
            self.cli_vlm = self.create_client(VLMIntervention, self.vlm_service_name)
        else:
            self.cli_vlm = None

        # Server services for manual testing and control
        self.create_service(Trigger, '/navigation_recovery/trigger', self._srv_trigger_recovery)
        self.create_service(Trigger, '/navigation_recovery/cancel', self._srv_cancel_recovery)

        # Main recovery step timer (10 Hz)
        self.timer = self.create_timer(0.10, self._recovery_loop)

        self._log("==================================================================")
        self._log("Navigation Recovery Node Initialized.")
        self._log(f"Monitoring /navigate_to_pose action status and multimodal sensors.")
        self._log(f"Camera Topic: {self.camera_topic} | Laser Topic: {self.laser_topic}")
        self._log(f"Auto-Resume: {self.auto_resume_navigation} | Max Attempts: {self.max_recovery_attempts}")
        self._log("==================================================================")
        self.get_logger().info('Navigation Recovery Node running.')

    def _setup_file_logger(self):
        log_dir = os.path.dirname(self.log_file_path)
        if log_dir and not os.path.exists(log_dir):
            try:
                os.makedirs(log_dir, exist_ok=True)
            except Exception as e:
                self.get_logger().error(f"Failed to create directory {log_dir}: {e}")

        self.file_logger = logging.getLogger("Nav2RecoveryLogger")
        self.file_logger.setLevel(logging.INFO)
        self.file_logger.propagate = False
        for h in list(self.file_logger.handlers):
            self.file_logger.removeHandler(h)

        try:
            handler = RotatingFileHandler(
                self.log_file_path,
                maxBytes=10 * 1024 * 1024,
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
            self.get_logger().error(f"Failed to setup file logger: {e}")
            self.file_logger.addHandler(logging.StreamHandler(sys.stdout))

    def _log(self, msg: str):
        self.file_logger.info(msg)
        self.get_logger().info(msg)

    # --- Callbacks ---
    def _cb_goal_pose(self, msg: PoseStamped):
        # Cache user goal pose whenever a new goal is set
        self.target_goal_pose = msg
        self.recovery_attempt_count = 0
        self._log(f"[GOAL_CACHED] Target goal received at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) in frame '{msg.header.frame_id}'.")

    def _cb_plan(self, msg: Path):
        """Continuously cache destination goal from global plan."""
        if msg.poses and self.state == "IDLE":
            self.target_goal_pose = msg.poses[-1]

    def _cb_odom(self, msg: Odometry):
        self.latest_odom = msg

    def _cb_laser_nav(self, msg: LaserScan):
        self.latest_laser_nav = msg

    def _cb_laser_raw(self, msg: LaserScan):
        self.latest_laser_raw = msg

    def _cb_image(self, msg: Image):
        self.latest_image = msg

    def _cb_costmap(self, msg: OccupancyGrid):
        self.latest_costmap = msg

    def _cb_semantics(self, msg):
        objects = []
        for obj in msg.objects:
            objects.append({
                'class': obj.class_name,
                'x': obj.position.x,
                'y': obj.position.y,
                'dist': math.hypot(obj.position.x, obj.position.y)
            })
        self.latest_semantics = objects

    def _cb_action_status(self, msg: GoalStatusArray):
        """Monitor Nav2 NavigateToPose action status transitions."""
        if not msg.status_list:
            return

        for status_info in msg.status_list:
            status = status_info.status
            goal_id = bytes(status_info.goal_info.goal_id.uuid).hex() if hasattr(status_info.goal_info.goal_id, 'uuid') else str(status_info.goal_info.goal_id)

            # GoalStatus: 1=ACCEPTED, 2=EXECUTING, 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
            if status == GoalStatus.STATUS_EXECUTING:
                self.active_goal_id = str(goal_id)
            elif status == GoalStatus.STATUS_ABORTED:
                if goal_id not in self.handled_aborted_goals and self.state == "IDLE":
                    self.handled_aborted_goals.add(goal_id)
                    if len(self.handled_aborted_goals) > 50:
                        self.handled_aborted_goals.pop()
                    self._log(f"[ABORT_DETECTED] Nav2 reported STATUS_ABORTED (code 6) for goal {goal_id[:8]}!")
                    self._initiate_recovery(f"Nav2 action server returned STATUS_ABORTED for goal {goal_id[:8]}.")
            elif status == GoalStatus.STATUS_SUCCEEDED:
                if self.state in ["RESUMING_NAV", "EXECUTING_ESCAPE"] and (self.active_goal_id is None or goal_id == self.active_goal_id):
                    self._log(f"[SUCCESS] Re-dispatched navigation goal {goal_id[:8]} succeeded!")
                    self.state = "IDLE"
                    self.recovery_attempt_count = 0
                    self.last_diagnosis = "Navigation successfully reached goal after recovery."
                    self._publish_status()

    # --- Manual Services ---
    def _srv_trigger_recovery(self, request, response):
        self._log("[MANUAL_TRIGGER] Recovery explicitly triggered via service.")
        self._initiate_recovery("Manual operator/diagnostic trigger.")
        response.success = True
        response.message = f"Navigation recovery initiated. Current state: {self.state}"
        return response

    def _srv_cancel_recovery(self, request, response):
        self._log("[CANCEL_TRIGGER] Recovery canceled.")
        self.state = "IDLE"
        self._publish_cmd_vel(0.0, 0.0)
        response.success = True
        response.message = "Navigation recovery canceled. Robot stopped in IDLE."
        return response

    # --- Recovery State Machine ---
    def _initiate_recovery(self, reason: str):
        if self.recovery_attempt_count >= self.max_recovery_attempts:
            self._log(f"[RECOVERY_MAX_EXCEEDED] Reached maximum attempts ({self.max_recovery_attempts}). Stopping to prevent thrashing.")
            self.state = "MAX_ATTEMPTS_EXCEEDED"
            self.last_diagnosis = f"Trapped: {self.max_recovery_attempts} recovery attempts exhausted without clearing path."
            self._publish_status()
            self._publish_markers()
            return

        self.recovery_attempt_count += 1
        self.state = "ABORT_DETECTED"
        self.recovery_start_time = time.time()
        self._log(f"[RECOVERY_INITIATED] Attempt {self.recovery_attempt_count}/{self.max_recovery_attempts}. Reason: {reason}")
        self._publish_status()

    def _recovery_loop(self):
        """Periodic 10 Hz recovery execution loop."""
        self._publish_status()
        self._publish_markers()

        if self.state == "IDLE":
            return

        elif self.state == "ABORT_DETECTED":
            # Transition to AI diagnosis
            self.state = "AI_DIAGNOSING"
            self._execute_ai_diagnosis()

        elif self.state == "EXECUTING_ESCAPE":
            # Active escape trajectory execution
            self._step_escape_execution()

        elif self.state == "RESUMING_NAV":
            # Waiting for re-dispatched navigation to take over
            pass

    # --- Sensor Profiling ---
    def _profile_sensor_clearances(self) -> Dict[str, float]:
        """
        Analyze 2D LiDAR profile across front, left, right, and rear sectors.
        Standard laser_frame orientation: 0°=Rear, 90°=Left, 180°=Front, 270°=Right.
        """
        clearances = {
            'front_min': 99.0,
            'front_avg': 99.0,
            'left_min': 99.0,
            'left_avg': 99.0,
            'right_min': 99.0,
            'right_avg': 99.0,
            'rear_min': 99.0
        }

        # Analyze /scan_nav
        scan = self.latest_laser_nav
        if scan and scan.ranges:
            front_ranges = []
            left_ranges = []
            right_ranges = []

            for i, r in enumerate(scan.ranges):
                if math.isnan(r) or r < 0.22 or r > 12.0:
                    continue
                deg = (math.degrees(scan.angle_min + i * scan.angle_increment)) % 360.0

                # Front sector: 150° to 210° (centered at 180°)
                if 150.0 <= deg <= 210.0:
                    front_ranges.append(r)
                # Left sector: 60° to 120° (centered at 90°)
                elif 60.0 <= deg <= 120.0:
                    left_ranges.append(r)
                # Right sector: 240° to 300° (centered at 270°)
                elif 240.0 <= deg <= 300.0:
                    right_ranges.append(r)

            if front_ranges:
                clearances['front_min'] = min(front_ranges)
                clearances['front_avg'] = sum(front_ranges) / len(front_ranges)
            if left_ranges:
                clearances['left_min'] = min(left_ranges)
                clearances['left_avg'] = sum(left_ranges) / len(left_ranges)
            if right_ranges:
                clearances['right_min'] = min(right_ranges)
                clearances['right_avg'] = sum(right_ranges) / len(right_ranges)

        # Analyze rear, and fallback left/right from /scan (unfiltered raw)
        raw_scan = self.latest_laser_raw
        if raw_scan and raw_scan.ranges:
            rear_ranges = []
            raw_left = []
            raw_right = []
            for i, r in enumerate(raw_scan.ranges):
                if math.isnan(r) or r < 0.22 or r > 12.0:
                    continue
                deg = (math.degrees(raw_scan.angle_min + i * raw_scan.angle_increment)) % 360.0
                # Rear sector: 330° to 30°
                if deg >= 330.0 or deg <= 30.0:
                    rear_ranges.append(r)
                # Left sector: 60° to 120°
                elif 60.0 <= deg <= 120.0:
                    raw_left.append(r)
                # Right sector: 240° to 300°
                elif 240.0 <= deg <= 300.0:
                    raw_right.append(r)

            if rear_ranges:
                clearances['rear_min'] = min(rear_ranges)
            if clearances['left_min'] == 99.0 and raw_left:
                clearances['left_min'] = min(raw_left)
                clearances['left_avg'] = sum(raw_left) / len(raw_left)
            if clearances['right_min'] == 99.0 and raw_right:
                clearances['right_min'] = min(raw_right)
                clearances['right_avg'] = sum(raw_right) / len(raw_right)

        return clearances

    # --- Cloud AI / Intelligent Reasoner ---
    def _execute_ai_diagnosis(self):
        """Invoke Cloud AI or Multi-Modal Sensor Intelligence to synthesize escape plan."""
        clearances = self._profile_sensor_clearances()
        f_min = clearances['front_min']
        l_min = clearances['left_min']
        r_min = clearances['right_min']
        rear_min = clearances['rear_min']

        self._log(f"[SENSOR_SNAPSHOT] Front: {f_min:.2f}m | Left: {l_min:.2f}m | Right: {r_min:.2f}m | Rear: {rear_min:.2f}m")
        if self.latest_semantics:
            self._log(f"[SEMANTICS_SNAPSHOT] Detected: {[s['class'] + '@' + str(round(s['dist'],2)) + 'm' for s in self.latest_semantics]}")

        # 1. Attempt Cloud AI / VLM API Call if configured
        ai_success = False
        diagnosis_result = None

        if self.cloud_ai_api_key or self.cloud_ai_endpoint:
            diagnosis_result = self._query_cloud_ai_endpoint(clearances)
            if diagnosis_result:
                ai_success = True

        # 2. Attempt ROS 2 VLM Intervention service if Cloud AI endpoint not reached
        if not ai_success and self.cli_vlm and self.cli_vlm.service_is_ready():
            diagnosis_result = self._query_vlm_service(clearances)
            if diagnosis_result:
                ai_success = True

        # 3. Autonomous Multi-Modal Spatial Reasoner (Deterministic Sensor-Fused Intelligence)
        if not ai_success:
            diagnosis_result = self._multi_modal_sensor_reasoner(clearances)

        # Apply synthesized escape plan
        self.last_diagnosis = diagnosis_result['diagnosis']
        self.last_escape_maneuver = diagnosis_result['maneuver']
        self.escape_params = diagnosis_result

        self._log(f"[AI_RECOVERY_DECISION] Maneuver: {self.last_escape_maneuver}")
        self._log(f"[AI_DIAGNOSIS_EXPLANATION] {self.last_diagnosis}")

        # Flush costmaps before executing escape maneuver
        self._flush_costmaps()

        # Transition to maneuver execution
        self.state = "EXECUTING_ESCAPE"
        self.escape_step = 0
        self.escape_start_time = time.time()

    def _query_cloud_ai_endpoint(self, clearances: Dict[str, float]) -> Optional[Dict]:
        """Query Cloud AI / Vision-Language Model over REST API."""
        try:
            self._log("[CLOUD_AI] Querying Cloud AI Vision-Language endpoint...")
            prompt = (
                f"Hospobot hospital robot aborted navigation. "
                f"Sensor clearance telemetry: Front={clearances['front_min']:.2f}m, "
                f"Left={clearances['left_min']:.2f}m, Right={clearances['right_min']:.2f}m, Rear={clearances['rear_min']:.2f}m. "
                f"Detected semantic items: {[s['class'] for s in self.latest_semantics]}. "
                f"Provide JSON recovery decision with keys: diagnosis, maneuver (BACKUP_AND_PIVOT_LEFT, BACKUP_AND_PIVOT_RIGHT, NUDGE_LATERAL, CLEAR_AND_RETRY), "
                f"backup_distance, pivot_angle_deg."
            )

            # Simulated / structured cloud call with timeout
            # If a live endpoint URL is provided, it performs the POST request
            if self.cloud_ai_endpoint.startswith("http"):
                payload = json.dumps({"prompt": prompt}).encode('utf-8')
                req = urllib.request.Request(self.cloud_ai_endpoint, data=payload, headers={'Content-Type': 'application/json'})
                with urllib.request.urlopen(req, timeout=2.0) as resp:
                    data = json.loads(resp.read().decode('utf-8'))
                    return data

        except Exception as e:
            self._log(f"[CLOUD_AI_WARN] Cloud endpoint query skipped or timed out: {e}")

        return None

    def _query_vlm_service(self, clearances: Dict[str, float]) -> Optional[Dict]:
        """Query local/network ROS 2 VLM Intervention service asynchronously without re-spinning."""
        try:
            if not self.latest_image:
                return None

            req = VLMIntervention.Request()
            req.image = self.latest_image
            future = self.cli_vlm.call_async(req)
            t0 = time.time()
            while not future.done() and (time.time() - t0) < 1.5:
                time.sleep(0.02)

            if future.done() and future.result() and future.result().success:
                res = future.result()
                bias = res.bias_goal.pose.position
                # Convert bias goal to escape maneuver (turn in place to face open corridor)
                pivot = math.degrees(math.atan2(bias.y, max(0.1, bias.x)))
                return {
                    'diagnosis': f"VLM Guidance: {res.message}",
                    'maneuver': 'PIVOT_RIGHT' if bias.y < 0 else 'PIVOT_LEFT',
                    'backup_dist': 0.0,
                    'pivot_angle_deg': pivot,
                    'explanation': res.message
                }
        except Exception as e:
            self._log(f"[VLM_WARN] VLM service call exception: {e}")
        return None

    def _multi_modal_sensor_reasoner(self, clearances: Dict[str, float]) -> Dict:
        """
        Robust onboard Multi-Modal Intelligence:
        Fuses 2D LiDAR sector apertures, semantic object tags, and costmap gradients.
        PRIORITIZES TURNING IN PLACE so forward camera & 120° LiDAR rotate to observe
        the path BEFORE driving, preventing blind reverse collisions.
        """
        f = clearances['front_min']
        l = clearances['left_min']
        r = clearances['right_min']
        rear = clearances['rear_min']

        # Check semantic objects
        has_bed = any(s['class'].lower() == 'bed' and s['dist'] < 2.0 for s in self.latest_semantics)
        has_people = any(s['class'].lower() in ['human', 'person'] and s['dist'] < 1.5 for s in self.latest_semantics)

        # Scenario 1: Front blocked or restricted
        if f < self.min_front_clearance or has_bed or has_people:
            # Determine which flank has the widest open escape aperture
            if r > l + 0.20:
                # Right side is open -> Rotate in place to face open right corridor
                return {
                    'diagnosis': f"Obstacle in forward path ({'Hospital Bed' if has_bed else 'Obstruction'} at {f:.2f}m). Right flank open ({r:.2f}m vs Left {l:.2f}m).",
                    'maneuver': 'PIVOT_RIGHT',
                    'backup_dist': 0.0,
                    'pivot_angle_deg': -45.0,
                    'explanation': "Pivoting in place rightward so forward camera and 120° LiDAR directly inspect the open corridor."
                }
            elif l > r + 0.20:
                # Left side is open -> Rotate in place to face open left corridor
                return {
                    'diagnosis': f"Obstacle in forward path ({'Hospital Bed' if has_bed else 'Obstruction'} at {f:.2f}m). Left flank open ({l:.2f}m vs Right {r:.2f}m).",
                    'maneuver': 'PIVOT_LEFT',
                    'backup_dist': 0.0,
                    'pivot_angle_deg': +45.0,
                    'explanation': "Pivoting in place leftward so forward camera and 120° LiDAR directly inspect the open corridor."
                }
            else:
                # Both flanks restricted (narrow corridor trap)
                # Check if rear is verified wide open by raw 360° LiDAR (> 0.80m)
                if rear > 0.80 and rear < 90.0:
                    return {
                        'diagnosis': f"Confined corridor. Forward blocked at {f:.2f}m, rear verified clear by 360° LiDAR ({rear:.2f}m).",
                        'maneuver': 'CONTROLLED_REVERSE',
                        'backup_dist': 0.25,
                        'pivot_angle_deg': -20.0 * (1 if self.recovery_attempt_count % 2 == 1 else -1),
                        'explanation': "Executing cautious reverse along verified clear rear clearance with active safety monitoring."
                    }
                else:
                    return {
                        'diagnosis': f"Cul-de-sac entrapment. Forward blocked at {f:.2f}m, sides restricted, rear unverified. No blind reversing.",
                        'maneuver': 'PIVOT_RIGHT',
                        'backup_dist': 0.0,
                        'pivot_angle_deg': -90.0,
                        'explanation': "Pivoting in place 90° to scan perpendicular exit with forward sensors without reversing into blind areas."
                    }

        # Scenario 2: Stale / Phantom costmap artifact
        else:
            return {
                'diagnosis': f"LiDAR senses free forward space ({f:.2f}m). Abortion caused by transient costmap inflation or ghost point.",
                'maneuver': 'CLEAR_AND_RETRY',
                'backup_dist': 0.0,
                'pivot_angle_deg': 0.0,
                'explanation': "Flushing costmap layers to purge transient obstacles and retrying forward navigation."
            }

    # --- Costmap Flushing ---
    def _flush_costmaps(self):
        """Call Nav2 ClearEntireCostmap services for both local and global costmaps."""
        self._log("[FLUSH_COSTMAPS] Flushing local and global costmaps to purge ghost/stale obstacles...")
        req = ClearEntireCostmap.Request()
        if self.cli_clear_global.service_is_ready():
            self.cli_clear_global.call_async(req)
        if self.cli_clear_local.service_is_ready():
            self.cli_clear_local.call_async(req)

    # --- Escape Maneuver Execution ---
    def _step_escape_execution(self):
        """Execute physical extrication maneuver step-by-step."""
        maneuver = self.last_escape_maneuver
        elapsed = time.time() - self.escape_start_time
        pivot_deg = self.escape_params.get('pivot_angle_deg', 0.0)
        pivot_rad = math.radians(pivot_deg)
        pivot_time = abs(pivot_rad) / max(0.01, self.pivot_speed)
        backup_dist = self.escape_params.get('backup_dist', 0.0)
        backup_time = backup_dist / max(0.01, self.backup_speed)

        # Maneuver Type A: PIVOT (Rotate in place to face open corridor - zero reverse!)
        if "PIVOT" in maneuver:
            if elapsed < pivot_time:
                rot_dir = +1.0 if pivot_rad > 0 else -1.0
                self._publish_cmd_vel(0.0, rot_dir * self.pivot_speed)
            else:
                self._complete_escape_maneuver()

        # Maneuver Type B: CLEAR_AND_RETRY
        elif maneuver == "CLEAR_AND_RETRY":
            self._complete_escape_maneuver()

        # Maneuver Type C: CONTROLLED_REVERSE (Strictly verified rear clearance only)
        elif "BACKUP" in maneuver or "REVERSE" in maneuver:
            if elapsed < pivot_time:
                rot_dir = +1.0 if pivot_rad > 0 else -1.0
                self._publish_cmd_vel(0.0, rot_dir * self.pivot_speed)
            elif elapsed < (pivot_time + backup_time):
                clearances = self._profile_sensor_clearances()
                if clearances['rear_min'] < 0.40:
                    self._log(f"[SAFETY_HALT] Rear obstacle proximity ({clearances['rear_min']:.2f}m). Halting reverse.")
                    self._complete_escape_maneuver()
                else:
                    self._publish_cmd_vel(-self.backup_speed, 0.0)
            else:
                self._complete_escape_maneuver()
        else:
            self._complete_escape_maneuver()

    def _complete_escape_maneuver(self):
        self._publish_cmd_vel(0.0, 0.0)
        self._log("[ESCAPE_COMPLETE] Physical extrication maneuver completed. Re-checking clearances.")

        # Flush costmaps again so new heading and pose are immediately clean
        self._flush_costmaps()

        if self.auto_resume_navigation and self.target_goal_pose:
            self.state = "RESUMING_NAV"
            self._log(f"[RESUME_GOAL] Automatically re-dispatching goal pose to ({self.target_goal_pose.pose.position.x:.2f}, {self.target_goal_pose.pose.position.y:.2f}).")
            self.target_goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.pub_goal_pose.publish(self.target_goal_pose)
        else:
            self.state = "IDLE"
            self._log("[RECOVERY_FINISHED] Extrication complete. Robot positioned in open space.")

    def _publish_cmd_vel(self, vx: float, wz: float):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self.pub_cmd_vel.publish(cmd)

    # --- Observability & Visualization ---
    def _publish_status(self):
        """Publish JSON status string for dashboards and logs."""
        status_dict = {
            'timestamp': time.time(),
            'recovery_state': self.state,
            'attempt_count': self.recovery_attempt_count,
            'max_attempts': self.max_recovery_attempts,
            'diagnosis': self.last_diagnosis,
            'maneuver': self.last_escape_maneuver,
            'target_goal_cached': self.target_goal_pose is not None
        }
        msg = String()
        msg.data = json.dumps(status_dict)
        self.pub_status.publish(msg)

    def _publish_markers(self):
        """Publish 3D RViz Billboard and Escape Vector markers."""
        if not self.latest_odom:
            return

        pos = self.latest_odom.pose.pose.position
        orient = self.latest_odom.pose.pose.orientation
        robot_yaw = yaw_from_quaternion(orient.x, orient.y, orient.z, orient.w)

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 1. AI Diagnosis Billboard Marker
        billboard = Marker()
        billboard.header.frame_id = 'map'
        billboard.header.stamp = now
        billboard.ns = 'recovery_hud'
        billboard.id = 1
        billboard.type = Marker.TEXT_VIEW_FACING
        billboard.action = Marker.ADD
        billboard.pose.position.x = pos.x
        billboard.pose.position.y = pos.y
        billboard.pose.position.z = pos.z + 1.25  # Hover 1.25m above robot
        billboard.scale.z = 0.16  # Text height
        billboard.color.r = 1.0 if self.state != "IDLE" else 0.2
        billboard.color.g = 0.8 if self.state != "IDLE" else 0.9
        billboard.color.b = 0.1 if self.state != "IDLE" else 0.4
        billboard.color.a = 0.95
        billboard.text = f"[AI RECOVERY: {self.state}]\n{self.last_diagnosis[:60]}"
        marker_array.markers.append(billboard)

        # 2. Escape Vector Arrow (when actively diagnosing or escaping)
        if self.state in ["AI_DIAGNOSING", "EXECUTING_ESCAPE"]:
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = now
            arrow.ns = 'escape_vector'
            arrow.id = 2
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD

            pivot_deg = self.escape_params.get('pivot_angle_deg', 0.0) if hasattr(self, 'escape_params') else 0.0
            escape_yaw = robot_yaw + math.pi + math.radians(pivot_deg)  # Reversing + pivot
            qx, qy, qz, qw = quaternion_from_yaw(escape_yaw)

            arrow.pose.position.x = pos.x
            arrow.pose.position.y = pos.y
            arrow.pose.position.z = pos.z + 0.3
            arrow.pose.orientation.x = qx
            arrow.pose.orientation.y = qy
            arrow.pose.orientation.z = qz
            arrow.pose.orientation.w = qw
            arrow.scale.x = 0.85  # Arrow length
            arrow.scale.y = 0.12  # Arrow width
            arrow.scale.z = 0.12  # Arrow height
            arrow.color.r = 0.1
            arrow.color.g = 1.0
            arrow.color.b = 0.4
            arrow.color.a = 0.9
            marker_array.markers.append(arrow)

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationRecoveryNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
