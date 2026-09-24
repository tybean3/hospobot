#!/usr/bin/env python3
"""
Nav Visualizer Node for Hospobot RViz
Publishes rich, high-visibility 3D decision markers to /nav_markers:
  1. Current Robot Pose & Dynamic Safety Footprint (475mm Square in base_footprint)
  2. Live Motion Intent Vector (Cmd Vel Arrow / In-Place Spin Ring in base_footprint)
  3. Lookahead Target Tracking Beacon & Connector Line (in map frame)
  4. Path Tangent Heading & Heading Error Arc (in HUD billboard)
  5. Nearest Obstacle Clearance Ray & Warning Sphere (in base_footprint)
  6. Live Multi-Line Decision HUD Billboard (floating above robot in base_footprint)
  7. Active Goal Pose (3D Beacon Pin, Landing Zone Rings, Heading Arrow in map frame)
  8. Global (A*) and Local (MPPI) Paths (in map / odom frame)

By publishing each marker in its natural reference frame (map objects in 'map',
robot objects in 'base_footprint') with stamp=0, RViz natively and perfectly projects
everything regardless of whether Fixed Frame is 'map' or 'base_footprint'.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import math
import time

def euler_from_quaternion(x, y, z, w):
    """Convert quaternion into euler roll, pitch, yaw."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def quaternion_from_yaw(yaw):
    """Create quaternion tuple (x, y, z, w) from yaw angle in radians."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

class NavVisualizerNode(Node):
    def __init__(self):
        super().__init__('nav_visualizer_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 10.0) # Hz

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Robot dimensions: 475mm square
        self.robot_half_size = 0.2375 # 0.475m / 2

        # TF listener for continuous real-time base pose in map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State caches
        self.current_pose_map = None  # (x, y, z, yaw) in map frame
        self.goal_pose = None         # (x, y, z, yaw, frame_id)
        self.global_path = None       # list of (x, y, z)
        self.local_path = None        # list of (x, y, z)
        self.global_path_frame = self.map_frame
        self.local_path_frame = self.map_frame

        # Motion & Sensor decision caches
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.last_cmd_time = 0.0

        self.min_obstacle_dist = 99.0
        self.min_obstacle_pt_base = None  # (x, y, z) in base_footprint

        # Subscriptions
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_cb,
            10
        )
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            10
        )
        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_cb,
            10
        )
        self.sub_global_plan = self.create_subscription(
            Path,
            '/plan',
            self.global_plan_cb,
            10
        )
        self.sub_local_plan = self.create_subscription(
            Path,
            '/transformed_global_plan',
            self.local_plan_cb,
            10
        )
        self.sub_mppi_plan = self.create_subscription(
            Path,
            '/local_plan',
            self.local_plan_cb,
            10
        )
        self.sub_cmd_vel_nav = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_cb,
            10
        )
        self.sub_cmd_vel_out = self.create_subscription(
            Twist,
            '/cmd_vel_out',
            self.cmd_vel_cb,
            10
        )
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            10
        )

        # Re-stamp RViz 2D Pose Estimates with stamp=0 so AMCL accepts remote laptop clicks
        self.sub_initialpose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_cb,
            10
        )
        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # Publisher for navigation markers
        self.marker_pub = self.create_publisher(MarkerArray, '/nav_markers', 10)

        # Timer loop for continuous publishing
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.publish_all_markers)

        self.get_logger().info('Nav Visualizer Node initialized. Publishing Decision Markers to /nav_markers')

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = euler_from_quaternion(o.x, o.y, o.z, o.w)
        self.current_pose_map = (p.x, p.y, p.z, yaw)

    def odom_cb(self, msg: Odometry):
        if self.current_pose_map is None:
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            yaw = euler_from_quaternion(o.x, o.y, o.z, o.w)
            self.current_pose_map = (p.x, p.y, p.z, yaw)

    def cmd_vel_cb(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z
        self.last_cmd_time = time.time()

    def scan_cb(self, msg: LaserScan):
        min_d = 99.0
        min_pt = None
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and not math.isnan(r) and not math.isinf(r):
                if r < min_d:
                    min_d = r
                    lx = r * math.cos(angle)
                    ly = r * math.sin(angle)
                    min_pt = (lx, ly, 0.0)
            angle += msg.angle_increment
        self.min_obstacle_dist = min_d
        self.min_obstacle_pt_base = min_pt

    def initialpose_cb(self, msg: PoseWithCovarianceStamped):
        if getattr(self, '_publishing_initialpose', False):
            return
        # If timestamp is non-zero, this came from remote RViz (e.g. MacBook) with clock skew.
        # Re-stamp with stamp=0 so AMCL looks up latest TF without future extrapolation errors.
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            p = msg.pose.pose.position
            self.get_logger().info(f'Synchronizing 2D Pose Estimate at ({p.x:.2f}, {p.y:.2f}) with stamp=0 for AMCL')
            synced = msg
            synced.header.stamp.sec = 0
            synced.header.stamp.nanosec = 0
            self._publishing_initialpose = True
            try:
                self.pub_initialpose.publish(synced)
            finally:
                self._publishing_initialpose = False

    def goal_cb(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        yaw = euler_from_quaternion(o.x, o.y, o.z, o.w)
        frame = msg.header.frame_id if msg.header.frame_id else self.map_frame
        self.goal_pose = (p.x, p.y, p.z, yaw, frame)
        self.get_logger().info(f'Visualizer received Goal Pose at ({p.x:.2f}, {p.y:.2f}) Frame: {frame}')

    def global_plan_cb(self, msg: Path):
        self.global_path = [(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) for pose in msg.poses]
        self.global_path_frame = msg.header.frame_id if msg.header.frame_id else self.map_frame

    def local_plan_cb(self, msg: Path):
        self.local_path = [(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) for pose in msg.poses]
        self.local_path_frame = msg.header.frame_id if msg.header.frame_id else self.map_frame

    def get_live_base_pose_in_map(self):
        """Query TF for the absolute real-time transform from base_footprint to map."""
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            rot = t.transform.rotation
            yaw = euler_from_quaternion(rot.x, rot.y, rot.z, rot.w)
            return (x, y, z, yaw)
        except Exception:
            return self.current_pose_map

    def publish_all_markers(self):
        markers = MarkerArray()
        # Stamp zero ensures RViz always uses the latest available TF transform
        stamp_zero = rclpy.time.Time().to_msg()

        # 1. Obtain current robot pose in map
        live_pose = self.get_live_base_pose_in_map()
        if live_pose is None:
            live_pose = (0.0, 0.0, 0.0, 0.0)

        map_x, map_y, map_z, map_yaw = live_pose

        # ----------------------------------------------------
        # Calculate Navigation Decisions, Lookahead & Errors
        # ----------------------------------------------------
        lookahead_pt_map = None
        heading_err_deg = 0.0
        cross_track_err = 0.0
        dist_to_goal = 0.0

        if self.global_path and len(self.global_path) > 1:
            # Find closest point on global path to robot
            closest_idx = 0
            min_dist_sq = 999999.0
            for idx, (px, py, _) in enumerate(self.global_path):
                d2 = (px - map_x)**2 + (py - map_y)**2
                if d2 < min_dist_sq:
                    min_dist_sq = d2
                    closest_idx = idx

            # Lookahead: ~0.8m ahead along path (or at least 8 waypoints)
            lookahead_idx = min(closest_idx + 12, len(self.global_path) - 1)
            lx, ly, lz = self.global_path[lookahead_idx]
            lookahead_pt_map = (lx, ly, lz)

            # Compute path tangent heading around lookahead point
            prev_idx = max(0, lookahead_idx - 3)
            next_idx = min(len(self.global_path) - 1, lookahead_idx + 3)
            if next_idx > prev_idx:
                path_dx = self.global_path[next_idx][0] - self.global_path[prev_idx][0]
                path_dy = self.global_path[next_idx][1] - self.global_path[prev_idx][1]
                path_heading_map = math.atan2(path_dy, path_dx)
                heading_err = normalize_angle(path_heading_map - map_yaw)
                heading_err_deg = math.degrees(heading_err)

            # Cross-track error
            cross_track_err = math.sqrt(min_dist_sq)

            # Distance to goal along path
            for idx in range(closest_idx, len(self.global_path) - 1):
                p1 = self.global_path[idx]
                p2 = self.global_path[idx + 1]
                dist_to_goal += math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        elif self.goal_pose is not None:
            gx, gy = self.goal_pose[0], self.goal_pose[1]
            dist_to_goal = math.hypot(gx - map_x, gy - map_y)

        # Classify decision state
        now_ts = time.time()
        is_cmd_recent = (now_ts - self.last_cmd_time) < 1.0
        cur_vx = self.cmd_vx if is_cmd_recent else 0.0
        cur_wz = self.cmd_wz if is_cmd_recent else 0.0

        if self.goal_pose is None:
            decision_state = "IDLE (NO ACTIVE GOAL)"
            state_color = (0.7, 0.7, 0.7)
        elif dist_to_goal < 0.35 and abs(cur_vx) < 0.05 and abs(cur_wz) < 0.05:
            decision_state = "GOAL REACHED"
            state_color = (0.2, 1.0, 0.4)
        elif abs(cur_vx) < 0.04 and abs(cur_wz) >= 0.05:
            turn_dir = "LEFT / CCW" if cur_wz > 0 else "RIGHT / CW"
            decision_state = f"ROTATING IN-PLACE [{turn_dir}]"
            state_color = (1.0, 0.65, 0.0) # Amber
        elif cur_vx >= 0.04:
            decision_state = f"TRACKING PATH FORWARD ({cur_vx:.2f} m/s)"
            state_color = (0.0, 1.0, 0.4) # Emerald
        elif self.min_obstacle_dist < 0.35:
            decision_state = f"OBSTACLE AVOIDANCE ({self.min_obstacle_dist:.2f}m)"
            state_color = (1.0, 0.2, 0.2) # Red
        else:
            decision_state = "PLANNING / PREPARING MOVE"
            state_color = (0.3, 0.8, 1.0) # Cyan

        # ====================================================
        # GROUP A: ROBOT-CENTRIC MARKERS (Frame: base_footprint)
        # ====================================================
        # 1a. Directional 3D Heading Arrow (Cyan) along +X
        arrow = Marker()
        arrow.header.stamp = stamp_zero
        arrow.header.frame_id = self.base_frame
        arrow.ns = 'current_pose_arrow'
        arrow.id = 100
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = 0.0
        arrow.pose.position.y = 0.0
        arrow.pose.position.z = 0.05
        arrow.pose.orientation.w = 1.0
        arrow.scale.x = 0.45
        arrow.scale.y = 0.08
        arrow.scale.z = 0.08
        arrow.color.r = 0.0
        arrow.color.g = 0.9
        arrow.color.b = 1.0
        arrow.color.a = 0.95
        markers.markers.append(arrow)

        # 1b. Base Footprint Square (475mm square, Color shifts with clearance)
        sq_marker = Marker()
        sq_marker.header.stamp = stamp_zero
        sq_marker.header.frame_id = self.base_frame
        sq_marker.ns = 'current_pose_footprint_square'
        sq_marker.id = 101
        sq_marker.type = Marker.LINE_STRIP
        sq_marker.action = Marker.ADD
        sq_marker.pose.orientation.w = 1.0
        sq_marker.scale.x = 0.03

        # Color: Red if <0.28m, Amber if <0.45m, Cyan/Green if clear
        if self.min_obstacle_dist < 0.28:
            sq_marker.color.r, sq_marker.color.g, sq_marker.color.b = 1.0, 0.1, 0.1
        elif self.min_obstacle_dist < 0.45:
            sq_marker.color.r, sq_marker.color.g, sq_marker.color.b = 1.0, 0.65, 0.0
        else:
            sq_marker.color.r, sq_marker.color.g, sq_marker.color.b = 0.0, 1.0, 0.8
        sq_marker.color.a = 0.95

        half_w = self.robot_half_size # 0.2375m
        corners = [
            (half_w, half_w),
            (-half_w, half_w),
            (-half_w, -half_w),
            (half_w, -half_w),
            (half_w, half_w)
        ]
        for c_x, c_y in corners:
            sq_marker.points.append(Point(x=c_x, y=c_y, z=0.02))
        markers.markers.append(sq_marker)

        # 2a. Linear Velocity Command Arrow
        if cur_vx > 0.03:
            vel_arrow = Marker()
            vel_arrow.header.stamp = stamp_zero
            vel_arrow.header.frame_id = self.base_frame
            vel_arrow.ns = 'cmd_vel_arrow'
            vel_arrow.id = 110
            vel_arrow.type = Marker.ARROW
            vel_arrow.action = Marker.ADD
            vel_arrow.scale.x = max(0.25, cur_vx * 1.5)
            vel_arrow.scale.y = 0.07
            vel_arrow.scale.z = 0.07
            vel_arrow.color.r = 0.0
            vel_arrow.color.g = 1.0
            vel_arrow.color.b = 0.2 # Neon Green
            vel_arrow.color.a = 0.95

            steer_angle = math.atan2(cur_wz * 0.3, cur_vx)
            vel_q = quaternion_from_yaw(steer_angle)
            vel_arrow.pose.position.x = half_w
            vel_arrow.pose.position.y = 0.0
            vel_arrow.pose.position.z = 0.05
            vel_arrow.pose.orientation.x = vel_q[0]
            vel_arrow.pose.orientation.y = vel_q[1]
            vel_arrow.pose.orientation.z = vel_q[2]
            vel_arrow.pose.orientation.w = vel_q[3]
            markers.markers.append(vel_arrow)

        # 2b. In-Place Rotation Indicator (Spin Ring)
        if abs(cur_vx) <= 0.03 and abs(cur_wz) >= 0.05:
            spin_ring = Marker()
            spin_ring.header.stamp = stamp_zero
            spin_ring.header.frame_id = self.base_frame
            spin_ring.ns = 'cmd_spin_ring'
            spin_ring.id = 111
            spin_ring.type = Marker.LINE_STRIP
            spin_ring.action = Marker.ADD
            spin_ring.pose.orientation.w = 1.0
            spin_ring.scale.x = 0.035
            spin_ring.color.r = 1.0
            spin_ring.color.g = 0.65
            spin_ring.color.b = 0.0 # Amber
            spin_ring.color.a = 0.95

            r_arc = 0.35
            start_th = 0.0 if cur_wz > 0 else math.pi
            span = 1.5 * math.pi if cur_wz > 0 else -1.5 * math.pi
            num_arc_pts = 24
            for i in range(num_arc_pts + 1):
                th = start_th + span * (i / num_arc_pts)
                spin_ring.points.append(Point(x=r_arc * math.cos(th), y=r_arc * math.sin(th), z=0.08))
            markers.markers.append(spin_ring)

        # 3. Nearest Obstacle Clearance Ray
        if self.min_obstacle_pt_base is not None and self.min_obstacle_dist < 1.2:
            ox, oy, oz = self.min_obstacle_pt_base
            obs_ray = Marker()
            obs_ray.header.stamp = stamp_zero
            obs_ray.header.frame_id = self.base_frame
            obs_ray.ns = 'obstacle_clearance_ray'
            obs_ray.id = 130
            obs_ray.type = Marker.LINE_STRIP
            obs_ray.action = Marker.ADD
            obs_ray.pose.orientation.w = 1.0
            obs_ray.scale.x = 0.025

            is_danger = (self.min_obstacle_dist < 0.35)
            obs_ray.color.r = 1.0
            obs_ray.color.g = 0.1 if is_danger else 0.7
            obs_ray.color.b = 0.1
            obs_ray.color.a = 0.85

            obs_ray.points.append(Point(x=0.0, y=0.0, z=0.04))
            obs_ray.points.append(Point(x=ox, y=oy, z=oz + 0.04))
            markers.markers.append(obs_ray)

            # Obstacle Warning Sphere
            obs_sph = Marker()
            obs_sph.header.stamp = stamp_zero
            obs_sph.header.frame_id = self.base_frame
            obs_sph.ns = 'obstacle_point'
            obs_sph.id = 131
            obs_sph.type = Marker.SPHERE
            obs_sph.action = Marker.ADD
            obs_sph.pose.position.x = ox
            obs_sph.pose.position.y = oy
            obs_sph.pose.position.z = oz + 0.04
            obs_sph.pose.orientation.w = 1.0
            obs_sph.scale.x = 0.08
            obs_sph.scale.y = 0.08
            obs_sph.scale.z = 0.08
            obs_sph.color = obs_ray.color
            markers.markers.append(obs_sph)

        # 4. Multi-Line Decision HUD Billboard (Floating directly above robot)
        hud = Marker()
        hud.header.stamp = stamp_zero
        hud.header.frame_id = self.base_frame
        hud.ns = 'decision_hud_billboard'
        hud.id = 140
        hud.type = Marker.TEXT_VIEW_FACING
        hud.action = Marker.ADD
        hud.pose.position.x = 0.0
        hud.pose.position.y = 0.0
        hud.pose.position.z = 0.75
        hud.pose.orientation.w = 1.0
        hud.scale.z = 0.10

        hud.text = (
            f"=== HOSPOBOT DECISION MONITOR ===\n"
            f"STATE: {decision_state}\n"
            f"CMD:   Vx: {cur_vx:+.2f} m/s  |  Wz: {cur_wz:+.2f} rad/s ({math.degrees(cur_wz):+.0f}°/s)\n"
            f"ALIGN: Error: {heading_err_deg:+.1f}°  |  Cross-Track: {cross_track_err:.2f}m\n"
            f"OBS:   Min Clearance: {self.min_obstacle_dist:.2f}m  |  475mm Square Footprint\n"
            f"GOAL:  Dist Remaining: {dist_to_goal:.1f}m"
        )
        hud.color.r = state_color[0]
        hud.color.g = state_color[1]
        hud.color.b = state_color[2]
        hud.color.a = 0.95
        markers.markers.append(hud)

        # ====================================================
        # GROUP B: MAP-LEVEL MARKERS (Frame: map)
        # ====================================================
        # 5. Goal Pose Visualizers
        if self.goal_pose is not None:
            gx, gy, gz, gyaw, gframe = self.goal_pose
            gq = quaternion_from_yaw(gyaw)

            # Goal Pole
            pole = Marker()
            pole.header.stamp = stamp_zero
            pole.header.frame_id = gframe
            pole.ns = 'goal_beacon_pole'
            pole.id = 200
            pole.type = Marker.CYLINDER
            pole.action = Marker.ADD
            pole.pose.position.x = gx
            pole.pose.position.y = gy
            pole.pose.position.z = gz + 0.30
            pole.pose.orientation.w = 1.0
            pole.scale.x = 0.03
            pole.scale.y = 0.03
            pole.scale.z = 0.60
            pole.color.r, pole.color.g, pole.color.b, pole.color.a = 1.0, 0.70, 0.0, 0.95
            markers.markers.append(pole)

            # Goal Sphere
            beacon_sphere = Marker()
            beacon_sphere.header.stamp = stamp_zero
            beacon_sphere.header.frame_id = gframe
            beacon_sphere.ns = 'goal_beacon_sphere'
            beacon_sphere.id = 201
            beacon_sphere.type = Marker.SPHERE
            beacon_sphere.action = Marker.ADD
            beacon_sphere.pose.position.x = gx
            beacon_sphere.pose.position.y = gy
            beacon_sphere.pose.position.z = gz + 0.62
            beacon_sphere.pose.orientation.w = 1.0
            beacon_sphere.scale.x, beacon_sphere.scale.y, beacon_sphere.scale.z = 0.14, 0.14, 0.14
            beacon_sphere.color.r, beacon_sphere.color.g, beacon_sphere.color.b, beacon_sphere.color.a = 1.0, 0.85, 0.1, 1.0
            markers.markers.append(beacon_sphere)

            # Goal Heading Arrow
            g_arrow = Marker()
            g_arrow.header.stamp = stamp_zero
            g_arrow.header.frame_id = gframe
            g_arrow.ns = 'goal_heading_arrow'
            g_arrow.id = 202
            g_arrow.type = Marker.ARROW
            g_arrow.action = Marker.ADD
            g_arrow.pose.position.x = gx
            g_arrow.pose.position.y = gy
            g_arrow.pose.position.z = gz + 0.04
            g_arrow.pose.orientation.x = gq[0]
            g_arrow.pose.orientation.y = gq[1]
            g_arrow.pose.orientation.z = gq[2]
            g_arrow.pose.orientation.w = gq[3]
            g_arrow.scale.x, g_arrow.scale.y, g_arrow.scale.z = 0.45, 0.09, 0.09
            g_arrow.color.r, g_arrow.color.g, g_arrow.color.b, g_arrow.color.a = 1.0, 0.40, 0.0, 0.95
            markers.markers.append(g_arrow)

            # Target Landing Rings
            for r_idx, radius in enumerate([0.22, 0.40]):
                target_ring = Marker()
                target_ring.header.stamp = stamp_zero
                target_ring.header.frame_id = gframe
                target_ring.ns = f'goal_target_ring_{r_idx}'
                target_ring.id = 203 + r_idx
                target_ring.type = Marker.LINE_STRIP
                target_ring.action = Marker.ADD
                target_ring.pose.orientation.w = 1.0
                target_ring.scale.x = 0.02
                target_ring.color.r, target_ring.color.g, target_ring.color.b = 1.0, 0.75, 0.0
                target_ring.color.a = 0.85 if r_idx == 0 else 0.45

                num_pts = 32
                for i in range(num_pts + 1):
                    theta = 2.0 * math.pi * i / num_pts
                    target_ring.points.append(Point(x=gx + radius * math.cos(theta), y=gy + radius * math.sin(theta), z=gz + 0.015))
                markers.markers.append(target_ring)

        # 6. Global Path Plan (Emerald Green)
        if self.global_path and len(self.global_path) > 1:
            path_line = Marker()
            path_line.header.stamp = stamp_zero
            path_line.header.frame_id = self.global_path_frame
            path_line.ns = 'global_path_line'
            path_line.id = 300
            path_line.type = Marker.LINE_STRIP
            path_line.action = Marker.ADD
            path_line.pose.orientation.w = 1.0
            path_line.scale.x = 0.045
            path_line.color.r, path_line.color.g, path_line.color.b, path_line.color.a = 0.0, 1.0, 0.35, 0.95

            waypoint_spheres = Marker()
            waypoint_spheres.header.stamp = stamp_zero
            waypoint_spheres.header.frame_id = self.global_path_frame
            waypoint_spheres.ns = 'global_path_waypoints'
            waypoint_spheres.id = 301
            waypoint_spheres.type = Marker.SPHERE_LIST
            waypoint_spheres.action = Marker.ADD
            waypoint_spheres.pose.orientation.w = 1.0
            waypoint_spheres.scale.x, waypoint_spheres.scale.y, waypoint_spheres.scale.z = 0.06, 0.06, 0.06
            waypoint_spheres.color.r, waypoint_spheres.color.g, waypoint_spheres.color.b, waypoint_spheres.color.a = 0.4, 1.0, 0.6, 0.85

            for idx, (px, py, pz) in enumerate(self.global_path):
                pt = Point(x=px, y=py, z=pz + 0.025)
                path_line.points.append(pt)
                if idx % 8 == 0 or idx == len(self.global_path) - 1:
                    waypoint_spheres.points.append(pt)

            markers.markers.append(path_line)
            markers.markers.append(waypoint_spheres)

        # 7. Lookahead Target Beacon & Connector Line
        if lookahead_pt_map is not None:
            lx, ly, lz = lookahead_pt_map

            # Lookahead Target Sphere (Electric Magenta)
            lk_sphere = Marker()
            lk_sphere.header.stamp = stamp_zero
            lk_sphere.header.frame_id = self.global_path_frame
            lk_sphere.ns = 'lookahead_target_sphere'
            lk_sphere.id = 120
            lk_sphere.type = Marker.SPHERE
            lk_sphere.action = Marker.ADD
            lk_sphere.pose.position.x = lx
            lk_sphere.pose.position.y = ly
            lk_sphere.pose.position.z = lz + 0.08
            lk_sphere.pose.orientation.w = 1.0
            lk_sphere.scale.x, lk_sphere.scale.y, lk_sphere.scale.z = 0.14, 0.14, 0.14
            lk_sphere.color.r, lk_sphere.color.g, lk_sphere.color.b, lk_sphere.color.a = 0.9, 0.1, 1.0, 0.95
            markers.markers.append(lk_sphere)

            # Connector Line from Robot to Lookahead Target
            lk_line = Marker()
            lk_line.header.stamp = stamp_zero
            lk_line.header.frame_id = self.map_frame
            lk_line.ns = 'lookahead_connector_line'
            lk_line.id = 121
            lk_line.type = Marker.LINE_STRIP
            lk_line.action = Marker.ADD
            lk_line.pose.orientation.w = 1.0
            lk_line.scale.x = 0.02
            lk_line.color.r, lk_line.color.g, lk_line.color.b, lk_line.color.a = 0.9, 0.1, 1.0, 0.65
            lk_line.points.append(Point(x=map_x, y=map_y, z=map_z + 0.05))
            lk_line.points.append(Point(x=lx, y=ly, z=lz + 0.08))
            markers.markers.append(lk_line)

        # 8. Local Controller Rollout Plan (Vibrant Amber)
        if self.local_path and len(self.local_path) > 1:
            local_line = Marker()
            local_line.header.stamp = stamp_zero
            local_line.header.frame_id = self.local_path_frame
            local_line.ns = 'local_path_line'
            local_line.id = 400
            local_line.type = Marker.LINE_STRIP
            local_line.action = Marker.ADD
            local_line.pose.orientation.w = 1.0
            local_line.scale.x = 0.035
            local_line.color.r, local_line.color.g, local_line.color.b, local_line.color.a = 1.0, 0.65, 0.0, 0.90

            for px, py, pz in self.local_path:
                local_line.points.append(Point(x=px, y=py, z=pz + 0.035))

            markers.markers.append(local_line)

        # Publish marker array
        if markers.markers:
            self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = NavVisualizerNode()
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
