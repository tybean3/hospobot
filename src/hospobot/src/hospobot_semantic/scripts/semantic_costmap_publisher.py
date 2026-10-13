#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan
from hospobot_interfaces.msg import SemanticObjectArray
from geometry_msgs.msg import PointStamped, Vector3Stamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import time
import math

class SemanticCostmapPublisher(Node):
    def __init__(self):
        super().__init__('semantic_costmap_publisher')
        self.declare_parameter('frame_id', 'base_footprint')
        
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # TF2 setup for transforming objects from odom to base_footprint
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.publisher_ = self.create_publisher(OccupancyGrid, '/semantic_network/dense_costmap', qos)
        
        # Publisher for the HMI screen
        self.hmi_pub_ = self.create_publisher(String, '/hmi/passing_intention', 10)
        
        # Subscribe to real semantic objects from the perception pipeline
        self.sub_ = self.create_subscription(SemanticObjectArray, '/semantic/tracked_objects', self.semantic_callback, 10)
        
        # Subscribe to depth scan to track depth obstacles with 0.5s clearing guarantee
        self.depth_sub_ = self.create_subscription(LaserScan, '/oakd/depth_scan', self.depth_scan_callback, 10)
        self.depth_obstacles = [] # list of (x, y, timestamp)
        
        self.latest_objects = []
        self.tracked_humans = []
        self.next_human_id = 0
        
        # Intention state with track-locked hysteresis to eliminate flip-flopping ("hallway dance")
        self.track_intentions = {}
        self.active_intention = "CLEAR"
        self.active_intention_expiry = 0.0
        
        # Fast 10 Hz update rate for responsive costmap updates
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pub_count = 0
        self.get_logger().info(f'Responsive Predictive Semantic Costmap Node started (frame: {self.frame_id}, 10 Hz, 8m horizon).')

    def depth_scan_callback(self, msg: LaserScan):
        now = time.time()
        new_obs = []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= min(msg.range_max, 4.2):
                ox = r * math.cos(angle)
                oy = r * math.sin(angle)
                new_obs.append((ox, oy, now))
            angle += msg.angle_increment

        # Merge with existing depth obstacles, keeping only those <= 0.50s old
        merged = [obs for obs in self.depth_obstacles if (now - obs[2]) <= 0.50]
        merged.extend(new_obs)
        # Deduplicate spatially (5cm binning)
        deduped = {}
        for ox, oy, t in merged:
            key = (round(ox, 2), round(oy, 2))
            if key not in deduped or t > deduped[key][2]:
                deduped[key] = (ox, oy, t)
        self.depth_obstacles = list(deduped.values())

    def semantic_callback(self, msg):
        current_humans = []
        other_objects = []
        now = time.time()
        
        for obj in msg.objects:
            if obj.class_name == 'HUMAN':
                current_humans.append({
                    'id': obj.tracking_id,
                    'pos': [obj.position.x, obj.position.y],
                    'vel': [obj.velocity.x, obj.velocity.y],
                    'last_time': now
                })
            else:
                other_objects.append({
                    'class_name': obj.class_name,
                    'position': obj.position,
                    'frame_id': msg.header.frame_id,
                    'last_time': now
                })
                
        self.tracked_humans = current_humans
        self.latest_objects = other_objects
        self.latest_msg_frame_id = msg.header.frame_id

    def timer_callback(self):
        now_time = self.get_clock().now()
        now_msg = now_time.to_msg()
        now_sec = time.time()

        # Prune any human, object, or depth obstacle older than 0.5s so past objects leave cleanly
        self.tracked_humans = [h for h in self.tracked_humans if (now_sec - h.get('last_time', now_sec)) <= 0.50]
        self.latest_objects = [o for o in self.latest_objects if (now_sec - o.get('last_time', now_sec)) <= 0.50]
        self.depth_obstacles = [obs for obs in self.depth_obstacles if (now_sec - obs[2]) <= 0.50]

        # 8m x 8m local costmap representation
        resolution = 0.05
        width = 160   # 8.0m wide (columns along X)
        height = 160  # 8.0m long (rows along Y)
        origin_x = -2.5 # -2.5m behind, +5.5m in front of robot
        origin_y = -4.0 # -4.0m to +4.0m lateral width
        
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = now_msg
        msg.header.frame_id = self.frame_id
        
        msg.info.map_load_time = now_msg
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        costmap_data = np.zeros((height, width), dtype=np.int8)
        detected_intention = "CLEAR"
        active_intention = self.active_intention if (now_sec < self.active_intention_expiry) else "CLEAR"
        
        try:
            if hasattr(self, 'latest_msg_frame_id'):
                if self.latest_msg_frame_id == self.frame_id:
                    robot_yaw = 0.0
                    transform = None
                else:
                    transform = self.tf_buffer.lookup_transform(self.frame_id, self.latest_msg_frame_id, rclpy.time.Time())
                    q = transform.transform.rotation
                    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                    robot_yaw = math.atan2(siny_cosp, cosy_cosp)

                # 1. EVALUATE GENUINE COLLISION & CROSSING PATHS
                # Robot forward speed (nominal cruise 0.35 m/s)
                v_robot = 0.35
                collision_humans = []

                for th in self.tracked_humans:
                    if transform is not None:
                        pt_world = PointStamped()
                        pt_world.header.frame_id = self.latest_msg_frame_id
                        pt_world.point.x = th['pos'][0]
                        pt_world.point.y = th['pos'][1]
                        pt_world.point.z = 0.0
                        pt_local = tf2_geometry_msgs.do_transform_point(pt_world, transform)
                        local_x = pt_local.point.x
                        local_y = pt_local.point.y
                        
                        # Accurately transform velocity vector from world frame to local base_footprint frame
                        v_world = Vector3Stamped()
                        v_world.header.frame_id = self.latest_msg_frame_id
                        v_world.vector.x = th['vel'][0]
                        v_world.vector.y = th['vel'][1]
                        v_world.vector.z = 0.0
                        v_local = tf2_geometry_msgs.do_transform_vector3(v_world, transform)
                        local_vx = v_local.vector.x
                        local_vy = v_local.vector.y
                    else:
                        local_x = th['pos'][0]
                        local_y = th['pos'][1]
                        local_vx = th['vel'][0]
                        local_vy = th['vel'][1]
                        
                    speed = math.hypot(local_vx, local_vy)
                    th['local_x'] = local_x
                    th['local_y'] = local_y
                    th['local_vx'] = local_vx
                    th['local_vy'] = local_vy
                    th['speed'] = speed
                    
                    # If human is already behind or level with the robot (x < 0.25m), they have passed!
                    if local_x < 0.25:
                        continue

                    # STRICT COLLISION & CROSSING DETECTION:
                    # Only signal if human is:
                    #   A) Actively approaching head-on (local_vx < -0.15 m/s) in corridor envelope (|y| <= 1.0m)
                    #   B) Predicted to cross paths with the robot based on velocity vectors
                    # Never signal for bystanders, standing people, or people walking away (local_vx > 0)
                    is_collision_course = False
                    urgency = 999.0

                    # Condition A: Oncoming pedestrian in corridor
                    if 0.5 <= local_x <= 5.5 and abs(local_y) <= 1.0 and local_vx < -0.15:
                        is_collision_course = True
                        urgency = local_x / max(0.1, abs(local_vx) + v_robot)
                        th['threat_type'] = 'oncoming'

                    # Condition B: Crossing pedestrian moving toward robot's centerline
                    elif 0.5 <= local_x <= 5.0 and abs(local_vy) >= 0.18 and (local_y * local_vy < 0):
                        t_cross = -local_y / local_vy
                        if 0.3 <= t_cross <= 4.5:
                            x_h_cross = local_x + local_vx * t_cross
                            x_r_cross = v_robot * t_cross
                            if abs(x_h_cross - x_r_cross) < 0.85 and x_h_cross > 0.3:
                                is_collision_course = True
                                urgency = t_cross
                                th['threat_type'] = 'crossing'

                    if is_collision_course:
                        th['urgency'] = urgency
                        collision_humans.append(th)

                # 2. INTENTION ARBITRATION (FOR SCREEN / HMI GUIDANCE ONLY)
                if not hasattr(self, 'session_locked_intention'):
                    self.session_locked_intention = "CLEAR"
                    self.session_lock_last_seen = 0.0
                    self.consecutive_flip_votes = 0

                if collision_humans:
                    # Pick the most urgent threat (closest arrival / crossing time)
                    collision_humans.sort(key=lambda h: (h['urgency'], h['local_x']))
                    primary = collision_humans[0]
                    ly = primary['local_y']
                    lvy = primary.get('local_vy', 0.0)

                    # Symmetrically evaluate human position relative to robot centerline:
                    # If human is to the robot's right (ly < -0.10m), open space is left -> PASS_LEFT
                    # If human is to the robot's left (ly > 0.10m), open space is right -> PASS_RIGHT
                    # If human is centered (|ly| <= 0.10m), check lateral velocity or hold previous intention
                    if ly < -0.10:
                        target_intention = "PASS_LEFT"
                    elif ly > 0.10:
                        target_intention = "PASS_RIGHT"
                    else:
                        if lvy < -0.10:
                            target_intention = "PASS_LEFT"
                        elif lvy > 0.10:
                            target_intention = "PASS_RIGHT"
                        elif self.session_locked_intention in ["PASS_RIGHT", "PASS_LEFT"]:
                            target_intention = self.session_locked_intention
                        else:
                            target_intention = "PASS_RIGHT"

                    # Responsive 3-frame debounce (~0.3s) to prevent visual flicker
                    if self.session_locked_intention != target_intention:
                        self.consecutive_flip_votes += 1
                        if self.consecutive_flip_votes >= 3:
                            self.session_locked_intention = target_intention
                            self.consecutive_flip_votes = 0
                    else:
                        self.consecutive_flip_votes = 0

                    self.session_lock_last_seen = now_sec
                    active_intention = self.session_locked_intention
                else:
                    # Responsive clearing within 0.5s once human clears or collision course ceases
                    if now_sec - self.session_lock_last_seen > 0.50:
                        self.session_locked_intention = "CLEAR"
                        self.consecutive_flip_votes = 0
                    active_intention = self.session_locked_intention

                # 2.5 RENDER 3D DEPTH OBSTACLES (CLEARS 0.5s AFTER LEAVING CAMERA FRAME)
                for ox, oy, t in self.depth_obstacles:
                    if (origin_x <= ox < origin_x + width * resolution) and (origin_y <= oy < origin_y + height * resolution):
                        col = int((ox - origin_x) / resolution)
                        row = int((oy - origin_y) / resolution)
                        if 0 <= row < height and 0 <= col < width:
                            costmap_data[row, col] = max(costmap_data[row, col], 99)

                # 3. RENDER STATIC OBJECTS ON COSTMAP
                for obj in self.latest_objects:
                    if transform is not None:
                        pt_odom = PointStamped()
                        pt_odom.header.frame_id = obj['frame_id']
                        pt_odom.point = obj['position']
                        pt_local = tf2_geometry_msgs.do_transform_point(pt_odom, transform)
                        x, y = pt_local.point.x, pt_local.point.y
                    else:
                        x, y = obj['position'].x, obj['position'].y
                    
                    if (origin_x <= x < origin_x + width * resolution) and (origin_y <= y < origin_y + height * resolution):
                        col = int((x - origin_x) / resolution)
                        row = int((y - origin_y) / resolution)
                        radius_cells = int(0.4 / resolution)
                        for r in range(max(0, row - radius_cells), min(height, row + radius_cells + 1)):
                            for c in range(max(0, col - radius_cells), min(width, col + radius_cells + 1)):
                                dist = math.hypot(r - row, c - col)
                                if dist <= radius_cells:
                                    costmap_data[r, c] = max(costmap_data[r, c], 90 if dist < (0.2 / resolution) else 50)

                # 4. RENDER TRACKED HUMANS TRUTHFULLY (SYMMETRIC PHYSICAL FOOTPRINT)
                for th in self.tracked_humans:
                    if 'local_x' not in th:
                        continue
                        
                    x = th['local_x']
                    y = th['local_y']
                    
                    # If human is already behind the robot rear bumper (x < -0.35m), do not render!
                    if x < -0.35:
                        continue
                    
                    if (origin_x <= x < origin_x + width * resolution) and (origin_y <= y < origin_y + height * resolution):
                        col = int((x - origin_x) / resolution)
                        row = int((y - origin_y) / resolution)
                        
                        # Human physical footprint (radius 0.25m, realistic human torso width)
                        base_radius = 0.25
                        base_cells = int(base_radius / resolution)
                        
                        r_min = max(0, row - base_cells)
                        r_max = min(height, row + base_cells + 1)
                        c_min = max(0, col - base_cells)
                        c_max = min(width, col + base_cells + 1)
                        
                        for r in range(r_min, r_max):
                            for c in range(c_min, c_max):
                                d = math.hypot((c - col) * resolution, (r - row) * resolution)
                                if d <= base_radius:
                                    cost = 75 if d < 0.15 else 50
                                    costmap_data[r, c] = max(costmap_data[r, c], cost)

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"TF Error: {ex}", throttle_duration_sec=3.0)

        # Publish the OccupancyGrid costmap
        msg.data = costmap_data.flatten().tolist()
        self.publisher_.publish(msg)
        
        # Publish HMI Passing Intention for the screen app
        msg_str = String()
        msg_str.data = active_intention
        self.hmi_pub_.publish(msg_str)
        
        self.pub_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = SemanticCostmapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
