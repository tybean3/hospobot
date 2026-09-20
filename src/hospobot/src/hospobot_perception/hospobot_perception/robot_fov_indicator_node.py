#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class RobotFovIndicatorNode(Node):
    def __init__(self):
        super().__init__('robot_fov_indicator_node')
        
        self.declare_parameter('flip_angles', False)
        self.declare_parameter('cone_range', 2.5) # Cone length in meters
        self.declare_parameter('robot_radius', 0.26) # Robot radius in meters
        self.declare_parameter('laser_frame', 'laser_frame')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 5.0) # Hz
        
        self.flip_angles = self.get_parameter('flip_angles').get_parameter_value().bool_value
        self.cone_range = self.get_parameter('cone_range').get_parameter_value().double_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.laser_frame = self.get_parameter('laser_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        self.marker_pub = self.create_publisher(MarkerArray, '/robot_fov_markers', 10)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.publish_markers)
        
        self.get_logger().info('Robot FOV Indicator Node initialized. Publishing to /robot_fov_markers')

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        flip = self.get_parameter('flip_angles').get_parameter_value().bool_value
        cone_r = self.get_parameter('cone_range').get_parameter_value().double_value
        robot_r = self.get_parameter('robot_radius').get_parameter_value().double_value

        marker_array = MarkerArray()
        marker_id = 0

        # ==========================================
        # 1. ROBOT POSE: Circular Footprint Disc & Ring
        # ==========================================
        # 1a. Base Disc (Semi-transparent circle)
        disc_marker = Marker()
        disc_marker.header.stamp = now
        disc_marker.header.frame_id = self.base_frame
        disc_marker.ns = 'robot_pose_circle'
        disc_marker.id = marker_id
        marker_id += 1
        disc_marker.type = Marker.CYLINDER
        disc_marker.action = Marker.ADD
        disc_marker.pose.position.x = 0.0
        disc_marker.pose.position.y = 0.0
        disc_marker.pose.position.z = 0.005
        disc_marker.pose.orientation.w = 1.0
        disc_marker.scale.x = robot_r * 2.0
        disc_marker.scale.y = robot_r * 2.0
        disc_marker.scale.z = 0.01
        disc_marker.color.r = 0.0
        disc_marker.color.g = 0.6
        disc_marker.color.b = 0.9
        disc_marker.color.a = 0.35
        marker_array.markers.append(disc_marker)

        # 1b. Outer Ring (Crisp boundary perimeter)
        ring_marker = Marker()
        ring_marker.header.stamp = now
        ring_marker.header.frame_id = self.base_frame
        ring_marker.ns = 'robot_pose_ring'
        ring_marker.id = marker_id
        marker_id += 1
        ring_marker.type = Marker.LINE_STRIP
        ring_marker.action = Marker.ADD
        ring_marker.pose.orientation.w = 1.0
        ring_marker.scale.x = 0.02  # Line thickness
        ring_marker.color.r = 0.0
        ring_marker.color.g = 0.95
        ring_marker.color.b = 1.0
        ring_marker.color.a = 0.95

        num_ring_pts = 64
        for i in range(num_ring_pts + 1):
            theta = 2.0 * math.pi * i / num_ring_pts
            p = Point()
            p.x = robot_r * math.cos(theta)
            p.y = robot_r * math.sin(theta)
            p.z = 0.01
            ring_marker.points.append(p)
        marker_array.markers.append(ring_marker)

        # 1c. Forward Heading Arrow
        arrow_marker = Marker()
        arrow_marker.header.stamp = now
        arrow_marker.header.frame_id = self.base_frame
        arrow_marker.ns = 'robot_heading_arrow'
        arrow_marker.id = marker_id
        marker_id += 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.pose.orientation.w = 1.0
        p_start = Point()
        p_start.x = 0.0
        p_start.y = 0.0
        p_start.z = 0.02
        p_end = Point()
        p_end.x = robot_r + 0.12  # Extends slightly past perimeter forward
        p_end.y = 0.0
        p_end.z = 0.02
        arrow_marker.points = [p_start, p_end]
        arrow_marker.scale.x = 0.035  # Shaft diameter
        arrow_marker.scale.y = 0.07   # Head diameter
        arrow_marker.scale.z = 0.07   # Head length
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.8
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0
        marker_array.markers.append(arrow_marker)

        # ==========================================
        # 2. THREE 2D LIDAR FOV CONES
        # ==========================================
        # Sector definitions based on laser_filter_node.py:
        # Standard mode (Lidar mounted backwards: 0°=Rear, 90°=Left, 180°=Front, 270°=Right):
        # 1. Left FOV: 57° to 123° (span 66°, centered at 90°)
        # 2. Front FOV: 147° to 213° (span 66°, centered at 180°)
        # 3. Right FOV: 237° to 303° (span 66°, centered at 270°)
        if not flip:
            sectors = [
                {
                    'name': 'FRONT',
                    'label': 'FRONT FOV (66°)',
                    'start_deg': 147.0,
                    'end_deg': 213.0,
                    'color_fill': (0.1, 0.9, 0.4, 0.28),    # Emerald green
                    'color_line': (0.1, 1.0, 0.4, 0.90),
                },
                {
                    'name': 'LEFT',
                    'label': 'LEFT FOV (66°)',
                    'start_deg': 57.0,
                    'end_deg': 123.0,
                    'color_fill': (0.0, 0.7, 1.0, 0.28),    # Cyan / Blue
                    'color_line': (0.0, 0.85, 1.0, 0.90),
                },
                {
                    'name': 'RIGHT',
                    'label': 'RIGHT FOV (66°)',
                    'start_deg': 237.0,
                    'end_deg': 303.0,
                    'color_fill': (1.0, 0.6, 0.1, 0.28),    # Vibrant Amber
                    'color_line': (1.0, 0.7, 0.1, 0.90),
                },
            ]
        else:
            # Flipped mode (Operator stands in front, 180° blocked):
            sectors = [
                {
                    'name': 'REAR',
                    'label': 'REAR FOV (66°)',
                    'start_deg': 327.0,
                    'end_deg': 393.0, # 327° to 33° mod 360
                    'color_fill': (0.1, 0.9, 0.4, 0.28),
                    'color_line': (0.1, 1.0, 0.4, 0.90),
                },
                {
                    'name': 'LEFT',
                    'label': 'LEFT FOV (66°)',
                    'start_deg': 57.0,
                    'end_deg': 123.0,
                    'color_fill': (0.0, 0.7, 1.0, 0.28),
                    'color_line': (0.0, 0.85, 1.0, 0.90),
                },
                {
                    'name': 'RIGHT',
                    'label': 'RIGHT FOV (66°)',
                    'start_deg': 237.0,
                    'end_deg': 303.0,
                    'color_fill': (1.0, 0.6, 0.1, 0.28),
                    'color_line': (1.0, 0.7, 0.1, 0.90),
                },
            ]

        for sec in sectors:
            s_deg = sec['start_deg']
            e_deg = sec['end_deg']
            c_fill = sec['color_fill']
            c_line = sec['color_line']
            mid_deg = (s_deg + e_deg) / 2.0

            # 2a. Filled Sector Triangle Fan
            fan_marker = Marker()
            fan_marker.header.stamp = now
            fan_marker.header.frame_id = self.laser_frame
            fan_marker.ns = f'fov_cone_{sec["name"].lower()}'
            fan_marker.id = marker_id
            marker_id += 1
            fan_marker.type = Marker.TRIANGLE_LIST
            fan_marker.action = Marker.ADD
            fan_marker.pose.orientation.w = 1.0
            fan_marker.scale.x = 1.0
            fan_marker.scale.y = 1.0
            fan_marker.scale.z = 1.0
            fan_marker.color.r = c_fill[0]
            fan_marker.color.g = c_fill[1]
            fan_marker.color.b = c_fill[2]
            fan_marker.color.a = c_fill[3]

            deg_step = 1.5
            current_deg = s_deg
            origin = Point()
            origin.x = 0.0
            origin.y = 0.0
            origin.z = 0.01

            while current_deg < e_deg:
                next_deg = min(current_deg + deg_step, e_deg)
                r1 = math.radians(current_deg)
                r2 = math.radians(next_deg)

                p1 = Point()
                p1.x = cone_r * math.cos(r1)
                p1.y = cone_r * math.sin(r1)
                p1.z = 0.01

                p2 = Point()
                p2.x = cone_r * math.cos(r2)
                p2.y = cone_r * math.sin(r2)
                p2.z = 0.01

                fan_marker.points.append(origin)
                fan_marker.points.append(p1)
                fan_marker.points.append(p2)

                current_deg = next_deg

            marker_array.markers.append(fan_marker)

            # 2b. Sharp Boundary Lines (Edge Rays + Arc)
            border_marker = Marker()
            border_marker.header.stamp = now
            border_marker.header.frame_id = self.laser_frame
            border_marker.ns = f'fov_border_{sec["name"].lower()}'
            border_marker.id = marker_id
            marker_id += 1
            border_marker.type = Marker.LINE_STRIP
            border_marker.action = Marker.ADD
            border_marker.pose.orientation.w = 1.0
            border_marker.scale.x = 0.018  # 1.8cm line thickness
            border_marker.color.r = c_line[0]
            border_marker.color.g = c_line[1]
            border_marker.color.b = c_line[2]
            border_marker.color.a = c_line[3]

            # Start from origin -> start angle point
            border_marker.points.append(origin)
            current_deg = s_deg
            while current_deg <= e_deg:
                rad = math.radians(current_deg)
                p = Point()
                p.x = cone_r * math.cos(rad)
                p.y = cone_r * math.sin(rad)
                p.z = 0.015
                border_marker.points.append(p)
                current_deg += deg_step
            # Return to origin
            border_marker.points.append(origin)
            marker_array.markers.append(border_marker)

            # 2c. Text Label at outer midpoint
            text_marker = Marker()
            text_marker.header.stamp = now
            text_marker.header.frame_id = self.laser_frame
            text_marker.ns = f'fov_text_{sec["name"].lower()}'
            text_marker.id = marker_id
            marker_id += 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_rad = math.radians(mid_deg)
            text_marker.pose.position.x = (cone_r + 0.18) * math.cos(text_rad)
            text_marker.pose.position.y = (cone_r + 0.18) * math.sin(text_rad)
            text_marker.pose.position.z = 0.05
            text_marker.pose.orientation.w = 1.0
            text_marker.text = sec['label']
            text_marker.scale.z = 0.14  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.95
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RobotFovIndicatorNode()
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
