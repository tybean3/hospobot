#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32

class FootprintPublisherNode(Node):
    def __init__(self):
        super().__init__('footprint_publisher_node')
        self.pub = self.create_publisher(PolygonStamped, '/robot_footprint', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)
        
        # 475mm Square Footprint (half-width = 0.2375m)
        pts_array = [
            [0.2375, 0.2375],   # Front Left
            [-0.2375, 0.2375],  # Rear Left
            [-0.2375, -0.2375], # Rear Right
            [0.2375, -0.2375]   # Front Right
        ]        
        self.poly = PolygonStamped()
        self.poly.header.frame_id = 'base_footprint'
        for pt in pts_array:
            p = Point32()
            p.x = pt[0]
            p.y = pt[1]
            p.z = 0.0
            self.poly.polygon.points.append(p)
            
        self.get_logger().info('Publishing /robot_footprint for RViz')

    def timer_cb(self):
        self.poly.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.poly)

def main(args=None):
    rclpy.init(args=args)
    node = FootprintPublisherNode()
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
