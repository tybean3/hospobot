#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32

class FootprintPublisherNode(Node):
    def __init__(self):
        super().__init__('footprint_publisher_node')
        self.pub = self.create_publisher(PolygonStamped, '/robot_footprint', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)
        
        # 16-point rounded square with a nose pointing forward (X-axis)
        pts_array = [
            [0.3000, 0.0000],  # Nose
            [0.2362, 0.1497],  # Front Left
            [0.2247, 0.1930],
            [0.1930, 0.2247],
            [0.1497, 0.2362],
            [-0.1497, 0.2362], # Rear Left
            [-0.1930, 0.2247],
            [-0.2247, 0.1930],
            [-0.2362, 0.1497],
            [-0.2362, -0.1497],# Rear Right
            [-0.2247, -0.1930],
            [-0.1930, -0.2247],
            [-0.1497, -0.2362],
            [0.1497, -0.2362], # Front Right
            [0.1930, -0.2247],
            [0.2247, -0.1930],
            [0.2362, -0.1497]
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
