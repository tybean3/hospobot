#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import time

class SemanticCostmapPublisher(Node):
    def __init__(self):
        super().__init__('semantic_costmap_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/semantic_network/dense_costmap', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Semantic Costmap Publisher Node started.')

    def timer_callback(self):
        msg = OccupancyGrid()
        
        # Populate the header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Populate the grid info
        msg.info.resolution = 0.05  # 5 cm per cell
        msg.info.width = 100
        msg.info.height = 100
        # Origin is typically relative to the frame (bottom-left corner of the grid)
        msg.info.origin.position.x = -2.5
        msg.info.origin.position.y = -2.5
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Generate a dummy costmap (e.g., simulating a neural network output)
        # 0 = free, 100 = occupied, -1 = unknown
        # In a real scenario, this would come from model inference.
        costmap_data = np.zeros((100, 100), dtype=np.int8)
        
        # Simulate some obstacles (e.g., in front of the robot)
        costmap_data[70:80, 40:60] = 100
        
        msg.data = costmap_data.flatten().tolist()
        
        self.publisher_.publish(msg)

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
