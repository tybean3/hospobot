import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

def main():
    rclpy.init()
    node = rclpy.create_node('init_pose_pub')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.orientation.w = 1.0
    
    # wait for subscriber
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        
    pub.publish(msg)
    node.get_logger().info('Initial pose published!')
    
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
