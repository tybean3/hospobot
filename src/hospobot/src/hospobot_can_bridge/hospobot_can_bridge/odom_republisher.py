import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

class OdomRepublisher(Node):
    def __init__(self):
        super().__init__('odom_republisher')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            Odometry,
            '/oak/vio/odometry',
            self.listener_callback,
            qos)
        self.publisher_ = self.create_publisher(Odometry, '/oak/vio/odometry_fixed', 10)
        self.count = 0
        self.get_logger().info('OdomRepublisher started, subscribing to /oak/vio/odometry')

    def listener_callback(self, msg):
        self.count += 1
        new_msg = msg
        new_msg.child_frame_id = 'oakd_frame'
        
        # If covariance is all zeros (which breaks EKF), add a default covariance
        if sum(new_msg.pose.covariance) == 0.0:
            new_msg.pose.covariance[0] = 0.05
            new_msg.pose.covariance[7] = 0.05
            new_msg.pose.covariance[14] = 0.05
            new_msg.pose.covariance[21] = 0.05
            new_msg.pose.covariance[28] = 0.05
            new_msg.pose.covariance[35] = 0.05
            
        if sum(new_msg.twist.covariance) == 0.0:
            new_msg.twist.covariance[0] = 0.05
            new_msg.twist.covariance[7] = 0.05
            new_msg.twist.covariance[14] = 0.05
            new_msg.twist.covariance[21] = 0.05
            new_msg.twist.covariance[28] = 0.05
            new_msg.twist.covariance[35] = 0.05
            
        self.publisher_.publish(new_msg)
        if self.count % 100 == 1:
            self.get_logger().info(f'Republished VIO odom #{self.count}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

