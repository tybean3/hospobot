import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        self.mode = 'manual' # default
        self.pub = self.create_publisher(Twist, '/cmd_vel_out', 10)
        self.sub_man = self.create_subscription(Twist, '/cmd_vel_manual', self.man_cb, 10)
        self.sub_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_cb, 10)
        self.sub_mode = self.create_subscription(String, '/control_mode', self.mode_cb, 10)
        self.get_logger().info('CmdVelMux started. Mode: manual')
        
    def mode_cb(self, msg):
        if msg.data in ['manual', 'auto']:
            self.mode = msg.data
            self.get_logger().info(f'Mode switched to {self.mode}')
            
    def man_cb(self, msg):
        if self.mode == 'manual':
            self.pub.publish(msg)
            
    def nav_cb(self, msg):
        if self.mode == 'auto':
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
