import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        
        self.declare_parameter('timeout', 0.5)
        self.timeout = self.get_parameter('timeout').value
        
        self.pub = self.create_publisher(Twist, '/cmd_vel_out', 10)
        
        self.mode = 'auto_priority'
        
        self.last_msg = {
            'web': None,
            'ps5': None,
            'nav': None
        }
        self.last_time = {
            'web': 0.0,
            'ps5': 0.0,
            'nav': 0.0
        }
        
        self.sub_mode = self.create_subscription(String, '/control_mode', self.mode_cb, 10)
        
        self.sub_web = self.create_subscription(Twist, '/cmd_vel_web', self.web_cb, 10)
        self.sub_ps5 = self.create_subscription(Twist, '/cmd_vel_ps5', self.ps5_cb, 10)
        self.sub_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_cb, 10)
        
        self.timer = self.create_timer(0.05, self.timer_cb) # 20Hz loop
        self.get_logger().info('Priority CmdVelMux started. Mode: auto_priority')

    def mode_cb(self, msg):
        valid_modes = ['auto_priority', 'web', 'ps5', 'nav']
        if msg.data in valid_modes:
            self.mode = msg.data
            self.get_logger().info(f'CmdVelMux Mode switched to: {self.mode}')

    def web_cb(self, msg):
        self.last_msg['web'] = msg
        self.last_time['web'] = self.get_clock().now().nanoseconds / 1e9

    def ps5_cb(self, msg):
        self.last_msg['ps5'] = msg
        self.last_time['ps5'] = self.get_clock().now().nanoseconds / 1e9

    def nav_cb(self, msg):
        self.last_msg['nav'] = msg
        self.last_time['nav'] = self.get_clock().now().nanoseconds / 1e9

    def is_active(self, mode_str):
        msg = self.last_msg[mode_str]
        if msg is None: return False
        
        # Consider it inactive if the message is effectively zero (controller drift / released)
        # Note: If they intentionally command zero, it will fall through to lower priorities 
        # or the final stop_msg, which still successfully stops the robot!
        is_zero = abs(msg.linear.x) < 0.05 and abs(msg.angular.z) < 0.05
        
        # Also check timeout
        now = self.get_clock().now().nanoseconds / 1e9
        is_recent = (now - self.last_time[mode_str]) < self.timeout
        
        return is_recent and not is_zero

    def timer_cb(self):
        source = None
        
        if self.mode == 'auto_priority':
            if self.is_active('web'): source = 'web'
            elif self.is_active('ps5'): source = 'ps5'
            elif self.is_active('nav'): source = 'nav'
        elif self.mode in ['web', 'ps5', 'nav']:
            if self.is_active(self.mode): source = self.mode
                
        if source is not None:
            self.pub.publish(self.last_msg[source])
        else:
            stop_msg = Twist()
            self.pub.publish(stop_msg)

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
