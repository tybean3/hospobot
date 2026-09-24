import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        
        self.declare_parameter('timeout', 0.5)
        self.timeout = self.get_parameter('timeout').value
        
        self.declare_parameter('invert_controls', False)
        self.declare_parameter('max_linear_speed', 0.0)
        self.declare_parameter('max_angular_speed', 0.0)
        
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
        # Nav2 final collision-checked, smoothed output publishes to /cmd_vel
        self.sub_nav = self.create_subscription(Twist, '/cmd_vel', self.nav_cb, 10)
        
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
        
        # Check timeout
        now = self.get_clock().now().nanoseconds / 1e9
        is_recent = (now - self.last_time[mode_str]) < self.timeout
        if not is_recent:
            return False

        # Only check zero deadband for manual joysticks (web/ps5) to prevent stick drift from blocking lower priorities.
        # Nav2 output should NEVER be chopped by a deadband as smooth deceleration/acceleration commands
        # must pass through without stuttering.
        if mode_str in ['web', 'ps5']:
            is_zero = abs(msg.linear.x) < 0.05 and abs(msg.angular.z) < 0.05
            return not is_zero
        
        return True

    def timer_cb(self):
        source = None
        
        if self.mode == 'auto_priority':
            if self.is_active('web'): source = 'web'
            elif self.is_active('ps5'): source = 'ps5'
            elif self.is_active('nav'): source = 'nav'
        elif self.mode in ['web', 'ps5', 'nav']:
            if self.is_active(self.mode): source = self.mode
                
        if source is not None:
            msg = self.last_msg[source]
            invert = self.get_parameter('invert_controls').get_parameter_value().bool_value
            max_lin = self.get_parameter('max_linear_speed').get_parameter_value().double_value
            max_ang = self.get_parameter('max_angular_speed').get_parameter_value().double_value

            out_msg = Twist()
            if invert and source in ['web', 'ps5', 'cmd_vel']:
                out_msg.linear.x = -msg.linear.x
                out_msg.linear.y = -msg.linear.y
                out_msg.linear.z = msg.linear.z
                out_msg.angular.x = msg.angular.x
                out_msg.angular.y = msg.angular.y
                out_msg.angular.z = msg.angular.z
            else:
                out_msg.linear.x = msg.linear.x
                out_msg.linear.y = msg.linear.y
                out_msg.linear.z = msg.linear.z
                out_msg.angular.x = msg.angular.x
                out_msg.angular.y = msg.angular.y
                out_msg.angular.z = msg.angular.z

            # Clamp velocities if limits are specified
            if max_lin > 0.0:
                out_msg.linear.x = max(-max_lin, min(max_lin, out_msg.linear.x))
            if max_ang > 0.0:
                out_msg.angular.z = max(-max_ang, min(max_ang, out_msg.angular.z))

            self.pub.publish(out_msg)
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
