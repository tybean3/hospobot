import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
class TestRx(Node):
    def __init__(self):
        super().__init__('test_rx')
        self.sub = self.create_subscription(Frame, 'can/rx', self.cb, 10)
    def cb(self, msg):
        print(msg)
rclpy.init()
node = TestRx()
rclpy.spin(node)
