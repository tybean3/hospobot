import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class TestEmpty(Node):
    def __init__(self):
        super().__init__('test_empty')
        self.sub = self.create_subscription(Empty, 'test_empty', self.cb, 10)
        self.pub = self.create_publisher(Empty, 'test_empty', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)
    def timer_cb(self):
        self.pub.publish(Empty())
    def cb(self, msg):
        print("Received Empty!")

rclpy.init()
node = TestEmpty()
rclpy.spin(node)
