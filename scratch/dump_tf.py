import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import time

class TFTreeNode(Node):
    def __init__(self):
        super().__init__('tf_tree_dumper')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

def main():
    rclpy.init()
    node = TFTreeNode()
    time.sleep(2)
    rclpy.spin_once(node, timeout_sec=2.0)
    
    print(node.tf_buffer.all_frames_as_yaml())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
