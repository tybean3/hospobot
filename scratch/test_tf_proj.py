import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class TestTF(Node):
    def __init__(self):
        super().__init__('test_tf')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.check_tf)

    def check_tf(self):
        try:
            # Create a point 1 meter directly in front of the camera
            p_cam = PointStamped()
            p_cam.header.frame_id = 'oak_rgb_camera_optical_frame'
            p_cam.header.stamp = self.get_clock().now().to_msg()
            p_cam.point.x = 0.0
            p_cam.point.y = 0.0
            p_cam.point.z = 1.0  # 1 meter forward along optical axis

            # Transform to base_footprint
            p_base = self.tf_buffer.transform(p_cam, 'base_footprint', timeout=rclpy.duration.Duration(seconds=1.0))
            self.get_logger().info(f'Point (0, 0, 1) in camera frame is at ({p_base.point.x:.3f}, {p_base.point.y:.3f}, {p_base.point.z:.3f}) in base_footprint')
            rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f'TF Error: {e}')

def main():
    rclpy.init()
    node = TestTF()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
