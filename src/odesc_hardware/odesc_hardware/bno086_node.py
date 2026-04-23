#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

try:
    import board
    import busio
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_ROTATION_VECTOR
    HAS_BNO08X = True
except ImportError:
    HAS_BNO08X = False

class BNO086Node(Node):
    def __init__(self):
        super().__init__('bno086_node')
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('frequency', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('alpha', 0.2) # Low pass filter alpha
        
        self.frame_id = self.get_parameter('frame_id').value
        self.alpha = self.get_parameter('alpha').value
        freq = self.get_parameter('frequency').value
        
        # Filter state
        self.filtered_accel = [0.0, 0.0, 0.0]
        self.filtered_gyro = [0.0, 0.0, 0.0]
        
        if HAS_BNO08X:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                self.bno = BNO08X_I2C(i2c, address=0x4b)
                self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
                self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
                self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                self.get_logger().info("BNO086 initialized on I2C.")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize BNO086: {e}")
                self.bno = None
        else:
            self.get_logger().warn("adafruit-circuitpython-bno08x not found. Running in mock mode.")
            self.bno = None
            
        self.timer = self.create_timer(1.0 / freq, self.timer_cb)

    def low_pass(self, current, previous):
        return [
            self.alpha * current[0] + (1 - self.alpha) * previous[0],
            self.alpha * current[1] + (1 - self.alpha) * previous[1],
            self.alpha * current[2] + (1 - self.alpha) * previous[2]
        ]

    def timer_cb(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        if self.bno:
            try:
                accel_x, accel_y, accel_z = self.bno.acceleration
                gyro_x, gyro_y, gyro_z = self.bno.gyro
                quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
                
                self.filtered_accel = self.low_pass([accel_x, accel_y, accel_z], self.filtered_accel)
                self.filtered_gyro = self.low_pass([gyro_x, gyro_y, gyro_z], self.filtered_gyro)
                
                msg.linear_acceleration.x = self.filtered_accel[0]
                msg.linear_acceleration.y = self.filtered_accel[1]
                msg.linear_acceleration.z = self.filtered_accel[2]
                
                msg.angular_velocity.x = self.filtered_gyro[0]
                msg.angular_velocity.y = self.filtered_gyro[1]
                msg.angular_velocity.z = self.filtered_gyro[2]
                
                msg.orientation.x = quat_i
                msg.orientation.y = quat_j
                msg.orientation.z = quat_k
                msg.orientation.w = quat_real
                
            except Exception as e:
                self.get_logger().warn(f"Error reading BNO086: {e}")
                return
        else:
            msg.orientation.w = 1.0
            
        # Standard Covariances for EKF
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BNO086Node()
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
