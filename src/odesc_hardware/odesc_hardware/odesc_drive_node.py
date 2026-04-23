#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String
from tf2_ros import TransformBroadcaster
import serial
import json
import math
import time

class OdescDriveNode(Node):
    def __init__(self):
        super().__init__('odesc_drive_node')
        
        # Parameters
        self.declare_parameter('left_serial_port', '/dev/ttyUSB0')
        self.declare_parameter('right_serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('track_width', 0.4)
        self.declare_parameter('wheel_radius', 0.08)
        self.declare_parameter('odom_freq', 50.0) # Hz
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('run_calibration_on_startup', False)
        
        self.left_port = self.get_parameter('left_serial_port').value
        self.right_port = self.get_parameter('right_serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.odom_freq = self.get_parameter('odom_freq').value
        self.invert_left = self.get_parameter('invert_left').value
        self.invert_right = self.get_parameter('invert_right').value
        self.run_calibration = self.get_parameter('run_calibration_on_startup').value
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        self.last_cmd_l = None
        self.last_cmd_r = None
        
        # Serial connections
        self.left_serial = None
        self.right_serial = None
        self.left_calibrated = False
        self.right_calibrated = False
        self.connect_serial(calibrate=True)
        
        # ROS
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_out', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.calib_l_sub = self.create_subscription(Empty, '/odesc_hardware/calibrate_left', self.calib_left_cb, 10)
        self.calib_r_sub = self.create_subscription(Empty, '/odesc_hardware/calibrate_right', self.calib_right_cb, 10)
        self.reboot_l_sub = self.create_subscription(Empty, '/odesc_hardware/reboot_left', self.reboot_left_cb, 10)
        self.reboot_r_sub = self.create_subscription(Empty, '/odesc_hardware/reboot_right', self.reboot_right_cb, 10)
        self.state_l_sub = self.create_subscription(String, '/odesc_hardware/state_left', self.state_left_cb, 10)
        self.state_r_sub = self.create_subscription(String, '/odesc_hardware/state_right', self.state_right_cb, 10)
        self.settings_sub = self.create_subscription(String, '/odesc_hardware/settings', self.settings_cb, 10)
        self.status_pub = self.create_publisher(String, '/odesc_hardware/motor_status', 10)
        
        # Diagnostic timer and state
        self.diag_counter = 0
        self.motor_diag = {
            "left": {"voltage": "0.0", "current": "0.0", "error": "None", "state": "Unknown"},
            "right": {"voltage": "0.0", "current": "0.0", "error": "None", "state": "Unknown"}
        }
        
        # Timer
        self.timer = self.create_timer(1.0 / self.odom_freq, self.odom_loop)

        self.get_logger().info('ODESC Drive Node initialized.')

    def calib_left_cb(self, msg):
        self.get_logger().info("Manual calibration requested for LEFT motor.")
        # Temporarily enable run_calibration for this run
        old_val = self.run_calibration
        self.run_calibration = True
        try:
            self.calibrate_motor(self.left_serial, "LEFT")
        except Exception as e:
            self.get_logger().error(f"Manual calibration failed: {e}")
        finally:
            self.run_calibration = old_val

    def calib_right_cb(self, msg):
        self.get_logger().info("Manual calibration requested for RIGHT motor.")
        old_val = self.run_calibration
        self.run_calibration = True
        try:
            self.calibrate_motor(self.right_serial, "RIGHT")
        except Exception as e:
            self.get_logger().error(f"Manual calibration failed: {e}")
        finally:
            self.run_calibration = old_val

    def reboot_left_cb(self, msg):
        self.get_logger().info("Reboot requested for LEFT motor.")
        self.reboot_motor(self.left_serial, "LEFT")
        
    def reboot_right_cb(self, msg):
        self.get_logger().info("Reboot requested for RIGHT motor.")
        self.reboot_motor(self.right_serial, "RIGHT")

    def set_motor_state(self, ser, name, state_str):
        if ser is None or not ser.is_open:
            return
        state_val = b"1" if state_str == "IDLE" else b"8"
        self.get_logger().info(f"[{name}] Changing state to {state_str}")
        if state_str == "IDLE":
            ser.write(b"v 0 0.0\n")
            time.sleep(0.05)
        ser.write(b"w axis0.requested_state " + state_val + b"\n")

    def state_left_cb(self, msg):
        self.set_motor_state(self.left_serial, "LEFT", msg.data)
        
    def state_right_cb(self, msg):
        self.set_motor_state(self.right_serial, "RIGHT", msg.data)

    def settings_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if 'invert_left' in data:
                self.invert_left = data['invert_left']
            if 'invert_right' in data:
                self.invert_right = data['invert_right']
            self.get_logger().info(f"Updated settings: Invert Left={self.invert_left}, Invert Right={self.invert_right}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse settings: {e}")

    def reboot_motor(self, ser, name):
        if ser is None or not ser.is_open:
            return
        self.get_logger().info(f"[{name}] Sending sr (Reboot) command...")
        ser.write(b"sr\n")
        time.sleep(1.0)
        # Clear buffer
        while ser.in_waiting > 0:
            ser.read(ser.in_waiting)
        # Re-initialize to State 8
        self.get_logger().info(f"[{name}] Re-initializing to Closed Loop Control (State 8)...")
        ser.write(b"w axis0.requested_state 8\n")
        time.sleep(0.5)

    def calibrate_motor(self, ser, name):
        if ser is None or not ser.is_open:
            return
        self.get_logger().info(f"[{name}] Returning to Idle and clearing errors...")
        # Clear errors first in case of undervoltage from previous motor
        ser.write(b"sc\n")
        time.sleep(0.2)
        # Force Idle
        ser.write(b"v 0 0.0\n")
        time.sleep(0.1)
        ser.write(b"w axis0.requested_state 1\n")
        time.sleep(0.2)
        # Clear errors again just to be perfectly sure
        ser.write(b"sc\n")
        time.sleep(0.5)
        
        while ser.in_waiting > 0:
            ser.read(ser.in_waiting)
            
        if self.run_calibration:
            self.get_logger().info(f"[{name}] Starting full calibration (Beeping + turning)...")
            ser.write(b"w axis0.requested_state 3\n")
            
            self.get_logger().info(f"[{name}] Sleeping 20 seconds for calibration to finish without UART interruption...")
            time.sleep(20.0)
            
            # Flush any UART garbage
            while ser.in_waiting > 0:
                ser.read(ser.in_waiting)
                
            self.get_logger().info(f"[{name}] Checking if calibration succeeded...")
            ser.write(b"r axis0.error\n")
            time.sleep(0.2)
            err_response = ser.readline().decode('ascii', errors='ignore').strip()
            if err_response.endswith('d'): err_response = err_response[:-1]
            
            if err_response and err_response != "0" and " " not in err_response:
                raise RuntimeError(f"[{name}] Motor error detected after calibration: {err_response}. Aborting.")
                
            ser.write(b"r axis0.current_state\n")
            time.sleep(0.2)
            state_response = ser.readline().decode('ascii', errors='ignore').strip()
            if state_response.endswith('d'): state_response = state_response[:-1]
            if "1" not in state_response and "8" not in state_response:
                 self.get_logger().warn(f"[{name}] Expected state 1 or 8 after calibration, got: '{state_response}'")
        else:
            self.get_logger().info(f"[{name}] Skipping full calibration as per parameter.")
        
        self.get_logger().info(f"[{name}] Setting to Closed Loop Control (State 8)...")
        ser.write(b"w axis0.requested_state 8\n")
        time.sleep(0.5)
        
        # Verify it went to State 8 (ignoring streaming output like pos/vel if present)
        ser.write(b"r axis0.current_state\n")
        time.sleep(0.1)
        response = ser.readline().decode('ascii', errors='ignore').strip()
        if response and "8" not in response and " " not in response:
            self.get_logger().warn(f"[{name}] Unexpected response checking State 8: '{response}'. Proceeding.")

    def connect_serial(self, calibrate=False):
        # Gracefully handle specific port connection
        try:
            if self.left_serial is None or not self.left_serial.is_open:
                self.left_serial = serial.Serial(self.left_port, self.baudrate, timeout=0.1)
                if not self.left_calibrated and calibrate:
                    self.calibrate_motor(self.left_serial, "LEFT")
                    self.left_calibrated = True
        except Exception as e:
            self.get_logger().error(f"Failed to connect or calibrate left serial '{self.left_port}': {e}", throttle_duration_sec=2.0)
            if self.left_serial:
                self.left_serial.close()
                self.left_serial = None
            self.left_calibrated = False
            
        try:
            if self.right_serial is None or not self.right_serial.is_open:
                self.right_serial = serial.Serial(self.right_port, self.baudrate, timeout=0.1)
                if not self.right_calibrated and calibrate:
                    self.calibrate_motor(self.right_serial, "RIGHT")
                    self.right_calibrated = True
        except Exception as e:
            self.get_logger().error(f"Failed to connect or calibrate right serial '{self.right_port}': {e}", throttle_duration_sec=2.0)
            if self.right_serial:
                self.right_serial.close()
                self.right_serial = None
            self.right_calibrated = False

    def send_command(self, ser, cmd):
        if ser is not None and ser.is_open:
            try:
                ser.write((cmd + '\n').encode('ascii'))
            except serial.SerialException:
                ser.close()
                self.get_logger().warn(f"Serial write failed. Connection closed.")

    def update_diagnostics(self):
        # Poll Left
        if self.left_serial and self.left_serial.is_open:
            v, i, e = self.poll_motor_diag(self.left_serial)
            if v: self.motor_diag["left"]["voltage"] = v
            if i: self.motor_diag["left"]["current"] = i
            if e: self.motor_diag["left"]["error"] = e
            self.motor_diag["left"]["state"] = "Closed Loop" if self.motor_diag["left"]["error"] == "0" else "Error"
        else:
            self.motor_diag["left"]["state"] = "Disconnected"
            
        # Poll Right
        if self.right_serial and self.right_serial.is_open:
            v, i, e = self.poll_motor_diag(self.right_serial)
            if v: self.motor_diag["right"]["voltage"] = v
            if i: self.motor_diag["right"]["current"] = i
            if e: self.motor_diag["right"]["error"] = e
            self.motor_diag["right"]["state"] = "Closed Loop" if self.motor_diag["right"]["error"] == "0" else "Error"
        else:
            self.motor_diag["right"]["state"] = "Disconnected"
            
        msg = String()
        msg.data = json.dumps(self.motor_diag)
        self.status_pub.publish(msg)

    def poll_motor_diag(self, ser):
        v = None
        i = None
        e = None
        try:
            # Voltage
            ser.write(b"r vbus_voltage\n")
            for _ in range(3):
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line and " " not in line:
                    v = f"{float(line):.1f}"
                    break
            
            # Current
            ser.write(b"r axis0.motor.current_control.Iq_measured\n")
            for _ in range(3):
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line and " " not in line:
                    i = f"{float(line):.1f}"
                    break
            
            # Error
            ser.write(b"r axis0.error\n")
            for _ in range(3):
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line and " " not in line:
                    if line.endswith('d'): line = line[:-1]
                    e = line
                    break
        except:
            pass
        return v, i, e

    def read_feedback(self, ser):
        if ser is not None and ser.is_open:
            try:
                self.send_command(ser, 'f 0')
                line = ser.readline().decode('ascii').strip()
                if line:
                    parts = line.split()
                    if len(parts) >= 2:
                        pos = float(parts[0])
                        vel = float(parts[1])
                        return pos, vel
            except (serial.SerialException, ValueError) as e:
                pass # Fail silently, will retry/reconnect on next loop
        return None, None

    def cmd_vel_callback(self, msg):
        v_x = msg.linear.x
        omega_z = msg.angular.z
        
        # Inverse kinematics
        v_left = v_x - (omega_z * self.track_width / 2.0)
        v_right = v_x + (omega_z * self.track_width / 2.0)
        
        # Convert ms to turns/s
        circumference = 2.0 * math.pi * self.wheel_radius
        turns_left = v_left / circumference
        turns_right = v_right / circumference
        
        # Clamp absolute max speed to 1 turn per sec
        turns_left = max(-1.0, min(1.0, turns_left))
        turns_right = max(-1.0, min(1.0, turns_right))
        
        if self.invert_left:
            turns_left = -turns_left
        if self.invert_right:
            turns_right = -turns_right
        
        # Store commands to be sent synchronously in odom_loop
        self.last_cmd_l = f'v 0 {turns_left:.3f}'
        self.last_cmd_r = f'v 0 {turns_right:.3f}'

    def odom_loop(self):
        self.connect_serial(calibrate=True)
        
        if self.last_cmd_l is not None:
            self.send_command(self.left_serial, self.last_cmd_l)
            self.last_cmd_l = None
            
        if self.last_cmd_r is not None:
            self.send_command(self.right_serial, self.last_cmd_r)
            self.last_cmd_r = None
        
        left_pos, left_vel_turns = self.read_feedback(self.left_serial)
        right_pos, right_vel_turns = self.read_feedback(self.right_serial)
        
        self.diag_counter += 1
        if self.diag_counter >= self.odom_freq:
            self.diag_counter = 0
            self.update_diagnostics()
            
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if left_vel_turns is not None and right_vel_turns is not None and dt > 0:
            if self.invert_left:
                left_vel_turns = -left_vel_turns
            if self.invert_right:
                right_vel_turns = -right_vel_turns

            # Convert turns/s to m/s
            circumference = 2.0 * math.pi * self.wheel_radius
            v_left = left_vel_turns * circumference
            v_right = right_vel_turns * circumference
            
            # Forward kinematics
            v_x = (v_right + v_left) / 2.0
            omega_z = (v_right - v_left) / self.track_width
            
            # Odometry integration
            delta_x = v_x * math.cos(self.theta) * dt
            delta_y = v_x * math.sin(self.theta) * dt
            delta_theta = omega_z * dt
            
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta
            
            # Quaternion from yaw
            q = [0.0, 0.0, math.sin(self.theta / 2.0), math.cos(self.theta / 2.0)]
            
            # Publish Transform
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)
            
            # Publish Odometry message
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            odom.twist.twist.linear.x = v_x
            odom.twist.twist.angular.z = omega_z
            self.odom_pub.publish(odom)
            
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdescDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.left_serial and node.left_serial.is_open:
            node.send_command(node.left_serial, 'v 0 0.0')
            time.sleep(0.1)
            node.left_serial.write(b"w axis0.requested_state 1\n")
            node.left_serial.close()
        if node.right_serial and node.right_serial.is_open:
            node.send_command(node.right_serial, 'v 0 0.0')
            time.sleep(0.1)
            node.right_serial.write(b"w axis0.requested_state 1\n")
            node.right_serial.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
