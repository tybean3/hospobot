#!/usr/bin/env python3
"""
ODrive CAN Drive Node
Replaces the serial-based odesc_drive_node.
Uses the ODrive CAN protocol (v0.5.x) over the ROS 2 SocketCAN bridge.

ODrive CAN Command IDs (cmd_id in frame ID = node_id << 5 | cmd_id):
  0x001 = Heartbeat            (ODrive -> NUC, auto-sent)
  0x008 = Set_Axis_State       (NUC -> ODrive)
  0x009 = Get_Encoder_Estimates (ODrive -> NUC, streamed)
  0x00E = Clear_Errors         (NUC -> ODrive)
  0x011 = Set_Input_Vel        (NUC -> ODrive)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String
from tf2_ros import TransformBroadcaster
from can_msgs.msg import Frame
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import struct
import json
import math
import time

CMD_HEARTBEAT         = 0x001
CMD_SET_AXIS_STATE    = 0x007
CMD_ENCODER_ESTIMATES = 0x009
CMD_SET_CONTROLLER_MODE = 0x00B
CMD_SET_INPUT_VEL     = 0x00D
CMD_REBOOT            = 0x016
CMD_CLEAR_ERRORS      = 0x018

STATE_IDLE                = 0x01
STATE_FULL_CALIBRATION    = 0x03
STATE_CLOSED_LOOP_CONTROL = 0x08

STATE_NAMES = {
    0x00: "Undefined",
    0x01: "Idle",
    0x02: "Startup-Sequence",
    0x03: "Full-Calibration",
    0x04: "Motor-Calibration",
    0x06: "Encoder-Offset-Calib",
    0x07: "Lockin-Spin",
    0x08: "Closed-Loop-Velocity",
    0x09: "Velocity-Control",
    0x0A: "Encoder-Dir-Find",
    0x0B: "Homing",
    0x0C: "Encoder-Hall-Polarity",
    0x0D: "Encoder-Hall-Phase",
}


class OdriveCanNode(Node):
    def __init__(self):
        super().__init__('odrive_can_node')

        self.declare_parameter('left_node_id', 0x01)
        self.declare_parameter('right_node_id', 0x02)
        self.declare_parameter('track_width', 0.4)
        self.declare_parameter('wheel_radius', 0.08)
        self.declare_parameter('odom_freq', 20.0)
        self.declare_parameter('invert_drive_left', False)
        self.declare_parameter('invert_drive_right', True)
        self.declare_parameter('invert_odom_left', True)
        self.declare_parameter('invert_odom_right', False)
        self.declare_parameter('heartbeat_timeout', 2.0)
        self.declare_parameter('swivel_assist_vel', 0.05)
        self.declare_parameter('left_vel_multiplier', 1.0)
        self.declare_parameter('right_vel_multiplier', 1.0)

        self.left_id      = self.get_parameter('left_node_id').value
        self.right_id     = self.get_parameter('right_node_id').value
        self.track_width  = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.odom_freq    = self.get_parameter('odom_freq').value
        self.invert_drive_left  = self.get_parameter('invert_drive_left').value
        self.invert_drive_right = self.get_parameter('invert_drive_right').value
        self.invert_odom_left  = self.get_parameter('invert_odom_left').value
        self.invert_odom_right = self.get_parameter('invert_odom_right').value
        self.hb_timeout   = self.get_parameter('heartbeat_timeout').value
        self.swivel_assist_vel = self.get_parameter('swivel_assist_vel').value
        self.left_vel_multiplier = self.get_parameter('left_vel_multiplier').value
        self.right_vel_multiplier = self.get_parameter('right_vel_multiplier').value
        
        self.max_linear = 1.05 # Default m/s

        self.can_pub = self.create_publisher(Frame, 'to_can_bus', 10)
        self.can_sub = self.create_subscription(Frame, 'from_can_bus', self.can_rx_cb, 20)

        self.odom_pub       = self.create_publisher(Odometry, '/odom_can', 10)
        self.status_pub     = self.create_publisher(String, '/odesc_hardware/motor_status', 10)
        self.diag_pub       = self.create_publisher(DiagnosticArray, '/diagnostics', 1)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_vel_sub  = self.create_subscription(Twist, '/cmd_vel_out', self.cmd_vel_cb, 10)
        self.state_l_sub  = self.create_subscription(String, '/odesc_hardware/state_left', self.state_left_cb, 10)
        self.state_r_sub  = self.create_subscription(String, '/odesc_hardware/state_right', self.state_right_cb, 10)
        self.calib_l_sub  = self.create_subscription(Empty, '/odesc_hardware/calibrate_left', self.calib_left_cb, 10)
        self.calib_r_sub  = self.create_subscription(Empty, '/odesc_hardware/calibrate_right', self.calib_right_cb, 10)
        self.reboot_l_sub = self.create_subscription(Empty, '/odesc_hardware/reboot_left', self.reboot_left_cb, 10)
        self.reboot_r_sub = self.create_subscription(Empty, '/odesc_hardware/reboot_right', self.reboot_right_cb, 10)
        self.settings_sub = self.create_subscription(String, '/odesc_hardware/settings', self.settings_cb, 10)

        self.motor_state = {
            'left':  {'state': 'Unknown', 'error': 0, 'vel': 0.0, 'pos': 0.0, 'last_seen': 0.0},
            'right': {'state': 'Unknown', 'error': 0, 'vel': 0.0, 'pos': 0.0, 'last_seen': 0.0},
        }

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()

        self.cmd_vel_l = 0.0
        self.cmd_vel_r = 0.0
        self.current_vel_l = 0.0
        self.current_vel_r = 0.0

        self.odom_timer = self.create_timer(1.0 / self.odom_freq, self.odom_timer_cb)
        self.diag_timer = self.create_timer(1.0, self.diag_timer_cb)

        self.get_logger().info(
            f'ODrive CAN Node started. Left ID=0x{self.left_id:02X}, Right ID=0x{self.right_id:02X}')

        # The ODrive is configured to auto-publish encoder estimates over CAN (encoder_rate_ms = 20),
        # so we no longer need the USB fallback for odometry!

    def can_rx_cb(self, msg: Frame):
        if msg.is_rtr:
            return
            
        node_id = (msg.id >> 5) & 0x3F
        cmd_id  = msg.id & 0x1F

        if node_id == self.left_id:
            side = 'left'
        elif node_id == self.right_id:
            side = 'right'
        else:
            return

        self.motor_state[side]['last_seen'] = time.time()

        if cmd_id == CMD_HEARTBEAT and msg.dlc >= 8:
            axis_error = struct.unpack_from('<I', bytes(msg.data), 0)[0]
            axis_state = msg.data[4]
            self.motor_state[side]['state'] = STATE_NAMES.get(axis_state, f'State-{axis_state}')
            self.motor_state[side]['error'] = axis_error
            
            # Auto-start motors if they are in IDLE
            if side == 'left' and not getattr(self, 'left_auto_started', False) and axis_state == STATE_IDLE:
                self.get_logger().info('Auto-starting Left motor into CLOSED LOOP CONTROL...')
                self.clear_errors(self.left_id)
                self.set_controller_mode(self.left_id, 2, 1)
                self.set_axis_state(self.left_id, STATE_CLOSED_LOOP_CONTROL)
                self.left_auto_started = True
            elif side == 'right' and not getattr(self, 'right_auto_started', False) and axis_state == STATE_IDLE:
                self.get_logger().info('Auto-starting Right motor into CLOSED LOOP CONTROL...')
                self.clear_errors(self.right_id)
                self.set_controller_mode(self.right_id, 2, 1)
                self.set_axis_state(self.right_id, STATE_CLOSED_LOOP_CONTROL)
                self.right_auto_started = True

        elif cmd_id == CMD_ENCODER_ESTIMATES and msg.dlc >= 8:
            pos, vel = struct.unpack_from('<ff', bytes(msg.data), 0)
            if side == 'left':
                pos = -pos if self.invert_odom_left else pos
                vel = -vel if self.invert_odom_left else vel
            else:
                pos = -pos if self.invert_odom_right else pos
                vel = -vel if self.invert_odom_right else vel
            self.motor_state[side]['pos'] = pos
            self.motor_state[side]['vel'] = vel

    def send_frame(self, node_id, cmd_id, data_bytes, rtr=False):
        msg = Frame()
        msg.id = (node_id << 5) | (cmd_id & 0x1F)
        msg.is_rtr      = rtr
        msg.is_extended = False
        msg.is_error    = False
        msg.dlc         = len(data_bytes) if not rtr else 8
        msg.data        = list(data_bytes) + [0] * (8 - len(data_bytes)) if not rtr else [0]*8
        self.can_pub.publish(msg)

    def request_encoder_estimates(self):
        # Poll both motors to guarantee we receive their encoder estimates
        self.send_frame(self.left_id, CMD_ENCODER_ESTIMATES, [], rtr=True)
        self.send_frame(self.right_id, CMD_ENCODER_ESTIMATES, [], rtr=True)

    def set_axis_state(self, node_id, state: int):
        self.send_frame(node_id, CMD_SET_AXIS_STATE, list(struct.pack('<I', state)))

    def set_controller_mode(self, node_id, control_mode: int, input_mode: int):
        self.send_frame(node_id, CMD_SET_CONTROLLER_MODE, list(struct.pack('<ii', control_mode, input_mode)))

    def set_velocity(self, node_id, vel_turns_per_sec: float):
        self.send_frame(node_id, CMD_SET_INPUT_VEL, list(struct.pack('<ff', vel_turns_per_sec, 0.0)))

    def clear_errors(self, node_id):
        self.send_frame(node_id, CMD_CLEAR_ERRORS, [])

    def reboot(self, node_id):
        self.send_frame(node_id, CMD_REBOOT, [])

    def cmd_vel_cb(self, msg: Twist):
        v_x   = msg.linear.x
        omega = msg.angular.z
        
        # Anti-scrub / Swivel assist for diamond configuration
        if abs(omega) > 0.05 and abs(v_x) < self.swivel_assist_vel:
            # Inject a small forward velocity to help casters align
            v_x = self.swivel_assist_vel

        circ  = 2.0 * math.pi * self.wheel_radius
        v_l   = (v_x - omega * self.track_width / 2.0) / circ
        v_r   = (v_x + omega * self.track_width / 2.0) / circ
        if self.invert_drive_left:  v_l = -v_l
        if self.invert_drive_right: v_r = -v_r
        
        v_l *= self.left_vel_multiplier
        v_r *= self.right_vel_multiplier
        
        max_revs_s = self.max_linear / circ
        max_v = max(abs(v_l), abs(v_r))
        if max_v > max_revs_s:
            scale = max_revs_s / max_v
            v_l *= scale
            v_r *= scale
        
        self.cmd_vel_l = v_l
        self.cmd_vel_r = v_r

    def state_left_cb(self,  msg: String): self._apply_state(self.left_id,  msg.data)
    def state_right_cb(self, msg: String): self._apply_state(self.right_id, msg.data)

    def _apply_state(self, node_id, state_str: str):
        s = state_str.upper()
        if   'IDLE'   in s: self.set_axis_state(node_id, STATE_IDLE)
        elif 'CLOSED' in s or 'LOOP' in s or 'CL-VEL' in s: 
            self.clear_errors(node_id)
            # 2 = Velocity Control, 1 = Passthrough (ROS node handles the acceleration ramp)
            self.set_controller_mode(node_id, 2, 1)
            self.set_axis_state(node_id, STATE_CLOSED_LOOP_CONTROL)
        elif 'CALIB'  in s: self.set_axis_state(node_id, STATE_FULL_CALIBRATION)

    def calib_left_cb(self,  msg: Empty): self.set_axis_state(self.left_id,  STATE_FULL_CALIBRATION)
    def calib_right_cb(self, msg: Empty): self.set_axis_state(self.right_id, STATE_FULL_CALIBRATION)
    def reboot_left_cb(self, msg: Empty): 
        self.clear_errors(self.left_id)
        self.reboot(self.left_id)
    def reboot_right_cb(self, msg: Empty): 
        self.clear_errors(self.right_id)
        self.reboot(self.right_id)

    def settings_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if 'invert_left'  in data: self.invert_left  = data['invert_left']
            if 'invert_right' in data: self.invert_right = data['invert_right']
            if 'max_linear'   in data: self.max_linear   = data['max_linear']
        except Exception as e:
            self.get_logger().error(f'Failed to parse settings: {e}')

    def odom_timer_cb(self):
        now_ts = time.time()
        
        # Poll the ODESC (right motor) for encoder estimates because it doesn't send them cyclically
        self.request_encoder_estimates()
        # Only send velocity commands to an ODrive if it has sent a heartbeat recently.
        # Sending to a disconnected node floods the CAN bus with unACKed frames,
        # filling the kernel socket buffer and causing 'No buffer space available'.
        left_alive  = (self.motor_state['left']['last_seen']  > 0.0 and
                       now_ts - self.motor_state['left']['last_seen']  < self.hb_timeout)
        right_alive = (self.motor_state['right']['last_seen'] > 0.0 and
                       now_ts - self.motor_state['right']['last_seen'] < self.hb_timeout)

        if left_alive or right_alive:
            # Software slew rate limiter (2 turns/sec^2 acceleration)
            dt_s = 1.0 / self.odom_freq
            max_delta = 2.0 * dt_s
            
            # Left motor ramp
            delta_l = self.cmd_vel_l - self.current_vel_l
            if delta_l > max_delta: self.current_vel_l += max_delta
            elif delta_l < -max_delta: self.current_vel_l -= max_delta
            else: self.current_vel_l = self.cmd_vel_l
            
            # Right motor ramp
            delta_r = self.cmd_vel_r - self.current_vel_r
            if delta_r > max_delta: self.current_vel_r += max_delta
            elif delta_r < -max_delta: self.current_vel_r -= max_delta
            else: self.current_vel_r = self.cmd_vel_r

        if left_alive:
            self.set_velocity(self.left_id, self.current_vel_l)
        if right_alive:
            self.set_velocity(self.right_id, self.current_vel_r)

        now = self.get_clock().now()
        dt  = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        circ   = 2.0 * math.pi * self.wheel_radius
        
        if left_alive:
            v_left  = self.motor_state['left'].get('vel', 0.0)  * circ
        else:
            v_left = 0.0
        
        if right_alive:
            v_right = self.motor_state['right'].get('vel', 0.0) * circ
        else:
            v_right = 0.0

        # LOG THE VELOCITIES SO THE USER CAN SEE THEM
        self.get_logger().info(f"Odom: v_left={v_left:.3f} m/s, v_right={v_right:.3f} m/s")

        v_x    = (v_right + v_left) / 2.0
        omega  = (v_right - v_left) / self.track_width

        self.x     += v_x * math.cos(self.theta) * dt
        self.y     += v_x * math.sin(self.theta) * dt
        self.theta += omega * dt

        q = [0.0, 0.0, math.sin(self.theta / 2.0), math.cos(self.theta / 2.0)]

        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        # TF broadcast removed to allow EKF to manage the odom -> base_footprint transform

        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x  = v_x
        odom.twist.twist.angular.z = omega
        
        # Populate covariances (required for EKF)
        # Pose covariance (x, y, z, roll, pitch, yaw)
        odom.pose.covariance[0]  = 0.05  # x
        odom.pose.covariance[7]  = 0.05  # y
        odom.pose.covariance[14] = 1e-9  # z
        odom.pose.covariance[21] = 1e-9  # roll
        odom.pose.covariance[28] = 1e-9  # pitch
        odom.pose.covariance[35] = 0.05  # yaw
        
        # Twist covariance (vx, vy, vz, vroll, vpitch, vyaw)
        odom.twist.covariance[0]  = 0.01  # vx
        odom.twist.covariance[7]  = 1e-9  # vy (non-holonomic)
        odom.twist.covariance[14] = 1e-9  # vz
        odom.twist.covariance[21] = 1e-9  # vroll
        odom.twist.covariance[28] = 1e-9  # vpitch
        odom.twist.covariance[35] = 0.5   # vyaw (high covariance so EKF trusts visual odometry for rotation)

        self.odom_pub.publish(odom)

    def diag_timer_cb(self):
        now = time.time()

        status_data = {
            'left':  {'state': self.motor_state['left']['state'],  'error': hex(self.motor_state['left']['error']),  'voltage': '0.0', 'current': '0.0'},
            'right': {'state': self.motor_state['right']['state'], 'error': hex(self.motor_state['right']['error']), 'voltage': '0.0', 'current': '0.0'},
        }
        s = String()
        s.data = json.dumps(status_data)
        self.status_pub.publish(s)

        diag_arr = DiagnosticArray()
        diag_arr.header.stamp = self.get_clock().now().to_msg()

        for side, node_id in [('left', self.left_id), ('right', self.right_id)]:
            ms = self.motor_state[side]
            st = DiagnosticStatus()
            st.name        = f'ODrive-{"L" if side == "left" else "R"}'
            st.hardware_id = f'CAN-0x{node_id:02X}'

            if ms['last_seen'] == 0.0:
                st.level   = DiagnosticStatus.WARN
                st.message = 'No heartbeat received yet'
            elif now - ms['last_seen'] > self.hb_timeout:
                st.level   = DiagnosticStatus.ERROR
                st.message = 'Heartbeat timeout'
            else:
                st.level   = DiagnosticStatus.OK
                st.message = ms['state']

            st.values.append(KeyValue(key='state',           value=ms['state']))
            st.values.append(KeyValue(key='error',           value=hex(ms['error'])))
            st.values.append(KeyValue(key='last_seen_s_ago', value=f'{now - ms["last_seen"]:.2f}'))
            diag_arr.status.append(st)

        self.diag_pub.publish(diag_arr)


def main(args=None):
    rclpy.init(args=args)
    node = OdriveCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down... Putting motors in IDLE directly via socketcan.')
        try:
            import can
            import struct
            bus = can.interface.Bus(channel='can0', bustype='socketcan')
            
            # Left motor IDLE
            msg_l = can.Message(arbitration_id=(node.left_id << 5) | 0x07, data=list(struct.pack('<I', 1)), is_extended_id=False)
            bus.send(msg_l)
            
            # Right motor IDLE
            msg_r = can.Message(arbitration_id=(node.right_id << 5) | 0x07, data=list(struct.pack('<I', 1)), is_extended_id=False)
            bus.send(msg_r)
            
            node.get_logger().info('Successfully sent IDLE commands to both motors via CAN.')
        except Exception as e:
            node.get_logger().error(f'Failed to send raw CAN IDLE command: {e}')
            
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
