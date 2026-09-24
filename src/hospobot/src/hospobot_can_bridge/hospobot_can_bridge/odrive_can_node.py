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
from std_msgs.msg import Empty, String, Float32
from tf2_ros import TransformBroadcaster
from can_msgs.msg import Frame
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import struct
import json
import math
import time
import os
import glob
import serial
import socket

CMD_HEARTBEAT         = 0x001
CMD_SET_AXIS_STATE    = 0x007
CMD_ENCODER_ESTIMATES = 0x009
CMD_SET_CONTROLLER_MODE = 0x00B
CMD_SET_INPUT_VEL     = 0x00D
CMD_GET_IBUS          = 0x014
CMD_GET_VBUS_VOLTAGE  = 0x017
CMD_REBOOT            = 0x016
CMD_CLEAR_ERRORS      = 0x018
CMD_SET_VEL_GAINS     = 0x01B

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
        self.declare_parameter('track_width', 0.300)
        self.declare_parameter('wheel_radius', 0.0625)
        self.declare_parameter('odom_freq', 20.0)
        self.declare_parameter('invert_drive_left', False)
        self.declare_parameter('invert_drive_right', True)
        self.declare_parameter('invert_odom_left', True)
        self.declare_parameter('invert_odom_right', False)
        self.declare_parameter('heartbeat_timeout', 2.0)
        self.declare_parameter('swivel_assist_vel', 0.0)
        self.declare_parameter('left_vel_multiplier', 1.0)
        self.declare_parameter('right_vel_multiplier', 1.0)
        self.declare_parameter('use_usb_left', True)
        self.declare_parameter('left_serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_vel_turns', 1.5)
        self.declare_parameter('max_linear', 1.05)       # Maximum linear speed (m/s)
        self.declare_parameter('accel_limit', 2.0)       # Acceleration ramp rate (turns/s^2)
        self.declare_parameter('vel_gain', 0.8)          # Velocity PI proportional gain
        self.declare_parameter('vel_integrator_gain', 4.0) # Velocity PI integrator gain
        self.declare_parameter('publish_tf', False)      # Disabled by default; enabled with odom_wheel for ICP guess
        self.declare_parameter('odom_frame', 'odom')

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
        self.use_usb_left = self.get_parameter('use_usb_left').value
        self.left_serial_port = self.get_parameter('left_serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.left_serial = None
        self.accel_limit = float(self.get_parameter('accel_limit').value)
        self.vel_gain = self.get_parameter('vel_gain').value
        self.vel_integrator_gain = self.get_parameter('vel_integrator_gain').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)


        # Native direct SocketCAN socket for zero-latency, synchronous CAN dispatch
        self.can_sock = None
        try:
            self.can_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can_sock.bind(('can0',))
            self.can_sock.setblocking(False)
            self.get_logger().info('Direct SocketCAN (can0) opened successfully for zero-latency motor control.')
        except Exception as e:
            self.get_logger().warn(f'Direct SocketCAN open failed: {e}. Falling back to /to_can_bus.')

        # Hard velocity cap — no motor ever exceeds this (turns/sec)
        self.max_vel_turns = float(self.get_parameter('max_vel_turns').value)
        self.max_linear = float(self.get_parameter('max_linear').value)

        self.can_pub = self.create_publisher(Frame, 'to_can_bus', 10)
        self.can_sub = self.create_subscription(Frame, 'from_can_bus', self.can_rx_cb, 20)

        self.odom_pub       = self.create_publisher(Odometry, '/odom', 10)
        self.odom_can_pub   = self.create_publisher(Odometry, '/odom_can', 10)
        self.status_pub     = self.create_publisher(String, '/odesc_hardware/motor_status', 10)
        self.diag_pub       = self.create_publisher(DiagnosticArray, '/diagnostics', 1)
        self.volt_pub       = self.create_publisher(Float32, '/sensors/bus_voltage', 10)
        self.curr_pub       = self.create_publisher(Float32, '/sensors/bus_current', 10)
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
            'left':  {'state': 'Unknown', 'error': 0, 'vel': 0.0, 'pos': 0.0, 'last_seen': 0.0,
                      'voltage': 0.0, 'current': 0.0},
            'right': {'state': 'Unknown', 'error': 0, 'vel': 0.0, 'pos': 0.0, 'last_seen': 0.0,
                      'voltage': 0.0, 'current': 0.0},
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

        if node_id == self.left_id and not self.use_usb_left:
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

            # Track and maintain CLOSED LOOP CONTROL unless explicitly commanded IDLE
            if axis_state == STATE_CLOSED_LOOP_CONTROL:
                if side == 'right':
                    self.right_auto_started = True
                elif side == 'left':
                    self.left_auto_started = True
            elif axis_state == STATE_IDLE and not getattr(self, f'{side}_explicit_idle', False):
                now_ts = time.time()
                last_retry = getattr(self, f'_{side}_last_engage_attempt', 0.0)
                if now_ts - last_retry > 0.5:
                    setattr(self, f'_{side}_last_engage_attempt', now_ts)
                    self.get_logger().info(f'Engaging {side.capitalize()} motor into CLOSED LOOP CONTROL...', throttle_duration_sec=2.0)
                    target_id = self.left_id if side == 'left' else self.right_id
                    self.clear_errors(target_id)
                    self.set_controller_mode(target_id, 2, 1)
                    self.set_vel_gains(target_id, self.vel_gain, self.vel_integrator_gain)
                    self.set_axis_state(target_id, STATE_CLOSED_LOOP_CONTROL)

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

        elif cmd_id == CMD_GET_VBUS_VOLTAGE and msg.dlc >= 4:
            vbus = struct.unpack_from('<f', bytes(msg.data), 0)[0]
            if 0.0 < vbus < 60.0:  # sanity check
                self.motor_state[side]['voltage'] = vbus

        elif cmd_id == CMD_GET_IBUS and msg.dlc >= 4:
            ibus = struct.unpack_from('<f', bytes(msg.data), 0)[0]
            self.motor_state[side]['current'] = abs(ibus)

    def send_frame(self, node_id, cmd_id, data_bytes, rtr=False):
        can_id = (node_id << 5) | (cmd_id & 0x1F)
        dlc = len(data_bytes) if not rtr else 8
        payload = list(data_bytes) + [0] * (8 - len(data_bytes)) if not rtr else [0] * 8

        # 1. Fast direct transmit via kernel SocketCAN (instantaneous, zero DDS/IPC latency)
        if self.can_sock:
            try:
                raw_id = can_id
                if rtr:
                    raw_id |= 0x40000000  # CAN_RTR_FLAG
                frame = struct.pack('=IB3x8s', raw_id, dlc, bytes(payload))
                self.can_sock.send(frame)
            except Exception as e:
                self.get_logger().warn(f"Direct CAN send error: {e}", throttle_duration_sec=2.0)

        # 2. Publish to ROS topic for diagnostic visibility, but skip SET_INPUT_VEL if direct CAN is active
        # to prevent socket_can_sender_node from rebroadcasting duplicate velocity frames with latency
        if not (self.can_sock and cmd_id == CMD_SET_INPUT_VEL):
            msg = Frame()
            msg.id = can_id
            msg.is_rtr      = rtr
            msg.is_extended = False
            msg.is_error    = False
            msg.dlc         = dlc
            msg.data        = payload
            self.can_pub.publish(msg)

    def connect_left_usb(self):
        if not self.use_usb_left:
            return
        if self.left_serial is None or not self.left_serial.is_open:
            ports_to_try = [self.left_serial_port] + sorted(glob.glob('/dev/ttyACM*'))
            ports_to_try = list(dict.fromkeys(ports_to_try))
            for port in ports_to_try:
                try:
                    self.left_serial = serial.Serial(port, self.baudrate, timeout=0.01, dsrdtr=False, rtscts=False)
                    self.left_serial_port = port
                    self.get_logger().info(f"Connected to Left ODrive on {port}")
                    
                    # Initialize Left ODrive into Closed Loop Velocity mode with matched gains
                    self.send_usb_cmd("sc")
                    self.send_usb_cmd(f"w axis0.controller.config.vel_gain {self.vel_gain}")
                    self.send_usb_cmd(f"w axis0.controller.config.vel_integrator_gain {self.vel_integrator_gain}")
                    self.send_usb_cmd("w axis0.controller.config.control_mode 2")
                    self.send_usb_cmd("w axis0.controller.config.input_mode 1")
                    if not getattr(self, 'left_explicit_idle', False):
                        self.send_usb_cmd("w axis0.requested_state 8")
                        self.left_auto_started = True
                        self.get_logger().info(f"Left ODrive (USB) auto-started into CLOSED_LOOP_CONTROL (vel_gain={self.vel_gain}, vel_integrator_gain={self.vel_integrator_gain}).")
                    break
                except Exception as e:
                    if self.left_serial:
                        try:
                            self.left_serial.close()
                        except Exception:
                            pass
                        self.left_serial = None
            if not self.left_serial or not self.left_serial.is_open:
                self.get_logger().error(f"Failed to connect to Left ODrive on any ttyACM port: {ports_to_try}", throttle_duration_sec=2.0)

    def send_usb_cmd(self, cmd):
        if self.left_serial and self.left_serial.is_open:
            try:
                self.left_serial.write((cmd + '\n').encode('ascii'))
                self.left_serial.flush()
            except Exception as e:
                self.get_logger().error(f"USB send error for '{cmd}': {e}")
                try:
                    self.left_serial.close()
                except Exception:
                    pass
                self.left_serial = None

    def request_encoder_estimates(self):
        # Poll both motors to guarantee we receive their encoder estimates
        if not self.use_usb_left:
            self.send_frame(self.left_id, CMD_ENCODER_ESTIMATES, [], rtr=True)
        self.send_frame(self.right_id, CMD_ENCODER_ESTIMATES, [], rtr=True)

    def set_axis_state(self, node_id, state: int):
        self.send_frame(node_id, CMD_SET_AXIS_STATE, list(struct.pack('<I', state)))

    def set_controller_mode(self, node_id, control_mode: int, input_mode: int):
        self.send_frame(node_id, CMD_SET_CONTROLLER_MODE, list(struct.pack('<ii', control_mode, input_mode)))

    def set_vel_gains(self, node_id, vel_gain: float, vel_integrator_gain: float):
        self.send_frame(node_id, CMD_SET_VEL_GAINS, list(struct.pack('<ff', vel_gain, vel_integrator_gain)))

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
        if self.swivel_assist_vel > 0.0 and abs(omega) > 0.05 and abs(v_x) < self.swivel_assist_vel:
            # Inject a small forward velocity to help casters align
            v_x = self.swivel_assist_vel

        circ  = 2.0 * math.pi * self.wheel_radius
        v_l   = (v_x - omega * self.track_width / 2.0) / circ
        v_r   = (v_x + omega * self.track_width / 2.0) / circ
        if self.invert_drive_left:  v_l = -v_l
        if self.invert_drive_right: v_r = -v_r

        v_l *= self.left_vel_multiplier
        v_r *= self.right_vel_multiplier

        # Legacy linear-speed proportional scaling (preserves differential ratio)
        max_revs_s = self.max_linear / circ
        max_v = max(abs(v_l), abs(v_r))
        if max_v > max_revs_s:
            scale = max_revs_s / max_v
            v_l *= scale
            v_r *= scale

        # Hard cap: no motor may ever be commanded above max_vel_turns (turns/sec)
        v_l = max(-self.max_vel_turns, min(self.max_vel_turns, v_l))
        v_r = max(-self.max_vel_turns, min(self.max_vel_turns, v_r))

        self.cmd_vel_l = v_l
        self.cmd_vel_r = v_r

    def state_left_cb(self,  msg: String): 
        if self.use_usb_left:
            self._apply_state_usb(msg.data)
        else:
            self._apply_state(self.left_id,  msg.data)
            
    def state_right_cb(self, msg: String): self._apply_state(self.right_id, msg.data)

    def _apply_state(self, node_id, state_str: str):
        s = state_str.upper()
        side = 'left' if node_id == self.left_id else 'right'
        if 'IDLE' in s:
            setattr(self, f'{side}_explicit_idle', True)
            self.set_axis_state(node_id, STATE_IDLE)
        elif 'CLOSED' in s or 'LOOP' in s or 'CL-VEL' in s or 'ENGAGED' in s:
            setattr(self, f'{side}_explicit_idle', False)
            self.clear_errors(node_id)
            # 2 = Velocity Control, 1 = Passthrough (ROS node handles the acceleration ramp)
            self.set_controller_mode(node_id, 2, 1)
            self.set_vel_gains(node_id, self.vel_gain, self.vel_integrator_gain)
            self.set_axis_state(node_id, STATE_CLOSED_LOOP_CONTROL)
        elif 'CALIB' in s:
            setattr(self, f'{side}_explicit_idle', False)
            self.set_axis_state(node_id, STATE_FULL_CALIBRATION)
        elif 'INDEX' in s:
            setattr(self, f'{side}_explicit_idle', False)
            self.set_axis_state(node_id, 6)

    def _apply_state_usb(self, state_str: str):
        s = state_str.upper()
        if 'IDLE' in s: 
            self.left_explicit_idle = True
            self.send_usb_cmd("v 0 0.0")
            self.send_usb_cmd("w axis0.requested_state 1")
        elif 'CLOSED' in s or 'LOOP' in s or 'CL-VEL' in s or 'ENGAGED' in s: 
            self.left_explicit_idle = False
            self.send_usb_cmd("sc")
            self.send_usb_cmd("w axis0.requested_state 1")
            self.send_usb_cmd("sc")
            self.send_usb_cmd(f"w axis0.controller.config.vel_gain {self.vel_gain}")
            self.send_usb_cmd(f"w axis0.controller.config.vel_integrator_gain {self.vel_integrator_gain}")
            self.send_usb_cmd("w axis0.controller.config.control_mode 2")
            self.send_usb_cmd("w axis0.controller.config.input_mode 1")
            self.send_usb_cmd("w axis0.requested_state 8")
        elif 'CALIB' in s: 
            self.left_explicit_idle = False
            self.send_usb_cmd("sc")
            self.send_usb_cmd("w axis0.requested_state 3")
        elif 'INDEX' in s:
            self.left_explicit_idle = False
            self.send_usb_cmd("sc")
            self.send_usb_cmd("w axis0.requested_state 1")
            self.send_usb_cmd("sc")
            self.send_usb_cmd("w axis0.requested_state 6")

    def calib_left_cb(self,  msg: Empty): 
        if self.use_usb_left:
            self._apply_state_usb("CALIB")
        else:
            self.set_axis_state(self.left_id,  STATE_FULL_CALIBRATION)
            
    def calib_right_cb(self, msg: Empty): self.set_axis_state(self.right_id, STATE_FULL_CALIBRATION)
    
    def reboot_left_cb(self, msg: Empty): 
        if self.use_usb_left:
            self.send_usb_cmd("sc")
            self.send_usb_cmd("sr")
        else:
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
        
        if self.use_usb_left:
            self.connect_left_usb()

        # Both motors use the heartbeat timeout grace period
        left_alive = (self.motor_state['left']['last_seen'] > 0.0 and
                      now_ts - self.motor_state['left']['last_seen'] < self.hb_timeout)
        right_alive = (self.motor_state['right']['last_seen'] > 0.0 and
                       now_ts - self.motor_state['right']['last_seen'] < self.hb_timeout)

        if left_alive or right_alive:
            # Software slew rate limiter (accel_limit turns/sec^2 acceleration)
            dt_s = 1.0 / self.odom_freq
            max_delta = self.accel_limit * dt_s
            
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

        # 1. Dispatch CAN FIRST (direct SocketCAN is non-blocking, ~10 microseconds into CAN FIFO)
        if right_alive:
            self.set_velocity(self.right_id, self.current_vel_r)

        # 2. Dispatch USB SECOND (pySerial write + flush, ~50 microseconds)
        if left_alive:
            if self.use_usb_left:
                self.send_usb_cmd(f"v 0 {self.current_vel_l:.3f}")
            else:
                self.set_velocity(self.left_id, self.current_vel_l)
        else:
            if abs(self.cmd_vel_l) > 0.01:
                self.get_logger().warn(
                    f"Left motor velocity NOT sent because left_alive is False! last_seen={now_ts - self.motor_state['left']['last_seen']:.2f}s ago",
                    throttle_duration_sec=1.0
                )

        # Log commands when moving
        if abs(self.current_vel_l) > 0.001 or abs(self.current_vel_r) > 0.001:
            self.get_logger().info(
                f"[VEL_CMD] Left: cmd={self.cmd_vel_l:.3f}, ramp={self.current_vel_l:.3f}, alive={left_alive} | "
                f"Right: cmd={self.cmd_vel_r:.3f}, ramp={self.current_vel_r:.3f}, alive={right_alive}",
                throttle_duration_sec=0.5
            )

        # 2. Telemetry & encoder polling (executed after velocity dispatch)
        # Poll right motor for encoder estimates
        self.request_encoder_estimates()
        
        # Poll left USB for encoder estimates
        if self.use_usb_left and self.left_serial and self.left_serial.is_open:
            try:
                self.left_serial.write(b"f 0\n")
                self.left_serial.flush()
                line = self.left_serial.readline().decode('ascii', errors='ignore').strip()
                if line:
                    parts = line.split()
                    if len(parts) >= 2:
                        pos = float(parts[0])
                        vel = float(parts[1])
                        if self.invert_odom_left:
                            pos, vel = -pos, -vel
                        self.motor_state['left']['pos'] = pos
                        self.motor_state['left']['vel'] = vel
                        self.motor_state['left']['last_seen'] = now_ts
            except Exception as e:
                self.get_logger().warn(f"Left USB encoder read error: {e}", throttle_duration_sec=2.0)

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
        t.header.frame_id = self.odom_frame
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = self.odom_frame
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
        odom.twist.covariance[0]  = 0.002 # vx (gives wheels a stronger factor in EKF to stabilize longitudinal scale)
        odom.twist.covariance[7]  = 1e-9  # vy (non-holonomic)
        odom.twist.covariance[14] = 1e-9  # vz
        odom.twist.covariance[21] = 1e-9  # vroll
        odom.twist.covariance[28] = 1e-9  # vpitch
        odom.twist.covariance[35] = 0.1   # vyaw (differential drive provides strong yaw, EKF fuses with VIO/laser)

        if self.publish_tf:
            self.odom_pub.publish(odom)
        self.odom_can_pub.publish(odom)

    def _usb_query(self, cmd_bytes):
        """Send a read command to the USB ODESC and return the stripped response line, or None."""
        if not (self.use_usb_left and self.left_serial and self.left_serial.is_open):
            return None
        try:
            self.left_serial.reset_input_buffer()
            self.left_serial.write(cmd_bytes)
            self.left_serial.flush()
            line = self.left_serial.readline().decode('ascii', errors='ignore').strip()
            return line if line else None
        except Exception:
            return None

    def diag_timer_cb(self):
        now = time.time()

        if self.use_usb_left and self.left_serial and self.left_serial.is_open:
            try:
                # Axis error
                line = self._usb_query(b"r axis0.error\n")
                if line:
                    if line.endswith('d'):
                        line = line[:-1]
                    if line.isdigit():
                        self.motor_state['left']['error'] = int(line)

                # Axis state
                line = self._usb_query(b"r axis0.current_state\n")
                if line:
                    if line.endswith('d'):
                        line = line[:-1]
                    if line.isdigit():
                        state_val = int(line)
                        states = {1: "Idle", 3: "Full-Calibration", 8: "Closed-Loop-Velocity"}
                        self.motor_state['left']['state'] = states.get(state_val, f"State-{state_val}")

                        # Maintain CLOSED LOOP CONTROL unless explicitly commanded IDLE (same as CAN motor)
                        if state_val == 1 and not getattr(self, 'left_explicit_idle', False):
                            now_ts = time.time()
                            last_retry = getattr(self, '_left_last_engage_attempt', 0.0)
                            if now_ts - last_retry > 1.0:
                                self._left_last_engage_attempt = now_ts
                                self.get_logger().info('Re-engaging Left motor (USB) into CLOSED LOOP CONTROL...', throttle_duration_sec=2.0)
                                self.send_usb_cmd("sc")
                                self.send_usb_cmd(f"w axis0.controller.config.vel_gain {self.vel_gain}")
                                self.send_usb_cmd(f"w axis0.controller.config.vel_integrator_gain {self.vel_integrator_gain}")
                                self.send_usb_cmd("w axis0.controller.config.control_mode 2")
                                self.send_usb_cmd("w axis0.controller.config.input_mode 1")
                                self.send_usb_cmd("w axis0.requested_state 8")

                # Bus voltage
                line = self._usb_query(b"r vbus_voltage\n")
                if line:
                    try:
                        v = float(line)
                        if 0.0 < v < 60.0:
                            self.motor_state['left']['voltage'] = v
                    except ValueError:
                        pass

                # Bus current (Iq_measured as proxy for motor current draw)
                line = self._usb_query(b"r axis0.motor.current_control.Iq_measured\n")
                if line:
                    try:
                        self.motor_state['left']['current'] = abs(float(line))
                    except ValueError:
                        pass

            except Exception:
                pass

        # Request vbus and ibus from right motor (ODrive S1) via CAN RTR frames
        self.send_frame(self.right_id, CMD_GET_VBUS_VOLTAGE, [], rtr=True)
        self.send_frame(self.right_id, CMD_GET_IBUS, [], rtr=True)

        # Compute averaged voltage and current across both drivers
        v_l = self.motor_state['left']['voltage']
        v_r = self.motor_state['right']['voltage']
        i_l = self.motor_state['left']['current']
        i_r = self.motor_state['right']['current']

        # Only average values that are non-zero (i.e. received)
        valid_voltages = [v for v in [v_l, v_r] if v > 0.0]
        valid_currents = [i for i in [i_l, i_r] if i >= 0.0]

        avg_volt_val = 0.0
        if valid_voltages:
            avg_volt = sum(valid_voltages) / len(valid_voltages)
            avg_volt_val = float(avg_volt)
            vm = Float32()
            vm.data = avg_volt_val
            self.volt_pub.publish(vm)

        avg_curr_val = 0.0
        if valid_currents:
            avg_curr = sum(valid_currents) / len(valid_currents)
            avg_curr_val = float(avg_curr)
            cm = Float32()
            cm.data = avg_curr_val
            self.curr_pub.publish(cm)

        if avg_volt_val > 0.0:
            try:
                with open('/tmp/hospobot_power.json.tmp', 'w') as pf:
                    json.dump({'voltage': avg_volt_val, 'current': avg_curr_val, 'timestamp': time.time()}, pf)
                os.replace('/tmp/hospobot_power.json.tmp', '/tmp/hospobot_power.json')
            except Exception:
                pass

        status_data = {
            'left':  {
                'state':   self.motor_state['left']['state'],
                'error':   hex(int(self.motor_state['left']['error'])),
                'voltage': f"{self.motor_state['left']['voltage']:.1f}",
                'current': f"{self.motor_state['left']['current']:.1f}",
            },
            'right': {
                'state':   self.motor_state['right']['state'],
                'error':   hex(int(self.motor_state['right']['error'])),
                'voltage': f"{self.motor_state['right']['voltage']:.1f}",
                'current': f"{self.motor_state['right']['current']:.1f}",
            },
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
        node.get_logger().info('Shutting down... Putting motors in IDLE.')
        if node.use_usb_left and node.left_serial and node.left_serial.is_open:
            try:
                node.left_serial.write(b"v 0 0.0\n")
                time.sleep(0.05)
                node.left_serial.write(b"w axis0.requested_state 1\n")
                node.left_serial.close()
            except:
                pass
                
        try:
            import can
            import struct
            bus = can.interface.Bus(channel='can0', bustype='socketcan')
            
            if not node.use_usb_left:
                # Left motor IDLE
                msg_l = can.Message(arbitration_id=(node.left_id << 5) | 0x07, data=list(struct.pack('<I', 1)), is_extended_id=False)
                bus.send(msg_l)
            
            # Right motor IDLE
            msg_r = can.Message(arbitration_id=(node.right_id << 5) | 0x07, data=list(struct.pack('<I', 1)), is_extended_id=False)
            bus.send(msg_r)
            
            node.get_logger().info('Successfully sent IDLE commands to motor(s) via CAN.')
        except Exception as e:
            node.get_logger().error(f'Failed to send raw CAN IDLE command: {e}')
            
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
