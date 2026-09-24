#!/usr/bin/env python3
# NOTE: NEVER call time.sleep() on the main Qt GUI thread.
# All blocking work runs in QThread (WorkerThread / PowerPollThread).
# Use QTimer.singleShot() for delayed GUI actions.
import sys
import os
import time
import math
import struct
import termios
import subprocess
import signal
import glob
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QLabel, QPushButton, QStackedWidget, QHBoxLayout,
                             QProgressBar, QFrame, QScrollArea, QLineEdit, QDialog)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QFont, QPalette, QColor
import can
import shutil
import json

# If ROS environment is not sourced, self-bootstrap via bash to guarantee rclpy & ROS 2 libraries
if "ROS_DISTRO" not in os.environ:
    os.environ["ROS_DOMAIN_ID"] = "42"
    bootstrap_cmd = (
        "source /opt/ros/jazzy/setup.bash 2>/dev/null && "
        "source /home/hospobot/hospobot_ws/install/setup.bash 2>/dev/null && "
        "export ROS_DOMAIN_ID=42 && "
        f"exec /home/hospobot/hospobot_ws/venv/bin/python3 -u {os.path.abspath(__file__)} " + " ".join(f'"{a}"' for a in sys.argv[1:])
    )
    os.execv("/bin/bash", ["/bin/bash", "-c", bootstrap_cmd])

if "ROS_DOMAIN_ID" not in os.environ:
    os.environ["ROS_DOMAIN_ID"] = "42"

try:
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseWithCovarianceStamped
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

# ──────────────────────────────────────────────────────────────────────────────
# ODrive serial helper (O_NONBLOCK, same as before)
# ──────────────────────────────────────────────────────────────────────────────
class OdriveSerial:
    def __init__(self, port, timeout=0.4):
        self.port = port
        self.timeout = timeout
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0; attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | termios.B115200
        attrs[3] = 0; attrs[4] = termios.B115200; attrs[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        time.sleep(0.3)
        self._drain()

    def _drain(self):
        try:
            while True:
                if not os.read(self.fd, 4096): break
        except BlockingIOError:
            pass

    def write(self, cmd: str):
        os.write(self.fd, (cmd + '\n').encode('ascii'))

    def query(self, cmd: str) -> str:
        self._drain()
        os.write(self.fd, (cmd + '\n').encode('ascii'))
        buf = b''
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            try:
                buf += os.read(self.fd, 256)
                if b'\n' in buf:
                    line = buf.split(b'\n')[0].decode('ascii', errors='ignore').strip()
                    if line.endswith('d'): line = line[:-1]
                    return line
            except BlockingIOError:
                time.sleep(0.01)
        return ''

    def close(self):
        try: os.close(self.fd)
        except Exception: pass

    def __enter__(self): return self
    def __exit__(self, *_): self.close()


def find_odrive_port() -> str:
    for port in sorted(glob.glob('/dev/ttyACM*')):
        try:
            with OdriveSerial(port, timeout=0.5) as s:
                resp = s.query('r vbus_voltage')
                if resp and resp.replace('.', '', 1).lstrip('-').isdigit():
                    return port
        except Exception:
            continue
    return ''


# ──────────────────────────────────────────────────────────────────────────────
# Power polling thread — reads directly from serial (no ROS dependency)
# Falls back to ros2 topic echo once the launch stack is running.
# Uses O_NONBLOCK so it never hangs, and silently skips if port is busy.
# ──────────────────────────────────────────────────────────────────────────────
class PowerPollThread(QThread):
    power_updated = pyqtSignal(float, float)   # voltage, current

    def __init__(self, app=None):
        super().__init__()
        self.app = app
        self._running = True
        self._last_volt = 0.0
        self._last_curr = 0.0

    def run(self):
        while self._running:
            volt, curr = self._try_ros_read()
            # Do NOT touch serial port if launch process is active, to prevent bus collision with odrive_can_node
            is_launch_active = bool(self.app and self.app._launch_proc and self.app._launch_proc.poll() is None)
            if volt <= 0.0 and not is_launch_active:
                volt, curr = self._try_serial_read()
            if volt > 0.0:
                self._last_volt = volt
                self._last_curr = curr
            self.power_updated.emit(self._last_volt, self._last_curr)
            time.sleep(2.0)

    def _try_serial_read(self):
        """Open serial with O_NONBLOCK, query voltage + current, close. 
        Returns (0,0) if port is busy or not available."""
        volt, curr = 0.0, 0.0
        port = ''
        fd = -1
        try:
            ports = sorted(glob.glob('/dev/ttyACM*'))
            if not ports:
                return 0.0, 0.0
            port = ports[0]
            # O_NONBLOCK so open() never blocks on a busy port
            fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0; attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | termios.B115200
            attrs[3] = 0; attrs[4] = termios.B115200; attrs[5] = termios.B115200
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            time.sleep(0.05)

            def _drain():
                try:
                    while True:
                        if not os.read(fd, 4096): break
                except (BlockingIOError, OSError): pass

            def _query(cmd, timeout=0.3):
                _drain()
                os.write(fd, (cmd + '\n').encode('ascii'))
                buf = b''
                deadline = time.time() + timeout
                while time.time() < deadline:
                    try:
                        buf += os.read(fd, 256)
                        if b'\n' in buf:
                            line = buf.split(b'\n')[0].decode('ascii', errors='ignore').strip()
                            if line.endswith('d'): line = line[:-1]
                            return line
                    except BlockingIOError:
                        time.sleep(0.005)
                return ''

            v_str = _query('r vbus_voltage')
            try:
                v = float(v_str)
                if 0.0 < v < 60.0:
                    volt = v
            except ValueError:
                pass

            i_str = _query('r axis0.motor.current_control.Iq_measured')
            try:
                curr = abs(float(i_str))
            except ValueError:
                pass

        except (OSError, BlockingIOError, termios.error):
            # Port is busy or unavailable — silently skip
            pass
        except Exception:
            pass
        finally:
            if fd >= 0:
                try: os.close(fd)
                except OSError: pass
        return volt, curr

    def _try_ros_read(self):
        """Read power stats directly from shared cache written by odrive_can_node.
        Avoids spawning ros2 CLI processes that flood DDS network discovery."""
        volt, curr = 0.0, 0.0
        try:
            power_file = '/tmp/hospobot_power.json'
            if os.path.exists(power_file):
                with open(power_file, 'r') as pf:
                    data = json.load(pf)
                # Ensure data is recent (within 5 seconds)
                if time.time() - data.get('timestamp', 0) < 5.0:
                    volt = float(data.get('voltage', 0.0))
                    curr = float(data.get('current', 0.0))
        except Exception:
            pass
        return volt, curr

    def stop(self):
        self._running = False
        self.wait(5000)


# ──────────────────────────────────────────────────────────────────────────────
# ROS 2 Remote Web Synchronization Thread
# Listens for remote launch commands (/hospobot/set_mode & /hospobot/mode_cmd)
# Broadcasts active robot state (/hospobot/current_mode & /hospobot/mode_status)
# Auto-localizes AMCL upon launching Auto Nav mode
# ──────────────────────────────────────────────────────────────────────────────
class DesktopRosSyncThread(QThread):
    mode_command_received = pyqtSignal(str, str)  # (mode, map_name)
    localized_signal = pyqtSignal(float, float, float)  # (x, y, yaw)

    def __init__(self, main_win=None):
        super().__init__()
        self.main_win = main_win
        self._running = True
        self._node = None
        self._status_pub = None
        self._status_pub_latched = None
        self._mode_status_pub = None
        self._initpose_pub = None
        self._initpose_pub_volatile = None

    def run(self):
        with open("/tmp/debug_sync.log", "a") as f:
            f.write(f"DesktopRosSyncThread.run started! HAVE_ROS={HAVE_ROS}\n")
        if not HAVE_ROS:
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self._node = rclpy.create_node('hospobot_desktop_sync')
            with open("/tmp/debug_sync.log", "a") as f:
                f.write(f"Node hospobot_desktop_sync created!\n")

            qos_latched = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE
            )

            self._status_pub = self._node.create_publisher(String, '/hospobot/current_mode', 10)
            self._status_pub_latched = self._node.create_publisher(String, '/hospobot/current_mode_latched', qos_latched)
            self._mode_status_pub = self._node.create_publisher(String, '/hospobot/mode_status', qos_latched)
            self._initpose_pub = self._node.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos_latched)
            self._initpose_pub_volatile = self._node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

            def _on_cmd(msg):
                try:
                    payload = json.loads(msg.data)
                    mode = str(payload.get('mode', '')).strip().lower()
                    map_name = str(payload.get('map', '')).strip()
                except Exception:
                    mode = msg.data.strip().lower()
                    map_name = ''
                self.mode_command_received.emit(mode, map_name)

            self._node.create_subscription(String, '/hospobot/set_mode', _on_cmd, 10)
            self._node.create_subscription(String, '/hospobot/mode_cmd', _on_cmd, 10)

            def _on_amcl_pose(msg):
                try:
                    px = msg.pose.pose.position.x
                    py = msg.pose.pose.position.y
                    qz = msg.pose.pose.orientation.z
                    qw = msg.pose.pose.orientation.w
                    yaw = 2.0 * math.atan2(qz, qw)
                    self.localized_signal.emit(px, py, yaw)
                except Exception:
                    pass

            self._node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', _on_amcl_pose, 10)

            while self._running and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.1)
        except Exception as e:
            print(f"[DesktopRosSync] Error: {e}")
        finally:
            if self._node:
                try:
                    self._node.destroy_node()
                except Exception:
                    pass

    def publish_status(self, mode: str, map_name: str, state_text: str, motors: bool):
        if not self._node:
            return
        mode_val = mode if mode else "idle"
        # 1. Plain string for simple subscribers
        msg_simple = String()
        msg_simple.data = mode_val
        try:
            if self._status_pub:
                self._status_pub.publish(msg_simple)
            if self._status_pub_latched:
                self._status_pub_latched.publish(msg_simple)
        except Exception:
            pass

        # 2. Rich JSON status
        status_dict = {
            "mode": mode_val,
            "map": map_name if map_name else "Building6-Floor1",
            "state": state_text,
            "motors": motors,
            "timestamp": time.time()
        }
        msg_rich = String()
        msg_rich.data = json.dumps(status_dict)
        try:
            if self._mode_status_pub:
                self._mode_status_pub.publish(msg_rich)
        except Exception:
            pass

    def pulse_initial_pose(self):
        """Pushes initial pose (0, 0, 0) to AMCL on /initialpose with both latched and volatile QoS."""
        if not self._node:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = 0.25      # X covariance (0.5m)
        cov[7] = 0.25      # Y covariance (0.5m)
        cov[35] = 0.0685   # Yaw covariance (~15 deg)
        msg.pose.covariance = cov
        try:
            if self._initpose_pub:
                self._initpose_pub.publish(msg)
            if self._initpose_pub_volatile:
                self._initpose_pub_volatile.publish(msg)
            print("[DesktopRosSync] Auto-localization: published /initialpose (x=0, y=0, yaw=0)")
        except Exception as e:
            print(f"[DesktopRosSync] Error publishing initial pose: {e}")

    def stop(self):
        self._running = False
        self.wait(1000)


# ──────────────────────────────────────────────────────────────────────────────
# Worker thread (hardware checks, index search, drive calibration)
# ──────────────────────────────────────────────────────────────────────────────
class WorkerThread(QThread):

    progress        = pyqtSignal(str, str, int)
    finished_checks = pyqtSignal(bool)
    finished_search = pyqtSignal(bool, str)
    finished_calib  = pyqtSignal(bool, str)

    def __init__(self, mode="check"):
        super().__init__()
        self.mode = mode
        self._left_port = ''
        self._ser = None
        self._bus = None

    def run(self):
        if   self.mode == "check":       self.run_checks()
        elif self.mode == "search":      self.run_search()
        elif self.mode == "drive_calib": self.run_drive_calib()

    # ── Hardware checks ────────────────────────────────────────────────────────
    def run_checks(self):
        self.progress.emit("Detecting Left Motor (USB)...", "yellow", 10)
        left_port = find_odrive_port()
        if not left_port:
            self.progress.emit("Left Motor not found on any /dev/ttyACM*!", "red", 10)
            self.finished_checks.emit(False)
            return
        self._left_port = left_port
        self.progress.emit(f"Left Motor found on {left_port} ✓", "green", 20)
        time.sleep(0.5)

        self.progress.emit("Bringing up CAN interface...", "yellow", 35)
        subprocess.run(["sudo", "ip", "link", "set", "can0", "down"], capture_output=True)
        up_result = subprocess.run(
            ["sudo", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "500000"],
            capture_output=True
        )
        if up_result.returncode != 0:
            subprocess.run(["sudo", "ip", "link", "set", "can0", "up"], capture_output=True)
        subprocess.run(["sudo", "ip", "link", "set", "can0", "txqueuelen", "1000"], capture_output=True)
        time.sleep(0.5)

        self.progress.emit("Checking Right Motor (CAN)...", "yellow", 40)
        right_ok = False
        try:
            with can.interface.Bus(channel='can0', interface='socketcan') as bus:
                while True:
                    msg = bus.recv(timeout=1.0)
                    if msg is None: continue
                    node_id = (msg.arbitration_id >> 5) & 0x3F
                    cmd_id  = msg.arbitration_id & 0x1F
                    if node_id == 2 and cmd_id == 0x001:
                        right_ok = True
                        break
        except Exception:
            pass

        if not right_ok:
            self.progress.emit("Right Motor (CAN) Not Found! Check power & wiring.", "red", 40)
            self.finished_checks.emit(False)
            return

        self.progress.emit("Checking OAK-D Pro Camera...", "yellow", 70)
        time.sleep(0.5)
        oakd_ok = False
        try:
            res = subprocess.run(["lsusb"], capture_output=True, text=True)
            if "Movidius" in res.stdout or "03e7:" in res.stdout or "MyriadX" in res.stdout:
                oakd_ok = True
        except Exception:
            pass

        if not oakd_ok:
            self.progress.emit("OAK-D Pro Camera Not Found!", "red", 70)
            self.finished_checks.emit(False)
            return

        self.progress.emit("All Hardware Verified Successfully ✓", "green", 100)
        time.sleep(1)
        self.finished_checks.emit(True)

    # ── Index search ───────────────────────────────────────────────────────────
    def run_search(self):
        left_port = self._left_port or find_odrive_port()
        if not left_port:
            self.finished_search.emit(False, "Left motor USB port not found")
            return

        self.progress.emit("Clearing errors and going IDLE...", "yellow", 5)
        try:
            ser = OdriveSerial(left_port, timeout=0.5)
            ser.write('sc'); time.sleep(0.1)
            ser.write('w axis0.requested_state 1'); time.sleep(0.1)
            ser.write('sc')
        except Exception as e:
            self.finished_search.emit(False, f"Failed Left Serial: {e}")
            return

        try:
            bus = can.interface.Bus(channel='can0', interface='socketcan')
            bus.send(can.Message(arbitration_id=(2 << 5) | 0x018, data=[], is_extended_id=False))
            time.sleep(0.1)
            bus.send(can.Message(arbitration_id=(2 << 5) | 0x007,
                                 data=list(struct.pack('<I', 1)), is_extended_id=False))
        except Exception as e:
            self.finished_search.emit(False, f"Failed Right CAN: {e}")
            ser.close(); return

        time.sleep(1.0)

        # RIGHT WHEEL index search
        self.progress.emit("Searching Index: Right Wheel...", "yellow", 25)
        bus.send(can.Message(arbitration_id=(2 << 5) | 0x018, data=[], is_extended_id=False))
        time.sleep(0.1)
        t_drain = time.time()
        while time.time() - t_drain < 0.2: bus.recv(0.02)
        bus.send(can.Message(arbitration_id=(2 << 5) | 0x007,
                             data=list(struct.pack('<I', 6)), is_extended_id=False))

        entered_right = False
        start_t = time.time()
        while time.time() - start_t < 3.0:
            msg = bus.recv(0.05)
            if msg and ((msg.arbitration_id >> 5) & 0x3F) == 2 and (msg.arbitration_id & 0x1F) == 0x001:
                axis_error = struct.unpack_from('<I', bytes(msg.data), 0)[0]
                axis_state = msg.data[4]
                if axis_error != 0:
                    self.finished_search.emit(False, f"Right Wheel Error: 0x{axis_error:X}")
                    ser.close()
                    try: bus.shutdown()
                    except: pass
                    return
                if axis_state == 6: entered_right = True; break

        if entered_right:
            spin_t = time.time()
            while time.time() - spin_t < 15.0:
                msg = bus.recv(0.05)
                if msg and ((msg.arbitration_id >> 5) & 0x3F) == 2 and (msg.arbitration_id & 0x1F) == 0x001:
                    axis_error = struct.unpack_from('<I', bytes(msg.data), 0)[0]
                    axis_state = msg.data[4]
                    if axis_error != 0:
                        self.finished_search.emit(False, f"Right Wheel Search Error: 0x{axis_error:X}")
                        ser.close()
                        try: bus.shutdown()
                        except: pass
                        return
                    if axis_state != 6: break

        # LEFT WHEEL index search
        self.progress.emit("Searching Index: Left Wheel...", "yellow", 55)
        ser.write('sc'); ser.write('w axis0.error 0')
        ser.write('w axis0.encoder.config.use_index 1')
        ser.write('w axis0.config.calibration_lockin.current 20.0')
        ser.write('w axis0.requested_state 6')

        entered_left = False
        start_t = time.time()
        while time.time() - start_t < 3.0:
            res = ser.query('r axis0.current_state')
            if res.isdigit() and int(res) == 6: entered_left = True; break
            time.sleep(0.05)

        if entered_left:
            spin_t = time.time()
            while time.time() - spin_t < 20.0:
                res = ser.query('r axis0.current_state')
                if res.isdigit() and int(res) != 6: break
                time.sleep(0.2)

        left_ready = ser.query('r axis0.encoder.is_ready')
        if left_ready not in ['1', 'True']:
            left_err = ser.query('r axis0.error')
            self.finished_search.emit(False, f"Left Wheel: Index not detected (err={left_err}). Check encoder cable.")
            ser.close()
            try: bus.shutdown()
            except: pass
            return

        # Enter closed loop (State 8) — needed if user chooses drive calibration
        self.progress.emit("Entering Closed Loop Control...", "yellow", 85)
        bus.send(can.Message(arbitration_id=(2 << 5) | 0x00B,
                             data=list(struct.pack('<ii', 2, 1)), is_extended_id=False))
        time.sleep(0.05)
        bus.send(can.Message(arbitration_id=(2 << 5) | 0x01B,
                             data=list(struct.pack('<ff', 0.8, 4.0)), is_extended_id=False))
        time.sleep(0.05)
        bus.send(can.Message(arbitration_id=(2 << 5) | 0x007,
                             data=list(struct.pack('<I', 8)), is_extended_id=False))

        ser.write('w axis0.controller.config.control_mode 2')
        ser.write('w axis0.controller.config.input_mode 1')
        ser.write('w axis0.controller.config.vel_gain 0.8')
        ser.write('w axis0.controller.config.vel_integrator_gain 4.0')
        ser.write('w axis0.requested_state 8')
        time.sleep(0.5)

        right_in_8 = False
        t_verify = time.time()
        while time.time() - t_verify < 2.0:
            msg = bus.recv(0.05)
            if msg and ((msg.arbitration_id >> 5) & 0x3F) == 2 and (msg.arbitration_id & 0x1F) == 0x001:
                r_err, r_state = struct.unpack_from('<IB', bytes(msg.data), 0)
                if r_state == 8: right_in_8 = True; break

        left_st8 = ser.query('r axis0.current_state')
        if left_st8 != '8':
            left_err8 = ser.query('r axis0.error')
            self.finished_search.emit(False, f"Left Wheel failed Closed Loop (state={left_st8}, err={left_err8})")
            ser.close()
            try: bus.shutdown()
            except: pass
            return

        if not right_in_8:
            self.finished_search.emit(False, "Right Wheel failed to enter Closed Loop Control (State 8)")
            ser.close()
            try: bus.shutdown()
            except: pass
            return

        # Save open handles — drive calibration will use them if chosen
        self._ser = ser
        self._bus = bus
        self.progress.emit("Index Search Complete! ✓", "green", 100)
        time.sleep(0.5)
        self.finished_search.emit(True, "")

    # ── Drive Calibration ──────────────────────────────────────────────────────
    def run_drive_calib(self):
        """
        Wheel geometry: track_width=0.300 m, wheel_radius=0.0625 m
          circ = 2π × 0.0625 = 0.3927 m
          0.5 m/s → 0.5 / 0.3927 ≈ 1.274 turns/s
          360° at ω=1.5 rad/s: each wheel = ±1.5×0.150/0.3927 = ±0.573 turns/s, 2π/1.5 ≈ 4.19 s
        Left motor drive direction is inverted.
        """
        ser = self._ser
        bus = self._bus
        if ser is None or bus is None:
            left_port = self._left_port or find_odrive_port()
            if not left_port:
                self.finished_calib.emit(False, "Cannot open serial port for calibration")
                return
            try:
                ser = OdriveSerial(left_port, timeout=0.5)
                bus = can.interface.Bus(channel='can0', interface='socketcan')
            except Exception as e:
                self.finished_calib.emit(False, f"Port open error: {e}")
                return

        WHEEL_CIRC = 2.0 * math.pi * 0.0625
        TRACK_W    = 0.300
        MAX_VEL    = 1.5

        def _clamp(v): return max(-MAX_VEL, min(MAX_VEL, v))

        def _set_vel(v_x, omega, duration):
            v_l = (v_x - omega * TRACK_W / 2.0) / WHEEL_CIRC
            v_r = (v_x + omega * TRACK_W / 2.0) / WHEEL_CIRC
            v_l = -_clamp(v_l)
            v_r =  _clamp(v_r)
            bus.send(can.Message(arbitration_id=(2 << 5) | 0x00D,
                                 data=list(struct.pack('<ff', v_r, 0.0)), is_extended_id=False))
            ser.write(f'v 0 {v_l:.4f}')
            time.sleep(duration)

        def _stop():
            bus.send(can.Message(arbitration_id=(2 << 5) | 0x00D,
                                 data=list(struct.pack('<ff', 0.0, 0.0)), is_extended_id=False))
            ser.write('v 0 0')
            time.sleep(0.3)

        try:
            self.progress.emit("Drive Calibration: Forward 0.5 m/s...", "yellow", 20)
            _set_vel(0.5, 0.0, 2.0)
            _stop()
            time.sleep(0.5)

            spin_duration = (2.0 * math.pi) / 1.5
            self.progress.emit("Drive Calibration: Rotating 360°...", "yellow", 55)
            _set_vel(0.0, 1.5, spin_duration)
            _stop()
            time.sleep(0.5)

            self.progress.emit("Drive Calibration: Reverse 0.5 m/s...", "yellow", 75)
            _set_vel(-0.5, 0.0, 2.0)
            _stop()
            time.sleep(0.5)

            self.progress.emit("Drive Calibration: Returning to IDLE...", "yellow", 90)
            bus.send(can.Message(arbitration_id=(2 << 5) | 0x007,
                                 data=list(struct.pack('<I', 1)), is_extended_id=False))
            ser.write('w axis0.requested_state 1')
            time.sleep(0.5)

            self.progress.emit("Drive Calibration Complete! ✓", "green", 100)
            time.sleep(1.0)
            self.finished_calib.emit(True, "")

        except Exception as e:
            self.finished_calib.emit(False, f"Calibration error: {e}")
        finally:
            try: ser.close()
            except: pass
            try: bus.shutdown()
            except: pass

    def skip_to_idle(self):
        """Put both motors to IDLE without running drive calibration."""
        ser = self._ser
        bus = self._bus
        if ser is None and bus is None:
            return
        try:
            if bus:
                bus.send(can.Message(arbitration_id=(2 << 5) | 0x007,
                                     data=list(struct.pack('<I', 1)), is_extended_id=False))
            if ser:
                ser.write('w axis0.requested_state 1')
            time.sleep(0.3)
        except Exception:
            pass
        finally:
            try: ser.close() if ser else None
            except: pass
            try: bus.shutdown() if bus else None
            except: pass


# ──────────────────────────────────────────────────────────────────────────────
# Map Save Dialogs & Background Worker
# ──────────────────────────────────────────────────────────────────────────────
class FinishMappingChoiceDialog(QDialog):
    def __init__(self, original_name, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Finish Mapping")
        self.setFixedSize(520, 270)
        self.setStyleSheet("background-color: #1e1e1e; color: white; border-radius: 12px;")
        self.setWindowFlags(self.windowFlags() | Qt.WindowType.FramelessWindowHint)
        self.choice = None  # 'overwrite', 'new'

        layout = QVBoxLayout(self)
        layout.setContentsMargins(30, 25, 30, 25)
        layout.setSpacing(14)

        title = QLabel("FINISH MAPPING")
        title.setFont(QFont("Inter", 20, QFont.Weight.Bold))
        title.setStyleSheet("color: #03dac6;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        info = QLabel(f"You loaded previous map: '{original_name}'\nHow would you like to save this map?")
        info.setFont(QFont("Inter", 14))
        info.setAlignment(Qt.AlignmentFlag.AlignCenter)
        info.setStyleSheet("color: #cccccc;")
        layout.addWidget(info)

        btn_orig = QPushButton(f"Save Map as '{original_name}'")
        btn_orig.setFont(QFont("Inter", 14, QFont.Weight.Bold))
        btn_orig.setFixedHeight(48)
        btn_orig.setStyleSheet(
            "QPushButton { background-color: #03dac6; color: #000; border-radius: 10px; font-weight: bold; }"
            "QPushButton:hover { background-color: #00f5e4; }"
        )
        btn_orig.clicked.connect(self._select_orig)
        layout.addWidget(btn_orig)

        btn_new = QPushButton("Save Map as New")
        btn_new.setFont(QFont("Inter", 14, QFont.Weight.Bold))
        btn_new.setFixedHeight(48)
        btn_new.setStyleSheet(
            "QPushButton { background-color: #bb86fc; color: #000; border-radius: 10px; font-weight: bold; }"
            "QPushButton:hover { background-color: #cf9fff; }"
        )
        btn_new.clicked.connect(self._select_new)
        layout.addWidget(btn_new)

        btn_cancel = QPushButton("Cancel / Continue Mapping")
        btn_cancel.setFont(QFont("Inter", 12))
        btn_cancel.setFixedHeight(34)
        btn_cancel.setStyleSheet(
            "QPushButton { background-color: #2a2a2a; color: #aaaaaa; border-radius: 8px; border: 1px solid #444; }"
            "QPushButton:hover { color: #ffffff; background-color: #383838; }"
        )
        btn_cancel.clicked.connect(self.reject)
        layout.addWidget(btn_cancel)

    def _select_orig(self):
        self.choice = 'overwrite'
        self.accept()

    def _select_new(self):
        self.choice = 'new'
        self.accept()


class MapNameInputDialog(QDialog):
    def __init__(self, default_name="", parent=None):
        super().__init__(parent)
        self.setWindowTitle("Save Map")
        self.setFixedSize(760, 480)
        self.setStyleSheet("background-color: #1a1a1a; color: white; border-radius: 14px;")
        self.setWindowFlags(self.windowFlags() | Qt.WindowType.FramelessWindowHint)
        self.map_name = ""
        self.is_caps = False
        self.letter_buttons = []

        layout = QVBoxLayout(self)
        layout.setContentsMargins(25, 20, 25, 20)
        layout.setSpacing(10)

        title = QLabel("SAVE MAP AS NEW")
        title.setFont(QFont("Inter", 18, QFont.Weight.Bold))
        title.setStyleSheet("color: #03dac6;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        self.txt_name = QLineEdit(default_name)
        self.txt_name.setFont(QFont("Inter", 18, QFont.Weight.Bold))
        self.txt_name.setPlaceholderText("tap keys below to type map name...")
        self.txt_name.setFixedHeight(48)
        self.txt_name.setStyleSheet(
            "QLineEdit { background-color: #262626; color: #00e676; border: 2px solid #03dac6; "
            "border-radius: 8px; padding: 0 12px; }"
        )
        self.txt_name.returnPressed.connect(self._do_save)
        layout.addWidget(self.txt_name)

        # Virtual Keyboard Container
        kb_layout = QVBoxLayout()
        kb_layout.setSpacing(6)

        key_style = (
            "QPushButton { background-color: #2a2a2a; color: white; border: 1px solid #444; "
            "border-radius: 6px; font-size: 16px; font-weight: bold; min-height: 42px; }"
            "QPushButton:hover { background-color: #383838; border-color: #03dac6; }"
            "QPushButton:pressed { background-color: #03dac6; color: black; }"
        )

        def _make_key(ch, width=52, custom_style=None):
            b = QPushButton(ch)
            b.setFixedWidth(width)
            b.setStyleSheet(custom_style or key_style)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            return b

        # Row 1: Numbers & symbols
        r1 = QHBoxLayout(); r1.setSpacing(5); r1.setAlignment(Qt.AlignmentFlag.AlignCenter)
        for ch in ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "-", "_"]:
            k = _make_key(ch)
            k.clicked.connect(lambda _, c=ch: self._key_pressed(c))
            r1.addWidget(k)
        kb_layout.addLayout(r1)

        # Row 2: QWERTY
        r2 = QHBoxLayout(); r2.setSpacing(5); r2.setAlignment(Qt.AlignmentFlag.AlignCenter)
        for ch in ["q", "w", "e", "r", "t", "y", "u", "i", "o", "p"]:
            k = _make_key(ch)
            k.clicked.connect(lambda _, c=ch: self._letter_pressed(c))
            r2.addWidget(k)
            self.letter_buttons.append((k, ch))
        kb_layout.addLayout(r2)

        # Row 3: ASDFGHJKL
        r3 = QHBoxLayout(); r3.setSpacing(5); r3.setAlignment(Qt.AlignmentFlag.AlignCenter)
        for ch in ["a", "s", "d", "f", "g", "h", "j", "k", "l"]:
            k = _make_key(ch)
            k.clicked.connect(lambda _, c=ch: self._letter_pressed(c))
            r3.addWidget(k)
            self.letter_buttons.append((k, ch))
        kb_layout.addLayout(r3)

        # Row 4: Caps, ZXCVBNM, Backspace
        r4 = QHBoxLayout(); r4.setSpacing(5); r4.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.btn_caps = _make_key("⇧ CAPS", width=88)
        self.btn_caps.clicked.connect(self._toggle_caps)
        r4.addWidget(self.btn_caps)

        for ch in ["z", "x", "c", "v", "b", "n", "m"]:
            k = _make_key(ch)
            k.clicked.connect(lambda _, c=ch: self._letter_pressed(c))
            r4.addWidget(k)
            self.letter_buttons.append((k, ch))

        btn_bs = _make_key("⌫ DEL", width=95, custom_style=(
            "QPushButton { background-color: #3b2828; color: #ffb74d; border: 1px solid #663333; "
            "border-radius: 6px; font-size: 15px; font-weight: bold; min-height: 42px; }"
            "QPushButton:hover { background-color: #4d3333; border-color: #ffb74d; }"
            "QPushButton:pressed { background-color: #ffb74d; color: black; }"
        ))
        btn_bs.clicked.connect(self._backspace_pressed)
        r4.addWidget(btn_bs)
        kb_layout.addLayout(r4)

        # Row 5: Actions
        r5 = QHBoxLayout(); r5.setSpacing(8); r5.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        btn_clr = _make_key("CLEAR", width=90, custom_style=(
            "QPushButton { background-color: #2c2c2c; color: #cf6679; border: 1px solid #444; "
            "border-radius: 6px; font-size: 14px; font-weight: bold; min-height: 44px; }"
            "QPushButton:hover { background-color: #3d2222; border-color: #cf6679; }"
        ))
        btn_clr.clicked.connect(self._clear_pressed)
        r5.addWidget(btn_clr)

        btn_space = _make_key("SPACE ( _ )", width=220, custom_style=(
            "QPushButton { background-color: #333333; color: white; border: 1px solid #555; "
            "border-radius: 6px; font-size: 14px; font-weight: bold; min-height: 44px; }"
            "QPushButton:hover { background-color: #444444; border-color: #03dac6; }"
            "QPushButton:pressed { background-color: #03dac6; color: black; }"
        ))
        btn_space.clicked.connect(lambda: self._key_pressed("_"))
        r5.addWidget(btn_space)

        btn_cancel = _make_key("CANCEL", width=120, custom_style=(
            "QPushButton { background-color: #2a2a2a; color: #aaaaaa; border: 1px solid #444; "
            "border-radius: 6px; font-size: 14px; font-weight: bold; min-height: 44px; }"
            "QPushButton:hover { background-color: #383838; color: white; }"
        ))
        btn_cancel.clicked.connect(self.reject)
        r5.addWidget(btn_cancel)

        btn_save = _make_key("💾 SAVE MAP", width=180, custom_style=(
            "QPushButton { background-color: #03dac6; color: black; border-radius: 6px; "
            "font-size: 16px; font-weight: bold; min-height: 44px; }"
            "QPushButton:hover { background-color: #00f5e4; }"
            "QPushButton:pressed { background-color: #018f84; }"
        ))
        btn_save.clicked.connect(self._do_save)
        r5.addWidget(btn_save)

        kb_layout.addLayout(r5)
        layout.addLayout(kb_layout)

    def _key_pressed(self, char):
        self.txt_name.setText(self.txt_name.text() + char)

    def _letter_pressed(self, char):
        actual_char = char.upper() if self.is_caps else char.lower()
        self.txt_name.setText(self.txt_name.text() + actual_char)

    def _backspace_pressed(self):
        curr = self.txt_name.text()
        if curr:
            self.txt_name.setText(curr[:-1])

    def _clear_pressed(self):
        self.txt_name.setText("")

    def _toggle_caps(self):
        self.is_caps = not self.is_caps
        self.btn_caps.setStyleSheet(
            "QPushButton { background-color: #00e676; color: black; border-radius: 6px; "
            "font-size: 15px; font-weight: bold; min-height: 42px; }" if self.is_caps else
            "QPushButton { background-color: #2a2a2a; color: white; border: 1px solid #444; "
            "border-radius: 6px; font-size: 15px; font-weight: bold; min-height: 42px; }"
        )
        for btn, char in self.letter_buttons:
            btn.setText(char.upper() if self.is_caps else char.lower())

    def _do_save(self):
        raw = self.txt_name.text().strip()
        cleaned = os.path.splitext(raw)[0].strip().replace(" ", "_")
        if cleaned:
            self.map_name = cleaned
            self.accept()


class MapSavedConfirmationDialog(QDialog):
    def __init__(self, title, message, success=True, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setFixedSize(500, 200)
        self.setStyleSheet("background-color: #1e1e1e; color: white; border-radius: 12px;")
        self.setWindowFlags(self.windowFlags() | Qt.WindowType.FramelessWindowHint)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(30, 25, 30, 25)
        layout.setSpacing(15)

        lbl_title = QLabel(title)
        lbl_title.setFont(QFont("Inter", 18, QFont.Weight.Bold))
        lbl_title.setStyleSheet(f"color: {'#00e676' if success else '#cf6679'};")
        lbl_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(lbl_title)

        lbl_msg = QLabel(message)
        lbl_msg.setFont(QFont("Inter", 14))
        lbl_msg.setWordWrap(True)
        lbl_msg.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_msg.setStyleSheet("color: #dddddd;")
        layout.addWidget(lbl_msg)

        btn_ok = QPushButton("OK")
        btn_ok.setFixedHeight(42)
        btn_ok.setStyleSheet("QPushButton { background-color: #03dac6; color: black; font-weight: bold; border-radius: 8px; }")
        btn_ok.clicked.connect(self.accept)
        layout.addWidget(btn_ok)


class MapSaveWorker(QThread):
    finished_save = pyqtSignal(bool, str)

    def __init__(self, target_name: str):
        super().__init__()
        self.target_name = target_name

    def run(self):
        global_maps_dir = "/home/hospobot/hospobot_ws/global_maps"
        os.makedirs(global_maps_dir, exist_ok=True)
        target_prefix = os.path.join(global_maps_dir, self.target_name)

        # 1. Run map_saver_cli to save .yaml and .pgm
        domain_id = os.environ.get("ROS_DOMAIN_ID", "42")
        cmd = (
            f"bash -c 'source /opt/ros/jazzy/setup.bash && "
            f"source /home/hospobot/hospobot_ws/install/setup.bash && "
            f"export ROS_DOMAIN_ID={domain_id} && "
            f"ros2 run nav2_map_server map_saver_cli -f \"{target_prefix}\"'"
        )
        try:
            res = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=20)
            yaml_path = f"{target_prefix}.yaml"
            if not os.path.exists(yaml_path):
                err = res.stderr.strip() or res.stdout.strip()
                self.finished_save.emit(False, f"Failed to save map files: {err[:200]}")
                return
        except subprocess.TimeoutExpired:
            self.finished_save.emit(False, "Timed out waiting for map_saver_cli. Ensure ROS mapping node is active.")
            return
        except Exception as e:
            self.finished_save.emit(False, f"Error running map_saver: {e}")
            return

        # 2. Persist active RTAB-Map database
        active_db = "/home/hospobot/hospobot_ws/.active_mapping.db"
        target_db = f"{target_prefix}.db"
        if os.path.exists(active_db):
            try:
                shutil.copyfile(active_db, target_db)
            except Exception as e:
                print(f"Warning copying active db: {e}")

        self.finished_save.emit(True, f"Map '{self.target_name}' successfully saved to global_maps!")


# ──────────────────────────────────────────────────────────────────────────────
# Main Window
# ──────────────────────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hospobot Initialization")
        self.setMinimumSize(960, 540)
        self.resize(1280, 720)
        self.setCursor(Qt.CursorShape.ArrowCursor)

        self._launch_proc = None
        self._power_monitor_proc = None
        self._motors_engaged = False
        self._search_worker = None  # keep reference so handles survive

        self._target_mode = "mapping"
        self._selected_map_name = ""
        self._active_loaded_map = ""
        self._current_nav_mode = ""
        self._countdown_val = 5
        self._countdown_timer = QTimer(self)
        self._countdown_timer.setInterval(1000)
        self._countdown_timer.timeout.connect(self._on_countdown_tick)
        self._save_worker = None
        self._map_button_group = []

        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Window, QColor(18, 18, 18))
        palette.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.white)
        self.setPalette(palette)

        # Root layout: persistent header + page stack
        root_widget = QWidget()
        root_layout = QVBoxLayout(root_widget)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        self.header_bar = self._make_header_bar()
        root_layout.addWidget(self.header_bar)

        self.stack = QStackedWidget()
        root_layout.addWidget(self.stack, stretch=1)
        self.setCentralWidget(root_widget)

        # Pages
        self.page_check         = QWidget(); self.setup_check_page();        self.stack.addWidget(self.page_check)
        self.page_safety        = QWidget(); self.setup_safety_page();       self.stack.addWidget(self.page_safety)
        self.page_search        = QWidget(); self.setup_search_page();       self.stack.addWidget(self.page_search)
        self.page_calib_choice  = QWidget(); self.setup_calib_choice_page(); self.stack.addWidget(self.page_calib_choice)
        self.page_drive_calib   = QWidget(); self.setup_drive_calib_page();  self.stack.addWidget(self.page_drive_calib)
        self.page_launch        = QWidget(); self.setup_launch_page();       self.stack.addWidget(self.page_launch)
        self.page_map_select    = QWidget(); self.setup_map_select_page();   self.stack.addWidget(self.page_map_select)
        self.page_mapping_prep  = QWidget(); self.setup_mapping_prep_page(); self.stack.addWidget(self.page_mapping_prep)
        self.page_idle          = QWidget(); self.setup_idle_page();         self.stack.addWidget(self.page_idle)

        # Power polling (direct serial + ROS fallback)
        self._power_thread = PowerPollThread(self)
        self._power_thread.power_updated.connect(self._on_power_update)
        self._power_thread.start()

        # WireGuard monitoring timer
        self._vpn_is_active = False
        self._vpn_timer = QTimer(self)
        self._vpn_timer.setInterval(3000)
        self._vpn_timer.timeout.connect(self._check_wireguard_status)
        self._vpn_timer.start()
        QTimer.singleShot(300, self._check_wireguard_status)

        # ROS 2 Remote Synchronization Thread
        self._ros_sync = None
        if HAVE_ROS:
            try:
                self._ros_sync = DesktopRosSyncThread(self)
                self._ros_sync.mode_command_received.connect(self._on_remote_mode_cmd)
                self._ros_sync.localized_signal.connect(self._on_robot_localized)
                self._ros_sync.start()
            except Exception as e:
                print(f"Warning initializing DesktopRosSyncThread: {e}")

        # Periodic status broadcast (syncs with web dashboard)
        self._status_broadcast_timer = QTimer(self)
        self._status_broadcast_timer.setInterval(1000)
        self._status_broadcast_timer.timeout.connect(self._broadcast_mode_status)
        self._status_broadcast_timer.start()

        self.showFullScreen()
        QTimer.singleShot(1000, self.start_hardware_check)


    # ── Header bar ─────────────────────────────────────────────────────────────
    def _make_header_bar(self):
        bar = QFrame()
        bar.setFixedHeight(48)
        bar.setStyleSheet("background-color: #0d0d0d; border-bottom: 1px solid #222;")
        h = QHBoxLayout(bar)
        h.setContentsMargins(20, 0, 20, 0)

        lbl = QLabel("HOSPOBOT")
        lbl.setFont(QFont("Inter", 14, QFont.Weight.Bold))
        lbl.setStyleSheet("color: #bb86fc;")
        h.addWidget(lbl)
        h.addStretch()

        self.lbl_volt = QLabel("⚡ --.- V")
        self.lbl_volt.setFont(QFont("Inter", 14, QFont.Weight.Bold))
        self.lbl_volt.setStyleSheet("color: #03dac6;")
        h.addWidget(self.lbl_volt)

        sep = QLabel("  |  ")
        sep.setStyleSheet("color: #444;")
        h.addWidget(sep)

        self.lbl_curr = QLabel("〜 --.- A")
        self.lbl_curr.setFont(QFont("Inter", 14, QFont.Weight.Bold))
        self.lbl_curr.setStyleSheet("color: #ffb74d;")
        h.addWidget(self.lbl_curr)

        sep2 = QLabel("  |  ")
        sep2.setStyleSheet("color: #444;")
        h.addWidget(sep2)

        self.btn_vpn_toggle = QPushButton("🔒 VPN: ...")
        self.btn_vpn_toggle.setFont(QFont("Inter", 12, QFont.Weight.Bold))
        self.btn_vpn_toggle.setFixedHeight(30)
        self.btn_vpn_toggle.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_vpn_toggle.setStyleSheet(
            "QPushButton { background-color: #222; color: #888; border: 1px solid #444; border-radius: 6px; padding: 2px 10px; }"
        )
        self.btn_vpn_toggle.clicked.connect(self._toggle_wireguard)
        h.addWidget(self.btn_vpn_toggle)

        sep3 = QLabel("  |  ")
        sep3.setStyleSheet("color: #444;")
        h.addWidget(sep3)

        self.btn_fullscreen = QPushButton("🗗 WINDOWED")
        self.btn_fullscreen.setFont(QFont("Inter", 12, QFont.Weight.Bold))
        self.btn_fullscreen.setFixedHeight(30)
        self.btn_fullscreen.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_fullscreen.setStyleSheet(
            "QPushButton { background-color: #222; color: #ccc; border: 1px solid #444; border-radius: 6px; padding: 2px 10px; }"
            "QPushButton:hover { background-color: #333; color: #fff; border-color: #03dac6; }"
        )
        self.btn_fullscreen.clicked.connect(self._toggle_fullscreen)
        h.addWidget(self.btn_fullscreen)

        return bar

    def _check_wireguard_status(self):
        try:
            res = subprocess.run(['systemctl', 'is-active', 'wg-quick@wg0'], capture_output=True, text=True, timeout=1.0)
            is_active = (res.stdout.strip() == 'active')
            self._update_vpn_btn(is_active)
        except Exception:
            pass

    def _update_vpn_btn(self, is_active: bool):
        self._vpn_is_active = is_active
        if is_active:
            self.btn_vpn_toggle.setText("🔒 VPN: ON")
            self.btn_vpn_toggle.setStyleSheet(
                "QPushButton { background-color: #14382e; color: #00e676; border: 1px solid #00e676; "
                "border-radius: 6px; padding: 2px 10px; font-weight: bold; }"
                "QPushButton:hover { background-color: #1a4d3e; }"
            )
        else:
            self.btn_vpn_toggle.setText("🔓 VPN: OFF")
            self.btn_vpn_toggle.setStyleSheet(
                "QPushButton { background-color: #381a1a; color: #cf6679; border: 1px solid #cf6679; "
                "border-radius: 6px; padding: 2px 10px; font-weight: bold; }"
                "QPushButton:hover { background-color: #4d2222; }"
            )

    def _toggle_wireguard(self):
        target_action = "stop" if getattr(self, '_vpn_is_active', False) else "start"
        self.btn_vpn_toggle.setEnabled(False)
        self.btn_vpn_toggle.setText("⏳ VPN...")

        class _VpnThread(QThread):
            done = pyqtSignal(bool)
            def __init__(self, action):
                super().__init__()
                self.action = action
            def run(self):
                cmd = f"sudo -n systemctl {self.action} wg-quick@wg0"
                subprocess.run(['bash', '-c', cmd], timeout=6)
                res = subprocess.run(['systemctl', 'is-active', 'wg-quick@wg0'], capture_output=True, text=True, timeout=2)
                self.done.emit(res.stdout.strip() == 'active')

        self._vpn_worker = _VpnThread(target_action)
        def _on_done(is_active):
            self.btn_vpn_toggle.setEnabled(True)
            self._update_vpn_btn(is_active)
        self._vpn_worker.done.connect(_on_done)
        self._vpn_worker.start()

    def _toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
            self.btn_fullscreen.setText("⛶ FULLSCREEN")
            self.setCursor(Qt.CursorShape.ArrowCursor)
        else:
            self.showFullScreen()
            self.btn_fullscreen.setText("🗗 WINDOWED")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_F11:
            self._toggle_fullscreen()
            event.accept()
        elif event.key() == Qt.Key.Key_Escape and self.isFullScreen():
            self._toggle_fullscreen()
            event.accept()
        else:
            super().keyPressEvent(event)

    def changeEvent(self, event):
        if event.type() == event.Type.WindowStateChange and hasattr(self, 'btn_fullscreen'):
            if self.isFullScreen():
                self.btn_fullscreen.setText("🗗 WINDOWED")
            else:
                self.btn_fullscreen.setText("⛶ FULLSCREEN")
                self.setCursor(Qt.CursorShape.ArrowCursor)
        super().changeEvent(event)

    def _on_power_update(self, volt, curr):
        if volt > 0:
            self.lbl_volt.setText(f"⚡ {volt:.1f} V")
        if curr >= 0:
            self.lbl_curr.setText(f"〜 {curr:.2f} A")

    # ── Check page ─────────────────────────────────────────────────────────────
    def setup_check_page(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title = QLabel("SYSTEM INITIALIZATION")
        title.setFont(QFont("Inter", 32, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color: #bb86fc;")
        layout.addWidget(title)
        self.lbl_check_status = QLabel("Starting checks...")
        self.lbl_check_status.setFont(QFont("Inter", 20))
        self.lbl_check_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.lbl_check_status)
        self.check_progress = QProgressBar()
        self.check_progress.setFixedSize(500, 30)
        self.check_progress.setStyleSheet("QProgressBar::chunk {background-color: #03dac6;}")
        self.check_progress.setValue(0)
        layout.addWidget(self.check_progress, alignment=Qt.AlignmentFlag.AlignCenter)
        self.btn_retry_check = QPushButton("RETRY")
        self.btn_retry_check.setFont(QFont("Inter", 16, QFont.Weight.Bold))
        self.btn_retry_check.setFixedSize(200, 60)
        self.btn_retry_check.setStyleSheet("background-color: #cf6679; color: black; border-radius: 10px;")
        self.btn_retry_check.hide()
        self.btn_retry_check.clicked.connect(self.start_hardware_check)
        layout.addWidget(self.btn_retry_check, alignment=Qt.AlignmentFlag.AlignCenter)
        self.page_check.setLayout(layout)

    # ── Safety page ────────────────────────────────────────────────────────────
    def setup_safety_page(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title = QLabel("⚠  SAFETY WARNING  ⚠")
        title.setFont(QFont("Inter", 48, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color: #ffb74d;")
        layout.addWidget(title)
        msg = QLabel(
            "The robot is about to perform an Encoder Index Search.\n\n"
            "The wheels will rotate in random directions.\n"
            "Ensure the robot is lifted off the ground\n"
            "or has at least 1 metre of clearance to any objects."
        )
        msg.setFont(QFont("Inter", 22))
        msg.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(msg)
        self.btn_confirm = QPushButton("CONFIRM SURROUNDING AREA IS CLEAR")
        self.btn_confirm.setFont(QFont("Inter", 20, QFont.Weight.Bold))
        self.btn_confirm.setFixedSize(450, 80)
        self.btn_confirm.setStyleSheet("background-color: #03dac6; color: black; border-radius: 15px;")
        self.btn_confirm.clicked.connect(self.start_index_search)
        layout.addWidget(self.btn_confirm, alignment=Qt.AlignmentFlag.AlignCenter)
        self.page_safety.setLayout(layout)

    # ── Index search page ──────────────────────────────────────────────────────
    def setup_search_page(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title = QLabel("ENCODER INDEX SEARCH")
        title.setFont(QFont("Inter", 32, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color: #03dac6;")
        layout.addWidget(title)
        self.lbl_search_status = QLabel("Initializing...")
        self.lbl_search_status.setFont(QFont("Inter", 20))
        self.lbl_search_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.lbl_search_status)
        self.search_progress = QProgressBar()
        self.search_progress.setFixedSize(500, 30)
        self.search_progress.setStyleSheet("QProgressBar::chunk {background-color: #bb86fc;}")
        self.search_progress.setValue(0)
        layout.addWidget(self.search_progress, alignment=Qt.AlignmentFlag.AlignCenter)
        self.btn_retry_search = QPushButton("RETRY")
        self.btn_retry_search.setFont(QFont("Inter", 16, QFont.Weight.Bold))
        self.btn_retry_search.setFixedSize(200, 60)
        self.btn_retry_search.setStyleSheet("background-color: #cf6679; color: black; border-radius: 10px;")
        self.btn_retry_search.hide()
        self.btn_retry_search.clicked.connect(self.start_index_search)
        layout.addWidget(self.btn_retry_search, alignment=Qt.AlignmentFlag.AlignCenter)
        self.page_search.setLayout(layout)

    # ── Calibration choice page (NEW) ─────────────────────────────────────────
    def setup_calib_choice_page(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.setSpacing(28)

        title = QLabel("INDEX SEARCH COMPLETE ✓")
        title.setFont(QFont("Inter", 36, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color: #00e676;")
        layout.addWidget(title)

        subtitle = QLabel("Would you like to run a Drive Calibration?")
        subtitle.setFont(QFont("Inter", 22))
        subtitle.setAlignment(Qt.AlignmentFlag.AlignCenter)
        subtitle.setStyleSheet("color: #cccccc;")
        layout.addWidget(subtitle)

        detail = QLabel(
            "The robot will drive forward 0.5 m/s for 2 s,\n"
            "rotate 360° on the spot, then reverse 0.5 m/s for 2 s.\n\n"
            "⚠  Ensure 1 m clearance on all sides before proceeding."
        )
        detail.setFont(QFont("Inter", 17))
        detail.setAlignment(Qt.AlignmentFlag.AlignCenter)
        detail.setStyleSheet("color: #888888;")
        layout.addWidget(detail)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(40)
        btn_row.setAlignment(Qt.AlignmentFlag.AlignCenter)

        btn_yes = QPushButton("✓  YES — RUN CALIBRATION")
        btn_yes.setFont(QFont("Inter", 20, QFont.Weight.Bold))
        btn_yes.setFixedSize(360, 90)
        btn_yes.setStyleSheet(
            "QPushButton { background-color: #ffb74d; color: #000; border-radius: 16px; }"
            "QPushButton:hover { background-color: #ffc96e; }"
            "QPushButton:pressed { background-color: #c87500; }"
        )
        btn_yes.clicked.connect(self.start_drive_calib)
        btn_row.addWidget(btn_yes)

        btn_no = QPushButton("✗  NO — SKIP TO LAUNCH")
        btn_no.setFont(QFont("Inter", 20, QFont.Weight.Bold))
        btn_no.setFixedSize(360, 90)
        btn_no.setStyleSheet(
            "QPushButton { background-color: #1e293b; color: #94a3b8; border: 2px solid #334155; border-radius: 16px; }"
            "QPushButton:hover { background-color: #334155; color: #e2e8f0; }"
            "QPushButton:pressed { background-color: #0f172a; }"
        )
        btn_no.clicked.connect(self._skip_calib_to_launch)
        btn_row.addWidget(btn_no)

        layout.addLayout(btn_row)
        self.page_calib_choice.setLayout(layout)

    # ── Drive calibration page ─────────────────────────────────────────────────
    def setup_drive_calib_page(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title = QLabel("DRIVE CALIBRATION")
        title.setFont(QFont("Inter", 32, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color: #ffb74d;")
        layout.addWidget(title)
        subtitle = QLabel("Forward → Spin 360° → Reverse")
        subtitle.setFont(QFont("Inter", 18))
        subtitle.setAlignment(Qt.AlignmentFlag.AlignCenter)
        subtitle.setStyleSheet("color: #888;")
        layout.addWidget(subtitle)
        layout.addSpacing(20)
        self.lbl_calib_status = QLabel("Preparing...")
        self.lbl_calib_status.setFont(QFont("Inter", 20))
        self.lbl_calib_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.lbl_calib_status)
        self.calib_progress = QProgressBar()
        self.calib_progress.setFixedSize(500, 30)
        self.calib_progress.setStyleSheet("QProgressBar::chunk {background-color: #ffb74d;}")
        self.calib_progress.setValue(0)
        layout.addWidget(self.calib_progress, alignment=Qt.AlignmentFlag.AlignCenter)
        layout.addSpacing(20)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(20)
        btn_row.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.btn_retry_calib = QPushButton("RETRY CALIBRATION")
        self.btn_retry_calib.setFont(QFont("Inter", 16, QFont.Weight.Bold))
        self.btn_retry_calib.setFixedSize(260, 60)
        self.btn_retry_calib.setStyleSheet("background-color: #cf6679; color: black; border-radius: 10px;")
        self.btn_retry_calib.hide()
        self.btn_retry_calib.clicked.connect(self.start_drive_calib)
        btn_row.addWidget(self.btn_retry_calib)

        self.btn_skip_calib = QPushButton("SKIP & CONTINUE")
        self.btn_skip_calib.setFont(QFont("Inter", 16, QFont.Weight.Bold))
        self.btn_skip_calib.setFixedSize(260, 60)
        self.btn_skip_calib.setStyleSheet("background-color: #444; color: white; border-radius: 10px;")
        self.btn_skip_calib.hide()
        self.btn_skip_calib.clicked.connect(self._go_to_launch)
        btn_row.addWidget(self.btn_skip_calib)

        layout.addLayout(btn_row)
        self.page_drive_calib.setLayout(layout)

    # ── Launch mode page ───────────────────────────────────────────────────────
    def setup_launch_page(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.setSpacing(30)
        title = QLabel("INITIALISATION COMPLETE")
        title.setFont(QFont("Inter", 32, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color: #00e676;")
        layout.addWidget(title)
        subtitle = QLabel("Select a launch mode to start the robot stack")
        subtitle.setFont(QFont("Inter", 18))
        subtitle.setAlignment(Qt.AlignmentFlag.AlignCenter)
        subtitle.setStyleSheet("color: #aaaaaa;")
        layout.addWidget(subtitle)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(40)
        btn_row.setAlignment(Qt.AlignmentFlag.AlignCenter)

        def _btn(label, color, hover, press):
            b = QPushButton(label)
            b.setFixedSize(280, 120)
            b.setStyleSheet(
                f"QPushButton {{ background-color: {color}; color: #000; border-radius: 20px; "
                f"font-size: 22px; font-weight: bold; padding: 20px; }}"
                f"QPushButton:hover {{ background-color: {hover}; }}"
                f"QPushButton:pressed {{ background-color: {press}; }}"
            )
            return b

        btn_mapping = _btn("Mapping",        "#03dac6", "#00f5e4", "#018f84")
        btn_mapping.clicked.connect(lambda: self.open_map_selection("mapping"))
        btn_row.addWidget(btn_mapping)

        btn_loc = _btn("Autonomous Nav",     "#80EF80", "#cf9fff", "#7c59b5")
        btn_loc.clicked.connect(lambda: self.open_map_selection("localization"))
        btn_row.addWidget(btn_loc)

        btn_drive = _btn("Driving",          "#ffb74d", "#ffc96e", "#c87500")
        btn_drive.clicked.connect(lambda: self.launch_ros("driving"))
        btn_row.addWidget(btn_drive)

        layout.addLayout(btn_row)
        self.page_launch.setLayout(layout)

    # ── Map Selection Page ─────────────────────────────────────────────────────
    def setup_map_select_page(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.setSpacing(20)
        layout.setContentsMargins(40, 20, 40, 20)

        self.lbl_map_select_title = QLabel("SELECT MAP")
        self.lbl_map_select_title.setFont(QFont("Inter", 28, QFont.Weight.Bold))
        self.lbl_map_select_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_map_select_title.setStyleSheet("color: #03dac6;")
        layout.addWidget(self.lbl_map_select_title)

        self.lbl_map_select_sub = QLabel("Choose a map from global_maps")
        self.lbl_map_select_sub.setFont(QFont("Inter", 16))
        self.lbl_map_select_sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_map_select_sub.setStyleSheet("color: #aaaaaa;")
        layout.addWidget(self.lbl_map_select_sub)

        # Scroll area for map list
        self.map_scroll = QScrollArea()
        self.map_scroll.setWidgetResizable(True)
        self.map_scroll.setFixedSize(680, 320)
        self.map_scroll.setStyleSheet(
            "QScrollArea { background: #141414; border: 1px solid #333; border-radius: 12px; }"
            "QScrollBar:vertical { background: #121212; width: 10px; border-radius: 5px; }"
            "QScrollBar::handle:vertical { background: #444; border-radius: 5px; }"
            "QScrollBar::handle:vertical:hover { background: #666; }"
        )

        self.map_list_container = QWidget()
        self.map_list_layout = QVBoxLayout(self.map_list_container)
        self.map_list_layout.setSpacing(10)
        self.map_list_layout.setContentsMargins(15, 15, 15, 15)
        self.map_list_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.map_scroll.setWidget(self.map_list_container)
        layout.addWidget(self.map_scroll, alignment=Qt.AlignmentFlag.AlignCenter)

        # Action buttons
        btn_row = QHBoxLayout()
        btn_row.setSpacing(25)
        btn_row.setAlignment(Qt.AlignmentFlag.AlignCenter)

        btn_back = QPushButton("⬅ BACK")
        btn_back.setFont(QFont("Inter", 16, QFont.Weight.Bold))
        btn_back.setFixedSize(200, 60)
        btn_back.setStyleSheet(
            "QPushButton { background-color: #2a2a2a; color: white; border-radius: 12px; border: 1px solid #444; }"
            "QPushButton:hover { background-color: #383838; }"
            "QPushButton:pressed { background-color: #1a1a1a; }"
        )
        btn_back.clicked.connect(lambda: self.stack.setCurrentWidget(self.page_launch))
        btn_row.addWidget(btn_back)

        self.btn_confirm_map = QPushButton("CONTINUE ➡")
        self.btn_confirm_map.setFont(QFont("Inter", 16, QFont.Weight.Bold))
        self.btn_confirm_map.setFixedSize(240, 60)
        self.btn_confirm_map.setStyleSheet(
            "QPushButton { background-color: #03dac6; color: #000; border-radius: 12px; font-weight: bold; }"
            "QPushButton:hover { background-color: #00f5e4; }"
            "QPushButton:disabled { background-color: #333333; color: #666666; }"
        )
        self.btn_confirm_map.setEnabled(False)
        self.btn_confirm_map.clicked.connect(self._on_map_confirmed)
        btn_row.addWidget(self.btn_confirm_map)

        layout.addLayout(btn_row)
        self.page_map_select.setLayout(layout)

    def open_map_selection(self, mode: str):
        self._target_mode = mode
        self._selected_map_name = None
        self.btn_confirm_map.setEnabled(False)

        if mode == "mapping":
            self.lbl_map_select_title.setText("SELECT MAP FOR MAPPING")
            self.lbl_map_select_title.setStyleSheet("color: #03dac6;")
            self.lbl_map_select_sub.setText("Select a map to build upon, or start a new map")
        else:
            self.lbl_map_select_title.setText("SELECT MAP FOR AUTONOMOUS NAV")
            self.lbl_map_select_title.setStyleSheet("color: #80EF80;")
            self.lbl_map_select_sub.setText("Select a known map from global_maps for localization")

        # Clear existing items in layout
        while self.map_list_layout.count():
            item = self.map_list_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        self._map_button_group = []

        global_maps_dir = "/home/hospobot/hospobot_ws/global_maps"
        os.makedirs(global_maps_dir, exist_ok=True)
        yaml_files = sorted(glob.glob(os.path.join(global_maps_dir, "*.yaml")))

        # If mapping, add option for brand new map
        if mode == "mapping":
            btn_new = self._create_map_card(
                map_id="__new__",
                title="✨  + CREATE NEW MAP",
                subtitle="Start mapping from scratch with a fresh SLAM database",
                is_new_option=True
            )
            self.map_list_layout.addWidget(btn_new)
            self._map_button_group.append(btn_new)

        if not yaml_files and mode != "mapping":
            lbl_empty = QLabel("No maps found in global_maps folder!")
            lbl_empty.setFont(QFont("Inter", 14))
            lbl_empty.setStyleSheet("color: #cf6679; padding: 20px;")
            lbl_empty.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.map_list_layout.addWidget(lbl_empty)
        else:
            valid_maps = []
            for yf in yaml_files:
                base_name = os.path.splitext(os.path.basename(yf))[0]
                if base_name.endswith('_keepout'):
                    continue
                valid_maps.append((base_name, yf))

            # Prioritize Building6-Floor1 as primary main map at the top
            valid_maps.sort(key=lambda x: (0 if x[0] == 'Building6-Floor1' else 1, x[0].lower()))

            for base_name, yf in valid_maps:
                db_path = os.path.join(global_maps_dir, f"{base_name}.db")
                has_db = os.path.exists(db_path)
                is_main = (base_name == "Building6-Floor1")
                sub = f"{base_name}.yaml" + ("  •  ⭐ MAIN MAP" if is_main else "") + ("  •  [3D/2D SLAM DB available]" if has_db else "")
                title_str = f"🗺  {base_name}" + (" (Primary)" if is_main else "")
                btn = self._create_map_card(
                    map_id=base_name,
                    title=title_str,
                    subtitle=sub,
                    is_new_option=False
                )
                self.map_list_layout.addWidget(btn)
                self._map_button_group.append(btn)

        self.stack.setCurrentWidget(self.page_map_select)

    def _create_map_card(self, map_id, title, subtitle, is_new_option=False):
        btn = QPushButton()
        btn.setFixedHeight(72)
        btn.setCheckable(True)
        btn.setCursor(Qt.CursorShape.PointingHandCursor)

        l = QVBoxLayout(btn)
        l.setContentsMargins(18, 10, 18, 10)
        l.setSpacing(4)
        lbl_t = QLabel(title)
        lbl_t.setFont(QFont("Inter", 15, QFont.Weight.Bold))
        lbl_s = QLabel(subtitle)
        lbl_s.setFont(QFont("Inter", 12))
        l.addWidget(lbl_t)
        l.addWidget(lbl_s)

        base_border = "#00e676" if is_new_option else "#3a3a3a"

        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: #202020;
                border: 2px solid {base_border};
                border-radius: 10px;
                text-align: left;
            }}
            QPushButton:hover {{
                background-color: #2a2a2a;
                border-color: #03dac6;
            }}
            QPushButton:checked {{
                background-color: #14382e;
                border: 2px solid #00e676;
            }}
            QLabel {{
                background: transparent;
                color: #ffffff;
            }}
        """)
        lbl_s.setStyleSheet("background: transparent; color: #999999;")

        def on_click():
            for b in self._map_button_group:
                if b is not btn:
                    b.setChecked(False)
            btn.setChecked(True)
            self._selected_map_name = "" if map_id == "__new__" else map_id
            self.btn_confirm_map.setEnabled(True)

        btn.clicked.connect(on_click)
        return btn

    def _on_map_confirmed(self):
        if self._target_mode == "localization":
            self.launch_ros("localization", self._selected_map_name)
        elif self._target_mode == "mapping":
            self._prepare_mapping_prep_page()
            self.stack.setCurrentWidget(self.page_mapping_prep)

    # ── Mapping Preparation & Countdown Page ───────────────────────────────────
    def setup_mapping_prep_page(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.setSpacing(25)
        layout.setContentsMargins(50, 25, 50, 25)

        title = QLabel("MAPPING MODE CONFIGURATION")
        title.setFont(QFont("Inter", 28, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color: #03dac6;")
        layout.addWidget(title)

        self.lbl_mapping_target_info = QLabel("")
        self.lbl_mapping_target_info.setFont(QFont("Inter", 16, QFont.Weight.Bold))
        self.lbl_mapping_target_info.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_mapping_target_info.setStyleSheet("color: #ffb74d;")
        layout.addWidget(self.lbl_mapping_target_info)

        # Warning Card
        card = QFrame()
        card.setFixedWidth(740)
        card.setStyleSheet(
            "background-color: #1a1a1a; border: 2px solid #03dac6; border-radius: 16px; padding: 20px;"
        )
        card_layout = QVBoxLayout(card)
        card_layout.setSpacing(14)

        lbl_warn_icon = QLabel("⚠️  OPERATOR NOTICE")
        lbl_warn_icon.setFont(QFont("Inter", 18, QFont.Weight.Bold))
        lbl_warn_icon.setStyleSheet("color: #03dac6; border: none;")
        card_layout.addWidget(lbl_warn_icon)

        # Exact text requested by user:
        exact_text = (
            "The robot will now utilise the 2D LiDAR scanner for Mapping/Localisation. "
            "The operator should stand in front of the robot during mapping, as all objects directly "
            "in front of the robot are excluded from the map. Controls for the robot are reversed and "
            "the forward direction is ⬆."
        )
        lbl_warn_text = QLabel(exact_text)
        lbl_warn_text.setFont(QFont("Inter", 16))
        lbl_warn_text.setWordWrap(True)
        lbl_warn_text.setStyleSheet("color: #e6e6e6; border: none; line-height: 1.4;")
        card_layout.addWidget(lbl_warn_text)

        layout.addWidget(card, alignment=Qt.AlignmentFlag.AlignCenter)

        # Countdown label
        self.lbl_countdown = QLabel("")
        self.lbl_countdown.setFont(QFont("Inter", 24, QFont.Weight.Bold))
        self.lbl_countdown.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_countdown.setStyleSheet("color: #00e676;")
        self.lbl_countdown.setFixedHeight(40)
        layout.addWidget(self.lbl_countdown)

        # Buttons
        btn_row = QHBoxLayout()
        btn_row.setSpacing(30)
        btn_row.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.btn_prep_back = QPushButton("⬅ BACK")
        self.btn_prep_back.setFont(QFont("Inter", 16, QFont.Weight.Bold))
        self.btn_prep_back.setFixedSize(200, 65)
        self.btn_prep_back.setStyleSheet(
            "QPushButton { background-color: #2a2a2a; color: white; border-radius: 12px; border: 1px solid #444; }"
            "QPushButton:hover { background-color: #383838; }"
        )
        self.btn_prep_back.clicked.connect(self._on_prep_back)
        btn_row.addWidget(self.btn_prep_back)

        self.btn_begin_mapping = QPushButton("BEGIN MAPPING")
        self.btn_begin_mapping.setFont(QFont("Inter", 18, QFont.Weight.Bold))
        self.btn_begin_mapping.setFixedSize(300, 65)
        self.btn_begin_mapping.setStyleSheet(
            "QPushButton { background-color: #03dac6; color: #000; border-radius: 14px; font-weight: bold; }"
            "QPushButton:hover { background-color: #00f5e4; }"
            "QPushButton:disabled { background-color: #333333; color: #666666; }"
        )
        self.btn_begin_mapping.clicked.connect(self._start_mapping_countdown)
        btn_row.addWidget(self.btn_begin_mapping)

        layout.addLayout(btn_row)
        self.page_mapping_prep.setLayout(layout)

    def _prepare_mapping_prep_page(self):
        self._countdown_timer.stop()
        self._countdown_val = 5
        self.lbl_countdown.setText("")
        self.btn_begin_mapping.setEnabled(True)
        self.btn_prep_back.setEnabled(True)
        if self._selected_map_name:
            self.lbl_mapping_target_info.setText(f"Target Map: Building upon '{self._selected_map_name}'")
        else:
            self.lbl_mapping_target_info.setText("Target Map: Starting a New Map from Scratch")

    def _start_mapping_countdown(self):
        self.btn_begin_mapping.setEnabled(False)
        self.btn_prep_back.setEnabled(False)
        self._countdown_val = 5
        self.lbl_countdown.setText(f"Starting mapping in {self._countdown_val} seconds...")
        self._countdown_timer.start()

    def _on_countdown_tick(self):
        self._countdown_val -= 1
        if self._countdown_val > 0:
            self.lbl_countdown.setText(f"Starting mapping in {self._countdown_val} second{'s' if self._countdown_val > 1 else ''}...")
        elif self._countdown_val == 0:
            self.lbl_countdown.setText("🚀 Launching Mapping Stack...")
        else:
            self._countdown_timer.stop()
            self.launch_ros("mapping", self._selected_map_name)

    def _on_prep_back(self):
        self._countdown_timer.stop()
        self.stack.setCurrentWidget(self.page_map_select)

    # ── IDLE page ──────────────────────────────────────────────────────────────
    def setup_idle_page(self):
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.setSpacing(24)

        title = QLabel("HOSPOBOT")
        title.setFont(QFont("Inter", 64, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color: #ffffff;")
        layout.addWidget(title)

        self.lbl_idle_status = QLabel("● IDLE & READY")
        self.lbl_idle_status.setFont(QFont("Inter", 24, QFont.Weight.Bold))
        self.lbl_idle_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_idle_status.setStyleSheet("color: #00e676;")
        layout.addWidget(self.lbl_idle_status)

        self.btn_motors_toggle = QPushButton("MOTORS: IDLE")
        self.btn_motors_toggle.setFont(QFont("Inter", 20, QFont.Weight.Bold))
        self.btn_motors_toggle.setFixedSize(340, 70)
        self._style_motors_btn(False)
        self.btn_motors_toggle.clicked.connect(self._toggle_motors)
        layout.addWidget(self.btn_motors_toggle, alignment=Qt.AlignmentFlag.AlignCenter)

        self.btn_finish_mapping = QPushButton("💾  FINISH MAPPING")
        self.btn_finish_mapping.setFont(QFont("Inter", 20, QFont.Weight.Bold))
        self.btn_finish_mapping.setFixedSize(340, 70)
        self.btn_finish_mapping.setStyleSheet(
            "QPushButton { background-color: #03dac6; color: #000; border-radius: 12px; font-weight: bold; }"
            "QPushButton:hover { background-color: #00f5e4; }"
            "QPushButton:pressed { background-color: #018f84; }"
        )
        self.btn_finish_mapping.clicked.connect(self._on_finish_mapping_clicked)
        self.btn_finish_mapping.hide()
        layout.addWidget(self.btn_finish_mapping, alignment=Qt.AlignmentFlag.AlignCenter)

        btn_exit = QPushButton("⬅  EXIT / CHANGE MODE")
        btn_exit.setFont(QFont("Inter", 18, QFont.Weight.Bold))
        btn_exit.setFixedSize(340, 70)
        btn_exit.setStyleSheet(
            "QPushButton { background-color: #dc2626; color: white; border-radius: 12px; }"
            "QPushButton:hover { background-color: #ef4444; }"
            "QPushButton:pressed { background-color: #991b1b; }"
        )
        btn_exit.clicked.connect(self._exit_launch_mode)
        layout.addWidget(btn_exit, alignment=Qt.AlignmentFlag.AlignCenter)
        self.page_idle.setLayout(layout)

    # ── Control flow ───────────────────────────────────────────────────────────
    def start_hardware_check(self):
        self.btn_retry_check.hide()
        self.lbl_check_status.setText("Starting checks...")
        self.lbl_check_status.setStyleSheet("color: white;")
        self.check_progress.setValue(0)
        self.worker = WorkerThread(mode="check")
        self.worker.progress.connect(self._update_check_progress)
        self.worker.finished_checks.connect(self._on_check_finished)
        self.worker.start()

    def _update_check_progress(self, text, color, val):
        self.lbl_check_status.setText(text)
        c = {"red": "#cf6679", "green": "#00e676"}.get(color, "white")
        self.lbl_check_status.setStyleSheet(f"color: {c};")
        chunk = "#cf6679" if color == "red" else "#03dac6"
        self.check_progress.setStyleSheet(f"QProgressBar::chunk {{background-color: {chunk};}}")
        self.check_progress.setValue(val)

    def _on_check_finished(self, success):
        if success:
            self._start_power_monitor()
            QTimer.singleShot(1500, lambda: self.stack.setCurrentWidget(self.page_safety))
        else:
            self.btn_retry_check.show()

    def start_index_search(self):
        self.stack.setCurrentWidget(self.page_search)
        self.btn_retry_search.hide()
        self.lbl_search_status.setText("Initializing...")
        self.lbl_search_status.setStyleSheet("color: white;")
        self.search_progress.setValue(0)
        self._search_worker = WorkerThread(mode="search")
        self._search_worker.progress.connect(self._update_search_progress)
        self._search_worker.finished_search.connect(self._on_search_finished)
        self._search_worker.start()

    def _update_search_progress(self, text, color, val):
        self.lbl_search_status.setText(text)
        if color == "red":
            self.lbl_search_status.setStyleSheet("color: #cf6679;")
            self.search_progress.setStyleSheet("QProgressBar::chunk {background-color: #cf6679;}")
        elif color == "green":
            self.lbl_search_status.setStyleSheet("color: #00e676;")
            self.search_progress.setStyleSheet("QProgressBar::chunk {background-color: #00e676;}")
        else:
            self.lbl_search_status.setStyleSheet("color: white;")
            self.search_progress.setStyleSheet("QProgressBar::chunk {background-color: #bb86fc;}")
        self.search_progress.setValue(val)

    def _on_search_finished(self, success, error_msg):
        if success:
            # Show the calibration choice screen
            QTimer.singleShot(600, lambda: self.stack.setCurrentWidget(self.page_calib_choice))
        else:
            self._update_search_progress(f"ERROR: {error_msg}", "red", 100)
            self.btn_retry_search.show()

    def _skip_calib_to_launch(self):
        """User chose NOT to run drive calibration.
        Run skip_to_idle in a thread (it has time.sleep) then go to launch."""
        worker = self._search_worker
        if worker and (worker._ser is not None or worker._bus is not None):
            # Run skip_to_idle in a disposable thread so GUI doesn't block
            class _SkipThread(QThread):
                done = pyqtSignal()
                def __init__(self, w): super().__init__(); self._w = w
                def run(self): self._w.skip_to_idle(); self.done.emit()
            self._skip_thread = _SkipThread(worker)
            self._skip_thread.done.connect(self._go_to_launch)
            self._skip_thread.start()
        else:
            self._go_to_launch()

    def start_drive_calib(self):
        self.stack.setCurrentWidget(self.page_drive_calib)
        self.btn_retry_calib.hide()
        self.btn_skip_calib.hide()
        self.lbl_calib_status.setText("Starting drive calibration...")
        self.lbl_calib_status.setStyleSheet("color: white;")
        self.calib_progress.setValue(0)
        self.calib_progress.setStyleSheet("QProgressBar::chunk {background-color: #ffb74d;}")

        self._calib_worker = WorkerThread(mode="drive_calib")
        # Pass the open serial/CAN handles from the search worker
        if self._search_worker:
            self._calib_worker._ser = self._search_worker._ser
            self._calib_worker._bus = self._search_worker._bus
            self._calib_worker._left_port = self._search_worker._left_port
            # Clear from search worker so it doesn't double-close
            self._search_worker._ser = None
            self._search_worker._bus = None
        self._calib_worker.progress.connect(self._update_calib_progress)
        self._calib_worker.finished_calib.connect(self._on_calib_finished)
        self._calib_worker.start()

    def _update_calib_progress(self, text, color, val):
        self.lbl_calib_status.setText(text)
        if color == "red":
            self.lbl_calib_status.setStyleSheet("color: #cf6679;")
            self.calib_progress.setStyleSheet("QProgressBar::chunk {background-color: #cf6679;}")
        elif color == "green":
            self.lbl_calib_status.setStyleSheet("color: #00e676;")
            self.calib_progress.setStyleSheet("QProgressBar::chunk {background-color: #00e676;}")
        else:
            self.lbl_calib_status.setStyleSheet("color: white;")
            self.calib_progress.setStyleSheet("QProgressBar::chunk {background-color: #ffb74d;}")
        self.calib_progress.setValue(val)

    def _on_calib_finished(self, success, error_msg):
        if success:
            QTimer.singleShot(1000, self._go_to_launch)
        else:
            self._update_calib_progress(f"ERROR: {error_msg}", "red", 100)
            self.btn_retry_calib.show()
            self.btn_skip_calib.show()

    def _go_to_launch(self):
        self.stack.setCurrentWidget(self.page_launch)

    def _terminate_launch_proc(self):
        """Cleanly and reliably terminate active launch process group and any lingering launch nodes."""
        if self._launch_proc:
            proc = self._launch_proc
            self._launch_proc = None
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGINT)
            except Exception:
                pass

        def _cleanup_lingering():
            try:
                subprocess.run(["pkill", "-2", "-f", "ros2 launch hospobot_bringup hospobot_launch.py"], check=False)
            except Exception:
                pass

        def _force_cleanup():
            targets = [
                "hospobot_launch.py",
                "odrive_can_node",
                "cmd_vel_mux",
                "diagnostics_node",
                "sllidar_node",
                "laser_filter_node",
                "system_can_bridge",
                "footprint_publisher_node",
                "robot_fov_indicator_node",
                "nav_visualizer_node",
                "lifecycle_manager_navigation",
                "lifecycle_manager_localization",
                "lifecycle_manager_costmap_filters",
                "amcl",
                "map_server",
                "controller_server",
                "planner_server",
                "bt_navigator",
                "behavior_server",
                "smoother_server",
                "velocity_smoother"
            ]
            pattern = "|".join(targets)
            try:
                subprocess.run(["pkill", "-9", "-f", f"({pattern})"], check=False)
            except Exception:
                pass
            try:
                for f in glob.glob("/dev/shm/fastrtps_*") + glob.glob("/dev/shm/sem.fastrtps_*"):
                    try: os.remove(f)
                    except OSError: pass
            except Exception:
                pass

        QTimer.singleShot(400, _cleanup_lingering)
        QTimer.singleShot(2500, _force_cleanup)

    def _broadcast_mode_status(self):
        """Broadcasts current robot mode and state to web dashboard via ROS 2."""
        if not getattr(self, '_ros_sync', None):
            return
        state_text = self.lbl_idle_status.text() if hasattr(self, 'lbl_idle_status') else "IDLE"
        self._ros_sync.publish_status(
            mode=getattr(self, '_current_nav_mode', '') or "idle",
            map_name=getattr(self, '_active_loaded_map', '') or getattr(self, '_selected_map_name', '') or "Building6-Floor1",
            state_text=state_text,
            motors=getattr(self, '_motors_engaged', False)
        )

    def _trigger_auto_localize(self):
        """Broadcasts initial pose to AMCL when Auto Nav mode is running."""
        if self._current_nav_mode == "localization" and getattr(self, '_ros_sync', None):
            self._ros_sync.pulse_initial_pose()

    def _on_robot_localized(self, x: float, y: float, yaw: float):
        """Called when AMCL reports valid pose on /amcl_pose."""
        if self._current_nav_mode == "localization" and not getattr(self, '_localized', False):
            self._localized = True
            map_name = getattr(self, '_active_loaded_map', '') or getattr(self, '_selected_map_name', '') or "Building6-Floor1"
            self.lbl_idle_status.setText(f"● NAV READY - LOCALIZED ({map_name})")
            self.lbl_idle_status.setStyleSheet("color: #00e676;")
            self._broadcast_mode_status()
            print(f"[Desktop App] AMCL Localization Confirmed! Robot at x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad")

    def _on_remote_mode_cmd(self, mode: str, map_name: str):
        """Handles remote launch commands received from the web dashboard."""
        mode = mode.lower().strip()
        target_map = map_name.strip() if map_name.strip() else (self._selected_map_name or "Building6-Floor1")
        print(f"[Desktop App] Remote command received: mode='{mode}', map='{target_map}'")

        if mode in ["localization", "autonav", "auto_nav", "nav"]:
            if self._current_nav_mode == "localization":
                return  # Already running, ignore duplicate command
            self._localized = False  # Reset localization state on mode switch
            self._terminate_launch_proc()
            self._selected_map_name = target_map
            QTimer.singleShot(1500, lambda: self.launch_ros("localization", target_map))

        elif mode in ["mapping", "map"]:
            if self._current_nav_mode == "mapping":
                return
            self._localized = False
            self._terminate_launch_proc()
            self._selected_map_name = target_map
            QTimer.singleShot(1500, lambda: self.launch_ros("mapping", target_map))

        elif mode in ["driving", "drive"]:
            if self._current_nav_mode == "driving":
                return
            self._localized = False
            self._terminate_launch_proc()
            QTimer.singleShot(1500, lambda: self.launch_ros("driving"))

        elif mode in ["idle", "stop", "exit"]:
            self._exit_launch_mode()

    def launch_ros(self, mode: str, map_name: str = ""):
        self._current_nav_mode = mode
        self._active_loaded_map = map_name
        self._stop_power_monitor()
        self.stack.setCurrentWidget(self.page_idle)
        self._style_motors_btn(False)

        if mode == "mapping":
            self.btn_finish_mapping.show()
            target_str = f"BUILDING UPON: {map_name}" if map_name else "NEW MAP"
            self.lbl_idle_status.setText(f"● MAPPING ACTIVE ({target_str})")
            self.lbl_idle_status.setStyleSheet("color: #03dac6;")
        elif mode == "localization":
            self.btn_finish_mapping.hide()
            self.lbl_idle_status.setText(f"● NAV READY ({map_name})")
            self.lbl_idle_status.setStyleSheet("color: #80EF80;")
        else:
            self.btn_finish_mapping.hide()
            self.lbl_idle_status.setText("● DRIVING MODE READY")
            self.lbl_idle_status.setStyleSheet("color: #ffb74d;")

        self._broadcast_mode_status()
        QTimer.singleShot(1000, lambda: self._do_launch_ros(mode, map_name))

        # Automatically localise on launching Auto Nav mode (no RViz 2D Pose Estimate needed)
        if mode == "localization":
            self._localized = False
            for delay in [3000, 4500, 6000, 7500, 9000, 11000]:
                QTimer.singleShot(delay, self._trigger_auto_localize)

    def _do_launch_ros(self, mode: str, map_name: str = ""):
        active_db = "/home/hospobot/hospobot_ws/.active_mapping.db"
        if mode == "mapping":
            global_maps_dir = "/home/hospobot/hospobot_ws/global_maps"
            source_db = os.path.join(global_maps_dir, f"{map_name}.db") if map_name else None
            if source_db and os.path.exists(source_db):
                try:
                    shutil.copyfile(source_db, active_db)
                except Exception as e:
                    print(f"Error copying base db: {e}")
            else:
                if os.path.exists(active_db):
                    try:
                        os.remove(active_db)
                    except Exception:
                        pass

        map_param = f" map:={map_name}" if map_name and mode != "driving" else ""
        domain_id = os.environ.get("ROS_DOMAIN_ID", "42")
        launch_cmd = (
            f"source /opt/ros/jazzy/setup.bash && "
            f"source /home/hospobot/hospobot_ws/install/setup.bash && "
            f"export ROS_DOMAIN_ID={domain_id} && "
            f"exec ros2 launch hospobot_bringup hospobot_launch.py nav_mode:={mode}{map_param}"
        )

        self._launch_proc = subprocess.Popen(
            ['/bin/bash', '-c', launch_cmd],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )

    def _exit_launch_mode(self):
        """Kill launch stack non-blocking, then return to mode selection."""
        self.btn_finish_mapping.hide()
        self._active_loaded_map = ""
        self._current_nav_mode = "idle"
        self._localized = False  # Reset localization state on stop
        self._terminate_launch_proc()
        self.stack.setCurrentWidget(self.page_launch)
        self._broadcast_mode_status()
        # Give processes time to die before restarting power monitor
        QTimer.singleShot(3000, self._start_power_monitor)

    def _on_finish_mapping_clicked(self):
        loaded_map = self._active_loaded_map
        if loaded_map:
            # User loaded a previous map
            dlg = FinishMappingChoiceDialog(loaded_map, self)
            if dlg.exec() == QDialog.DialogCode.Accepted:
                if dlg.choice == 'overwrite':
                    self._start_save_map(loaded_map)
                elif dlg.choice == 'new':
                    name_dlg = MapNameInputDialog(default_name=f"{loaded_map}_new", parent=self)
                    if name_dlg.exec() == QDialog.DialogCode.Accepted and name_dlg.map_name:
                        self._start_save_map(name_dlg.map_name)
        else:
            # User started a new map
            name_dlg = MapNameInputDialog(default_name="", parent=self)
            if name_dlg.exec() == QDialog.DialogCode.Accepted and name_dlg.map_name:
                self._start_save_map(name_dlg.map_name)

    def _start_save_map(self, target_name: str):
        self.btn_finish_mapping.setEnabled(False)
        self.btn_finish_mapping.setText("💾  SAVING MAP...")
        self._save_worker = MapSaveWorker(target_name)
        self._save_worker.finished_save.connect(self._on_save_finished)
        self._save_worker.start()

    def _on_save_finished(self, success: bool, msg: str):
        self.btn_finish_mapping.setEnabled(True)
        self.btn_finish_mapping.setText("💾  FINISH MAPPING")
        title = "Map Saved Successfully" if success else "Error Saving Map"
        dlg = MapSavedConfirmationDialog(title, msg, success=success, parent=self)
        dlg.exec()
        if success:
            self.lbl_idle_status.setText(f"● MAP SAVED: {self._save_worker.target_name}")
            self.lbl_idle_status.setStyleSheet("color: #00e676;")

    def _toggle_motors(self):
        self._motors_engaged = not self._motors_engaged
        state_str = "CL-VEL" if self._motors_engaged else "IDLE"
        env = dict(os.environ)
        domain_id = os.environ.get("ROS_DOMAIN_ID", "42")
        env['ROS_DOMAIN_ID'] = domain_id
        for topic in ['/odesc_hardware/state_left', '/odesc_hardware/state_right']:
            subprocess.Popen(
                ['bash', '-c',
                 f'source /opt/ros/jazzy/setup.bash && '
                 f'source /home/hospobot/hospobot_ws/install/setup.bash && '
                 f'export ROS_DOMAIN_ID={domain_id} && '
                 f"ros2 topic pub --once {topic} std_msgs/String \"{{data: '{state_str}'}}\""],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env
            )
        self._style_motors_btn(self._motors_engaged)
        self._broadcast_mode_status()

    def _style_motors_btn(self, engaged: bool):
        self._motors_engaged = engaged
        if engaged:
            self.btn_motors_toggle.setText("MOTORS: ENGAGED")
            self.btn_motors_toggle.setStyleSheet(
                "QPushButton { background-color: #1e1e1e; color: #00e676; border: 2px solid #00e676; "
                "border-radius: 12px; font-size: 20px; font-weight: bold; }"
                "QPushButton:hover { background-color: #2a2a2a; }"
            )
        else:
            self.btn_motors_toggle.setText("MOTORS: IDLE")
            self.btn_motors_toggle.setStyleSheet(
                "QPushButton { background-color: #1e1e1e; color: #f59e0b; border: 2px solid #f59e0b; "
                "border-radius: 12px; font-size: 20px; font-weight: bold; }"
                "QPushButton:hover { background-color: #2a2a2a; }"
            )

    # ── Power monitor process ──────────────────────────────────────────────────
    def _start_power_monitor(self):
        if self._power_monitor_proc and self._power_monitor_proc.poll() is None:
            return
        env = dict(os.environ)
        domain_id = os.environ.get("ROS_DOMAIN_ID", "42")
        env['ROS_DOMAIN_ID'] = domain_id
        cmd = (
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/hospobot/hospobot_ws/install/setup.bash && "
            f"export ROS_DOMAIN_ID={domain_id} && "
            "ros2 run hospobot_can_bridge power_monitor"
        )
        self._power_monitor_proc = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            env=env,
            preexec_fn=os.setsid   # isolate in own process group
        )

    def _stop_power_monitor(self):
        """Non-blocking kill — never call time.sleep() here (GUI thread)."""
        if self._power_monitor_proc is not None:
            try:
                os.killpg(os.getpgid(self._power_monitor_proc.pid), signal.SIGTERM)
            except Exception:
                try:
                    self._power_monitor_proc.terminate()
                except Exception:
                    pass
        self._power_monitor_proc = None

    def closeEvent(self, event):
        if hasattr(self, '_ros_sync') and self._ros_sync:
            try:
                self._ros_sync.stop()
            except Exception:
                pass
        if hasattr(self, '_status_broadcast_timer') and self._status_broadcast_timer:
            self._status_broadcast_timer.stop()
        self._power_thread.stop()
        self._stop_power_monitor()
        self._terminate_launch_proc()
        super().closeEvent(event)



def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
