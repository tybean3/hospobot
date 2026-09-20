#!/usr/bin/env python3
"""
power_monitor_node.py
Standalone node that reads bus voltage and current from both motor drivers
and publishes to /sensors/bus_voltage and /sensors/bus_current.

Left motor  (ODESC 4.2) — USB serial /dev/ttyACM0, ASCII protocol
Right motor (ODrive S1) — CAN bus, RTR frame CMD 0x017 / 0x014

Designed to run continuously, independent of the main launch stack.
Exits gracefully if the serial port is taken by another process.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import struct
import time
import glob
import os
import termios


class PowerMonitorNode(Node):
    def __init__(self):
        super().__init__('power_monitor')

        self.declare_parameter('poll_rate_hz', 1.0)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('right_node_id', 2)
        self.declare_parameter('serial_timeout', 0.3)

        self.poll_rate = self.get_parameter('poll_rate_hz').value
        self.serial_port = self.get_parameter('serial_port').value
        self.can_channel = self.get_parameter('can_channel').value
        self.right_node_id = self.get_parameter('right_node_id').value
        self.serial_timeout = self.get_parameter('serial_timeout').value

        self.volt_pub = self.create_publisher(Float32, '/sensors/bus_voltage', 10)
        self.curr_pub = self.create_publisher(Float32, '/sensors/bus_current', 10)

        self._serial_fd = None
        self._can_bus = None
        self._left_voltage = 0.0
        self._left_current = 0.0
        self._right_voltage = 0.0
        self._right_current = 0.0

        self._timer = self.create_timer(1.0 / self.poll_rate, self._poll)
        self.get_logger().info('Power monitor started.')

    def _open_serial(self):
        ports = [self.serial_port] + sorted(glob.glob('/dev/ttyACM*'))
        ports = list(dict.fromkeys(ports))
        for port in ports:
            try:
                fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                attrs = termios.tcgetattr(fd)
                attrs[0] = 0
                attrs[1] = 0
                attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | termios.B115200
                attrs[3] = 0
                attrs[4] = termios.B115200
                attrs[5] = termios.B115200
                termios.tcsetattr(fd, termios.TCSANOW, attrs)
                time.sleep(0.15)
                self._drain_fd(fd)
                self._serial_fd = fd
                self.serial_port = port
                self.get_logger().info(f'Power monitor: opened {port}')
                return True
            except Exception as e:
                self.get_logger().debug(f'Power monitor: could not open {port}: {e}')
        return False

    def _drain_fd(self, fd):
        try:
            while True:
                data = os.read(fd, 4096)
                if not data:
                    break
        except (BlockingIOError, OSError):
            pass

    def _serial_query(self, cmd: str) -> str:
        if self._serial_fd is None:
            return ''
        try:
            self._drain_fd(self._serial_fd)
            os.write(self._serial_fd, (cmd + '\n').encode('ascii'))
            buf = b''
            deadline = time.time() + self.serial_timeout
            while time.time() < deadline:
                try:
                    buf += os.read(self._serial_fd, 256)
                    if b'\n' in buf:
                        line = buf.split(b'\n')[0].decode('ascii', errors='ignore').strip()
                        if line.endswith('d'):
                            line = line[:-1]
                        return line
                except BlockingIOError:
                    time.sleep(0.005)
            return ''
        except OSError as e:
            self.get_logger().warn(f'Serial error: {e}', throttle_duration_sec=5.0)
            self._close_serial()
            return ''

    def _close_serial(self):
        if self._serial_fd is not None:
            try:
                os.close(self._serial_fd)
            except OSError:
                pass
            self._serial_fd = None

    def _open_can(self):
        try:
            import can
            self._can_bus = can.interface.Bus(channel=self.can_channel, interface='socketcan')
            self.get_logger().info(f'Power monitor: opened CAN {self.can_channel}')
            return True
        except Exception as e:
            self.get_logger().warn(f'Power monitor: CAN open failed: {e}', throttle_duration_sec=10.0)
            return False

    def _request_can_rtr(self, cmd_id: int):
        if self._can_bus is None:
            return
        try:
            import can
            arb_id = (self.right_node_id << 5) | (cmd_id & 0x1F)
            msg = can.Message(
                arbitration_id=arb_id,
                is_remote_frame=True,
                dlc=8,
                is_extended_id=False
            )
            self._can_bus.send(msg)
        except Exception as e:
            self.get_logger().debug(f'CAN RTR error: {e}')

    def _read_can_response(self, cmd_id: int, timeout: float = 0.15):
        if self._can_bus is None:
            return None
        try:
            deadline = time.time() + timeout
            while time.time() < deadline:
                msg = self._can_bus.recv(timeout=max(0.0, deadline - time.time()))
                if msg is None:
                    break
                node = (msg.arbitration_id >> 5) & 0x3F
                cmd = msg.arbitration_id & 0x1F
                if node == self.right_node_id and cmd == cmd_id and msg.dlc >= 4:
                    return struct.unpack_from('<f', bytes(msg.data), 0)[0]
        except Exception as e:
            self.get_logger().debug(f'CAN recv error: {e}')
        return None

    def _poll(self):
        # Left motor (USB serial)
        if self._serial_fd is None:
            self._open_serial()

        if self._serial_fd is not None:
            v_str = self._serial_query('r vbus_voltage')
            try:
                v = float(v_str)
                if 0.0 < v < 60.0:
                    self._left_voltage = v
            except ValueError:
                pass

            i_str = self._serial_query('r axis0.motor.current_control.Iq_measured')
            try:
                self._left_current = abs(float(i_str))
            except ValueError:
                pass

        # Right motor (CAN)
        CMD_GET_VBUS = 0x017
        CMD_GET_IBUS = 0x014

        if self._can_bus is None:
            self._open_can()

        if self._can_bus is not None:
            try:
                self._request_can_rtr(CMD_GET_VBUS)
                vbus = self._read_can_response(CMD_GET_VBUS)
                if vbus is not None and 0.0 < vbus < 60.0:
                    self._right_voltage = vbus

                self._request_can_rtr(CMD_GET_IBUS)
                ibus = self._read_can_response(CMD_GET_IBUS)
                if ibus is not None:
                    self._right_current = abs(ibus)
            except Exception as e:
                self.get_logger().debug(f'CAN poll error: {e}')
                try:
                    self._can_bus.shutdown()
                except Exception:
                    pass
                self._can_bus = None

        # Average and publish
        valid_v = [v for v in [self._left_voltage, self._right_voltage] if v > 0.0]
        valid_i = [i for i in [self._left_current, self._right_current] if i >= 0.0]

        if valid_v:
            vm = Float32()
            vm.data = float(sum(valid_v) / len(valid_v))
            self.volt_pub.publish(vm)

        if valid_i:
            cm = Float32()
            cm.data = float(sum(valid_i) / len(valid_i))
            self.curr_pub.publish(cm)

    def destroy_node(self):
        self._close_serial()
        if self._can_bus is not None:
            try:
                self._can_bus.shutdown()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PowerMonitorNode()
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
