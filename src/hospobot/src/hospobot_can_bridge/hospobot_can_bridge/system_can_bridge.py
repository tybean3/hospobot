#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import struct
import time

from can_msgs.msg import Frame
from std_msgs.msg import Float32MultiArray, Float32, UInt8MultiArray, UInt16, Bool, UInt32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SystemCanBridge(Node):
    def __init__(self):
        super().__init__('system_can_bridge')
        
        # CAN Publisher and Subscriber
        self.can_pub = self.create_publisher(Frame, 'to_can_bus', 10)
        self.can_sub = self.create_subscription(Frame, 'from_can_bus', self.can_rx_callback, 10)
        
        # Sensor Publishers (ESP32 -> NUC)
        self.ultrasonic_pub = self.create_publisher(Float32MultiArray, 'sensors/ultrasonics', 10)
        self.beam_break_pub = self.create_publisher(Bool, 'sensors/beam_break', 10)
        self.rfid_pub = self.create_publisher(UInt32, 'sensors/rfid', 10)
        self.voltage_pub = self.create_publisher(Float32, 'sensors/bus_voltage', 10)
        self.current_pub = self.create_publisher(Float32, 'sensors/bus_current', 10)
        
        # Actuator Subscribers (NUC -> ESP32)
        # Array of 16 bytes: [M1, B1, M2, B2, ... M8, B8]
        self.led_filaments_sub = self.create_subscription(UInt8MultiArray, 'actuators/led_filaments', self.led_filaments_callback, 10)
        self.led_indicators_sub = self.create_subscription(UInt16, 'actuators/led_indicators', self.led_indicators_callback, 10)
        self.draw_lock_sub = self.create_subscription(Bool, 'actuators/draw_lock', self.draw_lock_callback, 10)
        
        # Diagnostics
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)
        self.diag_timer = self.create_timer(1.0, self.diag_timer_callback)
        
        # Node tracking for diagnostics
        self.node_last_seen = {
            0x1A: 0.0,
            0x1B: 0.0,
            0x1C: 0.0
        }
        
        # State arrays
        self.ultrasonics_data = [0.0] * 10
        
        self.get_logger().info('System CAN Bridge Node initialized with updated ESP32 A/B/C support')

    def can_rx_callback(self, msg: Frame):
        node_id = (msg.id >> 5) & 0x3F
        cmd_id = msg.id & 0x1F
        
        if node_id in self.node_last_seen:
            self.node_last_seen[node_id] = time.time()
            self.decode_esp32_message(node_id, cmd_id, msg.data, msg.dlc)
            
    def decode_esp32_message(self, node_id, cmd_id, data, dlc):
        if cmd_id == 0x01:
            pass # Heartbeat handled in rx_callback
            
        elif node_id == 0x1A:
            # ESP32-A: Ultrasonics
            if 0x02 <= cmd_id <= 0x06 and dlc >= 8:
                idx = (cmd_id - 0x02) * 2
                v1, v2 = struct.unpack('<ff', bytes(data[:8]))
                if idx < 10:
                    self.ultrasonics_data[idx] = v1
                    self.ultrasonics_data[idx+1] = v2
                    
                # Publish on receiving the last frame (0x06) to sync
                if cmd_id == 0x06:
                    msg = Float32MultiArray()
                    msg.data = self.ultrasonics_data
                    self.ultrasonic_pub.publish(msg)
                    
        elif node_id == 0x1B:
            # ESP32-B: Beam & RFID
            if cmd_id == 0x02 and dlc >= 5:
                b_msg = Bool()
                b_msg.data = bool(data[0] > 0)
                self.beam_break_pub.publish(b_msg)
                
                rfid_uid = struct.unpack('<I', bytes(data[1:5]))[0]
                if rfid_uid != 0:
                    r_msg = UInt32()
                    r_msg.data = rfid_uid
                    self.rfid_pub.publish(r_msg)
            elif cmd_id == 0x03 and dlc >= 4:
                # Top ESP32 proximity test value
                test_dist = struct.unpack('<f', bytes(data[:4]))[0]
                self.get_logger().info(f"Received Top ESP32 (ESP32-B) proximity test value: {test_dist:.2f}m")
            elif cmd_id == 0x0E:
                # Parrot back filaments 1-4
                self.get_logger().info(f"Verified LED Filaments 1-4 parroted back from ESP32-B: {list(data[:dlc])}")
            elif cmd_id == 0x0F:
                # Parrot back filaments 5-6
                self.get_logger().info(f"Verified LED Filaments 5-6 parroted back from ESP32-B: {list(data[:dlc])}")
                    
        elif node_id == 0x1C:
            # ESP32-C: INA226 Power
            if cmd_id == 0x02 and dlc >= 8:
                v, i = struct.unpack('<ff', bytes(data[:8]))
                v_msg, i_msg = Float32(), Float32()
                v_msg.data, i_msg.data = v, i
                self.voltage_pub.publish(v_msg)
                self.current_pub.publish(i_msg)
            elif cmd_id == 0x0E:
                # Parrot back filaments 7-8
                self.get_logger().info(f"Verified LED Filaments 7-8 parroted back from ESP32-C: {list(data[:dlc])}")


    # --- Actuator Callbacks (Encoding) ---
    def send_can_frame(self, node_id, cmd_id, data_bytes):
        msg = Frame()
        msg.id = (node_id << 5) | (cmd_id & 0x1F)
        msg.is_rtr = False
        msg.is_extended = False
        msg.is_error = False
        msg.dlc = len(data_bytes)
        msg.data = data_bytes + [0] * (8 - len(data_bytes))
        self.can_pub.publish(msg)

    def led_filaments_callback(self, msg: UInt8MultiArray):
        # We expect 16 bytes: [M1, B1, M2, B2 ... M8, B8]
        if len(msg.data) >= 16:
            # ESP32-B Filaments (1-4)
            self.send_can_frame(0x1B, 0x0A, list(msg.data[0:8]))
            # ESP32-B Filaments (5-6)
            self.send_can_frame(0x1B, 0x0B, list(msg.data[8:12]))
            # ESP32-C Filaments (7-8)
            self.send_can_frame(0x1C, 0x0A, list(msg.data[12:16]))

    def led_indicators_callback(self, msg: UInt16):
        # ESP32-B LED Indicators (Bitmask)
        data = struct.pack('<H', msg.data)
        self.send_can_frame(0x1B, 0x0C, list(data))

    def draw_lock_callback(self, msg: Bool):
        # ESP32-B Draw Lock
        state = 1 if msg.data else 0
        self.send_can_frame(0x1B, 0x0D, [state])

    def diag_timer_callback(self):
        diag_arr = DiagnosticArray()
        diag_arr.header.stamp = self.get_clock().now().to_msg()
        
        current_time = time.time()
        timeout = 2.0 
        
        node_names = {0x1A: "ESP32-A", 0x1B: "ESP32-B", 0x1C: "ESP32-C"}
        
        for node_id, last_seen in self.node_last_seen.items():
            status = DiagnosticStatus()
            status.name = node_names[node_id]
            status.hardware_id = f"0x{node_id:X}"
            
            if last_seen == 0.0:
                status.level = DiagnosticStatus.WARN
                status.message = "Node has not been seen yet"
            elif current_time - last_seen > timeout:
                status.level = DiagnosticStatus.ERROR
                status.message = "Timeout: No heartbeat received"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "OK"
                
            status.values.append(KeyValue(key="Last Seen (s ago)", value=str(current_time - last_seen)))
            diag_arr.status.append(status)
            
        if len(diag_arr.status) > 0:
            self.diag_pub.publish(diag_arr)


def main(args=None):
    rclpy.init(args=args)
    node = SystemCanBridge()
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
