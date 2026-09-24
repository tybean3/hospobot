import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import subprocess

class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('diagnostics_node')
        self.pub = self.create_publisher(String, '/web_diagnostics', 1)
        self.timer = self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('DiagnosticsNode started.')

    def timer_cb(self):
        try:
            nodes = self.get_node_names()
        except Exception:
            nodes = []

        try:
            topic_names_types = self.get_topic_names_and_types()
            topics = [t[0] for t in topic_names_types]
        except Exception:
            topics = []

        # OS Checks
        try:
            # OAK-D
            lsusb_out = subprocess.check_output(['lsusb'], text=True)
            oakd_plugged = '03e7:2485' in lsusb_out or 'Movidius' in lsusb_out or 'Luxonis' in lsusb_out
        except:
            oakd_plugged = False

        try:
            # USB-CAN Adapter (OpenMoko / Geschwister Schneider CAN, ID 1d50:606f)
            lsusb_can = subprocess.check_output(['lsusb'], text=True)
            can_plugged = '1d50:606f' in lsusb_can
        except:
            can_plugged = False

        try:
            # Ethernet
            ip_link_eth = subprocess.check_output(['ip', '-br', 'link', 'show'], text=True)
            eth_plugged = any(line for line in ip_link_eth.splitlines() if ('eth' in line or 'en' in line) and 'UP' in line)
            # SSH
            ssh_connected = 'SSH_CONNECTION' in subprocess.check_output(['env'], text=True) or len(subprocess.check_output(['ss', '-tnp'], text=True).split('sshd')) > 1
        except:
            eth_plugged = False
            ssh_connected = False

        try:
            # WiFi
            wifi_ssid = subprocess.check_output(['iwgetid', '-r'], text=True).strip()
        except:
            wifi_ssid = ""

        try:
            # Bluetooth
            bt_out = subprocess.check_output(['bluetoothctl', 'devices', 'Connected'], text=True)
            bt_connected = len(bt_out.strip()) > 0
        except:
            bt_connected = False

        diag_data = {
            'battery': 85,
            'ping': 15,
            'raw_nodes': nodes,
            'raw_topics': topics,
            'os': {
                'oakd': oakd_plugged,
                'can': can_plugged,
                'eth': eth_plugged,
                'ssh': ssh_connected,
                'wifi_ssid': wifi_ssid,
                'bt': bt_connected
            },
            'pipelines': {
                'driving': 'odesc_drive_node' in nodes and 'cmd_vel_mux' in nodes,
                'nav2': 'nav2_manager_node' in nodes or 'bt_navigator' in nodes,
                'semantic': any(n in nodes for n in ['oak', 'oak_container', 'oakd_yolo_node', 'semantic_tracker_node']),
                'comms': 'system_can_bridge' in nodes and 'rosbridge_websocket' in nodes
            },
            'nodes': {
                'node_lidar': 'FUNCTIONAL' if any(n in nodes for n in ['sllidar_node', 'rplidar_node']) else 'OFFLINE',
                'node_imu': 'FUNCTIONAL' if 'bno086_node' in nodes else 'OFFLINE',
                'node_ekf': 'FUNCTIONAL' if 'ekf_filter_node' in nodes else 'OFFLINE',
                'node_slam': 'FUNCTIONAL' if 'slam_toolbox' in nodes else 'OFFLINE',
                'node_nav': 'FUNCTIONAL' if any(n in nodes for n in ['nav2_manager_node', 'controller_server', 'bt_navigator']) else 'OFFLINE',
                'node_base': 'FUNCTIONAL' if 'odesc_drive_node' in nodes else 'OFFLINE',
                'node_mux': 'FUNCTIONAL' if 'cmd_vel_mux' in nodes else 'OFFLINE',
                'node_diag': 'FUNCTIONAL',
                'node_rsp': 'FUNCTIONAL' if 'robot_state_publisher' in nodes else 'OFFLINE',
                'node_bridge': 'FUNCTIONAL' if 'rosbridge_websocket' in nodes else 'OFFLINE',
                'node_camera': 'FUNCTIONAL' if any(n in nodes for n in ['oak', 'oak_container', 'oakd_yolo_node']) else 'OFFLINE'
            },
            'topics': {
                'topic_scan': 'FUNCTIONAL' if '/scan' in topics else 'OFFLINE',
                'topic_imu': 'FUNCTIONAL' if '/imu/data' in topics else 'OFFLINE',
                'topic_odom': 'FUNCTIONAL' if '/odom' in topics else 'OFFLINE',
                'topic_tf': 'FUNCTIONAL' if any(t in topics for t in ['/tf', '/tf_static']) else 'OFFLINE',
                'topic_map': 'FUNCTIONAL' if '/map' in topics else 'OFFLINE',
                'topic_cmd': 'FUNCTIONAL' if '/cmd_vel' in topics else 'OFFLINE',
                'topic_cmd_out': 'FUNCTIONAL' if '/cmd_vel_out' in topics else 'OFFLINE',
                'action_nav': 'FUNCTIONAL' if any('/navigate_to_pose' in t for t in topics) else 'OFFLINE'
            }
        }
        msg = String()
        msg.data = json.dumps(diag_data)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
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
