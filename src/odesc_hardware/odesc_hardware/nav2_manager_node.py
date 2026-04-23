import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import subprocess
import os

class Nav2ManagerNode(Node):
    def __init__(self):
        super().__init__('nav2_manager_node')
        self.sub = self.create_subscription(String, '/control_mode', self.mode_cb, 10)
        self.nav2_process = None
        self.get_logger().info('Nav2 Manager Node started. Waiting for HTML mode switch...')

    def mode_cb(self, msg):
        mode = msg.data
        if mode == 'auto':
            if self.nav2_process is None or self.nav2_process.poll() is not None:
                self.get_logger().info('UI requested Autonomous mode. Launching Nav2 stack...')
                try:
                    rover_bringup_dir = get_package_share_directory('rover_bringup')
                    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
                    map_yaml = os.path.join(rover_bringup_dir, 'maps', 'hospital_map.yaml')
                    params_yaml = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
                    
                    cmd = [
                        'ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
                        f'map:={map_yaml}',
                        f'params_file:={params_yaml}',
                        'use_sim_time:=false'
                    ]
                    self.nav2_process = subprocess.Popen(cmd)
                except Exception as e:
                    self.get_logger().error(f"Failed to launch Nav2: {e}")
        
        elif mode == 'manual':
            if self.nav2_process is not None and self.nav2_process.poll() is None:
                self.get_logger().info('UI requested Manual mode. Shutting down Nav2 stack...')
                # Terminate the process group to ensure all child nodes are killed
                self.nav2_process.terminate()
                try:
                    self.nav2_process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self.nav2_process.kill()
                self.nav2_process = None

def main(args=None):
    rclpy.init(args=args)
    node = Nav2ManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.nav2_process and node.nav2_process.poll() is None:
            node.nav2_process.terminate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
