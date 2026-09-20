import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import sys

def activate_node(node_name):
    node = rclpy.create_node('activator_' + node_name.strip('/'))
    client = node.create_client(ChangeState, f'{node_name}/change_state')
    
    if not client.wait_for_service(timeout_sec=5.0):
        print(f"Service not available for {node_name}")
        return False

    req = ChangeState.Request()
    
    # Configure
    req.transition.id = Transition.TRANSITION_CONFIGURE
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    print(f"Configured {node_name}: {future.result().success}")

    # Activate
    req.transition.id = Transition.TRANSITION_ACTIVATE
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    print(f"Activated {node_name}: {future.result().success}")
    
    node.destroy_node()
    return True

def main():
    rclpy.init()
    activate_node('/map_server')
    activate_node('/amcl')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
