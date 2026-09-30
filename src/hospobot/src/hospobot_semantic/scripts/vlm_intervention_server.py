#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hospobot_interfaces.srv import VLMIntervention
from geometry_msgs.msg import PoseStamped
import time

class VLMInterventionServer(Node):
    def __init__(self):
        super().__init__('vlm_intervention_server')
        self.srv = self.create_service(
            VLMIntervention, 
            '/vlm_intervention', 
            self.vlm_callback
        )
        self.get_logger().info('VLM Intervention Server started.')

    def vlm_callback(self, request, response):
        self.get_logger().info('Received image for VLM inference. Processing...')
        
        # Here you would pass request.image to your Large Vision-Language Model
        # e.g., prompt = "The robot is stuck. Based on this image, where should it move?"
        # bias_output = my_vlm_model.infer(request.image, prompt)
        
        # Simulated inference delay
        time.sleep(1.0)
        
        self.get_logger().info('VLM inference complete. Providing bias goal.')
        
        bias_goal = PoseStamped()
        bias_goal.header.stamp = self.get_clock().now().to_msg()
        bias_goal.header.frame_id = 'base_link'
        
        # Provide a short-horizon directional preference (e.g., move 0.5m forward and slightly left)
        bias_goal.pose.position.x = 0.5
        bias_goal.pose.position.y = 0.2
        bias_goal.pose.position.z = 0.0
        bias_goal.pose.orientation.w = 1.0
        
        response.bias_goal = bias_goal
        response.success = True
        response.message = "VLM guidance successfully generated."
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = VLMInterventionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
