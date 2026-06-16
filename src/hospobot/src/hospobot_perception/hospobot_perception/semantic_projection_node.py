#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from hospobot_interfaces.msg import SemanticObject, SemanticObjectArray
from cv_bridge import CvBridge
import message_filters
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Point
import tf2_geometry_msgs

class SemanticProjectionNode(Node):
    def __init__(self):
        super().__init__('semantic_projection_node')
        self.get_logger().info('Initializing Semantic Projection Node...')
        
        self.cb_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()
        
        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Camera Intrinsics (Placeholder: should subscribe to CameraInfo)
        self.fx = 320.0
        self.fy = 320.0
        self.cx = 320.0
        self.cy = 240.0
        
        # Subscribers
        self.det_sub = message_filters.Subscriber(self, Detection2DArray, '/oakd/detections', callback_group=self.cb_group)
        self.depth_sub = message_filters.Subscriber(self, Image, '/oakd/depth/image_rect', callback_group=self.cb_group)
        
        # Synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer([self.det_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)
        
        # Publisher
        self.pub = self.create_publisher(SemanticObjectArray, '/semantic/raw_objects', 10)

    def sync_callback(self, det_msg, depth_msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
            
        semantic_array_msg = SemanticObjectArray()
        semantic_array_msg.header.stamp = det_msg.header.stamp
        semantic_array_msg.header.frame_id = 'odom'
        
        target_frame = 'odom'
        source_frame = depth_msg.header.frame_id
        
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform {source_frame} to {target_frame}: {ex}")
            return

        for det in det_msg.detections:
            # Transform to odom frame
            pt_opt = tf2_geometry_msgs.PointStamped()
            pt_opt.header.frame_id = source_frame
            pt_opt.header.stamp = det_msg.header.stamp
            
            # Check if 3D coordinates are already provided by the detection node
            if len(det.results) > 0 and det.results[0].pose.pose.position.z > 0:
                # OAK-D uses Camera Optical Frame (Z forward, X right, Y down)
                pt_opt.point.x = det.results[0].pose.pose.position.x
                pt_opt.point.y = det.results[0].pose.pose.position.y
                pt_opt.point.z = det.results[0].pose.pose.position.z
            else:
                u = int(det.bbox.center.position.x * depth_image.shape[1]) if det.bbox.center.position.x <= 1.0 else int(det.bbox.center.position.x)
                v = int(det.bbox.center.position.y * depth_image.shape[0]) if det.bbox.center.position.y <= 1.0 else int(det.bbox.center.position.y)
                
                # Bounds check
                if v < 0 or v >= depth_image.shape[0] or u < 0 or u >= depth_image.shape[1]:
                    continue
                    
                z_depth_mm = depth_image[v, u]
                if z_depth_mm == 0:
                    continue
                    
                z_depth = z_depth_mm / 1000.0 # Convert to meters
                
                # Pinhole projection
                x_opt = (u - self.cx) * z_depth / self.fx
                y_opt = (v - self.cy) * z_depth / self.fy
                z_opt = z_depth
                
                pt_opt.point.x = float(x_opt)
                pt_opt.point.y = float(y_opt)
                pt_opt.point.z = float(z_opt)
            
            try:
                pt_odom = tf2_geometry_msgs.do_transform_point(pt_opt, transform)
            except Exception as e:
                self.get_logger().warn(f"Transformation failed: {e}")
                continue
            
            obj = SemanticObject()
            obj.class_name = det.results[0].hypothesis.class_id if det.results else "Unknown"
            obj.confidence = det.results[0].hypothesis.score if det.results else 0.0
            obj.position.x = pt_odom.point.x
            obj.position.y = pt_odom.point.y
            obj.position.z = pt_odom.point.z
            
            semantic_array_msg.objects.append(obj)
            
        if semantic_array_msg.objects:
            self.pub.publish(semantic_array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticProjectionNode()
    # Use MultiThreadedExecutor due to ReentrantCallbackGroup
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
