#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import depthai as dai
import numpy as np



class OakdYoloNode(Node):
    def __init__(self):
        super().__init__('oakd_yolo_node')
        self.get_logger().info('Initializing OAK-D YOLOv8 Node...')
        
        # QoS parameters
        self.declare_parameter('qos_reliability', 'best_effort')
        qos_reliability_str = self.get_parameter('qos_reliability').get_parameter_value().string_value
        
        if qos_reliability_str == 'best_effort':
            qos_profile = qos_profile_sensor_data
        else:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.VOLATILE
            )
            
        # Publishers
        self.det_pub = self.create_publisher(Detection2DArray, '/oakd/detections', 10)
        self.depth_pub = self.create_publisher(Image, '/oakd/depth/image_rect', 10)
        self.rgb_pub = self.create_publisher(Image, '/oakd/rgb/image_rect', qos_profile)
        self.info_pub = self.create_publisher(CameraInfo, '/oakd/camera_info', 10)
        self.points_pub = self.create_publisher(PointCloud2, '/oakd/points', qos_profile)
        
        self.bridge = CvBridge()
        
        # PointField layout for XYZRGB (packed float representation)
        self.point_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        # Cache for synchronization
        self.last_rgb_frame = None
        self.last_depth_frame = None
        self.calib_intrinsics = None
        
        # Cached grid for point cloud optimization
        self.cached_width = 0
        self.cached_height = 0
        self.u_grid = None
        self.v_grid = None
        
        # Declare and get parameters
        self.declare_parameter('blob_path', 'path/to/your/yolov8_blob.blob')
        blob_path = self.get_parameter('blob_path').get_parameter_value().string_value
        
        self.declare_parameter('update_rate', 15.0)
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        
        self.declare_parameter('point_cloud_decimation', 2)
        self.point_cloud_decimation = self.get_parameter('point_cloud_decimation').get_parameter_value().integer_value
        
        self.use_yolo = False
        if blob_path and blob_path != 'path/to/your/yolov8_blob.blob' and os.path.exists(blob_path):
            self.use_yolo = True
            self.get_logger().info(f"YOLO Blob path found: '{blob_path}'. Initializing spatial object detection.")
        else:
            self.get_logger().warning("YOLO Blob path not provided or file does not exist. Falling back to Depth + RGB stream mode only.")

        # DepthAI Pipeline Setup
        self.pipeline = dai.Pipeline()
        
        # Define sources and outputs
        self.cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        self.mono_left = self.pipeline.create(dai.node.MonoCamera)
        self.mono_right = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        
        self.xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        self.xout_depth = self.pipeline.create(dai.node.XLinkOut)
        
        self.xout_rgb.setStreamName("rgb")
        self.xout_depth.setStreamName("depth")
        
        # Properties
        if self.use_yolo:
            self.cam_rgb.setPreviewSize(300, 300)
        else:
            self.cam_rgb.setPreviewSize(640, 400)
            
        self.cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        self.cam_rgb.setFps(self.update_rate)
        self.cam_rgb.setInterleaved(False)
        self.cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        
        self.mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.mono_left.setCamera("left")
        self.mono_left.setFps(self.update_rate)
        self.mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.mono_right.setCamera("right")
        self.mono_right.setFps(self.update_rate)
        
        # StereoDepth Setup & Hardware Filtering to reduce noise
        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        self.stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A) # Align depth to RGB
        self.stereo.setLeftRightCheck(True) # Occlusion check to clean edges
        self.stereo.setMedianFilter(dai.MedianFilter.KERNEL_5x5) # High-speed noise filter
        self.stereo.setSubpixel(False) # Disable subpixel to boost VPU speed
        
        # Post-Processing configs to run directly on camera chip (MyriadX)
        config = self.stereo.initialConfig.get()
        config.postProcessing.decimationFilter.decimationFactor = 1 # Keep 1:1 scale
        config.postProcessing.spatialFilter.enable = False # Disable to save memory for NN
        config.postProcessing.temporalFilter.enable = False # Disable to save memory for NN
        config.postProcessing.speckleFilter.enable = False # Disable to save memory for NN
        config.postProcessing.thresholdFilter.minRange = 200 # 20 cm
        # Keep depth at standard 640x400 to prevent Stereo block matching crashes
        self.stereo.setOutputSize(640, 400)
        
        if self.use_yolo:
            self.spatial_det_nn = self.pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
            self.xout_nn = self.pipeline.create(dai.node.XLinkOut)
            self.xout_nn.setStreamName("detections")
            
            # MobileNet Spatial Detection Network
            self.spatial_det_nn.setBlobPath(blob_path)
            self.spatial_det_nn.setConfidenceThreshold(0.5)
            self.spatial_det_nn.input.setBlocking(False)
            self.spatial_det_nn.setBoundingBoxScaleFactor(0.5)
            self.spatial_det_nn.setDepthLowerThreshold(100)
            self.spatial_det_nn.setDepthUpperThreshold(5000)
            
            # Linking
            self.mono_left.out.link(self.stereo.left)
            self.mono_right.out.link(self.stereo.right)
            
            self.cam_rgb.preview.link(self.spatial_det_nn.input)
            self.spatial_det_nn.out.link(self.xout_nn.input)
            
            self.stereo.depth.link(self.spatial_det_nn.inputDepth)
            self.spatial_det_nn.passthroughDepth.link(self.xout_depth.input)
            self.spatial_det_nn.passthrough.link(self.xout_rgb.input)
        else:
            # Linking without YOLO
            self.mono_left.out.link(self.stereo.left)
            self.mono_right.out.link(self.stereo.right)
            
            self.cam_rgb.preview.link(self.xout_rgb.input)
            self.stereo.depth.link(self.xout_depth.input)
            
        # Connect to device and start pipeline
        try:
            self.device = dai.Device(self.pipeline)
            
            # Read calibration data
            try:
                self.calib_data = self.device.readCalibration()
                self.get_logger().info("Successfully loaded camera calibration from OAK-D device.")
            except Exception as calib_err:
                self.calib_data = None
                self.get_logger().warning(f"Could not read calibration from device: {calib_err}. Using generic defaults.")
                
            self.q_rgb = self.device.getOutputQueue("rgb", 4, False)
            self.q_depth = self.device.getOutputQueue("depth", 4, False)
            if self.use_yolo:
                self.q_det = self.device.getOutputQueue("detections", 4, False)
            
            # Start timer for processing
            timer_period = 1.0 / self.update_rate
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.get_logger().info(f'OAK-D Pipeline started at {self.update_rate} Hz.')
        except Exception as e:
            self.get_logger().error(f"Failed to start OAK-D pipeline: {e}")

    def timer_callback(self):
        if not hasattr(self, 'device'):
            return
            
        # Get the latest frame from the queue to prevent latency accumulation
        in_rgb = None
        while True:
            frame = self.q_rgb.tryGet()
            if frame is None:
                break
            in_rgb = frame
            
        in_depth = None
        while True:
            frame = self.q_depth.tryGet()
            if frame is None:
                break
            in_depth = frame
        
        # Check subscriber status
        rgb_subbed = self.rgb_pub.get_subscription_count() > 0
        points_subbed = self.points_pub.get_subscription_count() > 0
        depth_subbed = self.depth_pub.get_subscription_count() > 0
        
        # Retrieve frames only if needed
        if in_rgb is not None and (rgb_subbed or points_subbed):
            self.last_rgb_frame = in_rgb.getCvFrame()
            
        if in_depth is not None and (depth_subbed or points_subbed):
            self.last_depth_frame = in_depth.getFrame()
            
        # Ensure intrinsics are loaded
        if self.calib_intrinsics is None:
            w = 640
            h = 640 if self.use_yolo else 400
            if self.calib_data is not None:
                try:
                    intrinsics = self.calib_data.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, w, h)
                    self.calib_intrinsics = (intrinsics[0][0], intrinsics[1][1], intrinsics[0][2], intrinsics[1][2])
                except Exception:
                    self.calib_intrinsics = (w * 0.8, w * 0.8, w / 2.0, h / 2.0)
            else:
                self.calib_intrinsics = (w * 0.8, w * 0.8, w / 2.0, h / 2.0)

        fx, fy, cx, cy = self.calib_intrinsics
        now = self.get_clock().now().to_msg()
            
        # Process Detections and draw on RGB frame
        det_array_msg = None
        if self.use_yolo:
            in_det = self.q_det.tryGet()
            if in_det is not None:
                det_array_msg = Detection2DArray()
                det_array_msg.header.stamp = now
                det_array_msg.header.frame_id = "oakd_frame"
                
                detections = in_det.detections
                for detection in detections:
                    det_msg = Detection2D()
                    
                    x1, y1 = detection.xmin, detection.ymin
                    x2, y2 = detection.xmax, detection.ymax
                    
                    det_msg.bbox.center.position.x = (x1 + x2) / 2.0
                    det_msg.bbox.center.position.y = (y1 + y2) / 2.0
                    det_msg.bbox.size_x = x2 - x1
                    det_msg.bbox.size_y = y2 - y1
                    
                    hyp = ObjectHypothesisWithPose()
                    class_label = "HUMAN" if str(detection.label) == "15" else f"CLASS_{detection.label}"
                    hyp.hypothesis.class_id = class_label
                    hyp.hypothesis.score = detection.confidence
                    
                    # Pack the robust 3D coordinates from DepthAI
                    hyp.pose.pose.position.x = float(detection.spatialCoordinates.x) / 1000.0
                    hyp.pose.pose.position.y = float(detection.spatialCoordinates.y) / 1000.0
                    hyp.pose.pose.position.z = float(detection.spatialCoordinates.z) / 1000.0
                    
                    det_msg.results.append(hyp)
                    det_array_msg.detections.append(det_msg)
                    
                    # Draw bounding box on RGB frame
                    if self.last_rgb_frame is not None:
                        h, w = self.last_rgb_frame.shape[:2]
                        px1 = int(x1 * w)
                        py1 = int(y1 * h)
                        px2 = int(x2 * w)
                        py2 = int(y2 * h)
                        import cv2
                        cv2.rectangle(self.last_rgb_frame, (px1, py1), (px2, py2), (0, 255, 0), 2)
                        cv2.putText(self.last_rgb_frame, f"{class_label} {detection.confidence:.2f}", 
                                    (px1, py1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
        # Publish RGB
        if rgb_subbed and self.last_rgb_frame is not None:
            rgb_msg = self.bridge.cv2_to_imgmsg(self.last_rgb_frame, encoding="bgr8")
            rgb_msg.header.stamp = now
            rgb_msg.header.frame_id = "oakd_frame"
            self.rgb_pub.publish(rgb_msg)
            
        # Publish Depth Image (16-bit uint)
        if depth_subbed and self.last_depth_frame is not None:
            depth_msg = self.bridge.cv2_to_imgmsg(self.last_depth_frame, encoding="16UC1")
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = "oakd_frame"
            self.depth_pub.publish(depth_msg)
            
        # Publish Camera Info
        if depth_subbed or rgb_subbed or points_subbed:
            h, w = (300, 300) if self.use_yolo else (400, 640)
            if self.last_depth_frame is not None:
                h, w = self.last_depth_frame.shape[:2]
            
            info_msg = CameraInfo()
            info_msg.header.stamp = now
            info_msg.header.frame_id = "oakd_frame"
            info_msg.width = w
            info_msg.height = h
            info_msg.k = [float(fx), 0.0, float(cx), 0.0, float(fy), float(cy), 0.0, 0.0, 1.0]
            info_msg.p = [float(fx), 0.0, float(cx), 0.0, 0.0, float(fy), float(cy), 0.0, 0.0, 0.0, 1.0, 0.0]
            info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            info_msg.distortion_model = "plumb_bob"
            self.info_pub.publish(info_msg)
            
        # Generate and publish colorized PointCloud2
        if points_subbed and self.last_rgb_frame is not None and self.last_depth_frame is not None:
            try:
                decimation = max(1, self.point_cloud_decimation)
                
                # Convert depth to meters (OAK-D is in mm) and downsample
                depth_m = self.last_depth_frame[::decimation, ::decimation].astype(np.float32) / 1000.0
                
                # Mask valid depth values within range (0.1m to 6.0m)
                valid_mask = (depth_m > 0.1) & (depth_m < 6.0)
                
                # Generate pixel grid coordinates and cache them
                height, width = depth_m.shape
                if width != self.cached_width or height != self.cached_height:
                    self.u_grid, self.v_grid = np.meshgrid(np.arange(width), np.arange(height))
                    self.cached_width = width
                    self.cached_height = height
                
                # Apply mask
                u_val = self.u_grid[valid_mask]
                v_val = self.v_grid[valid_mask]
                z_val = depth_m[valid_mask]
                
                # Scale intrinsics for the downsampled grid
                fx_dec = fx / decimation
                fy_dec = fy / decimation
                cx_dec = cx / decimation
                cy_dec = cy / decimation
                
                # Project pixels to 3D camera coordinates
                x_val = (u_val - cx_dec) * z_val / fx_dec
                y_val = (v_val - cy_dec) * z_val / fy_dec
                
                # Extract and pack BGR color channels
                # Resize RGB to match the decimated depth map if they don't match
                if self.last_rgb_frame.shape[:2] != depth_m.shape[:2]:
                    import cv2
                    rgb_resized = cv2.resize(self.last_rgb_frame, (depth_m.shape[1], depth_m.shape[0]))
                    bgr_val = rgb_resized[valid_mask]
                else:
                    bgr_val = self.last_rgb_frame[::decimation, ::decimation][valid_mask]
                    
                b = bgr_val[:, 0].astype(np.uint32)
                g = bgr_val[:, 1].astype(np.uint32)
                r = bgr_val[:, 2].astype(np.uint32)
                
                # Pack colors into a single 32-bit float (0x00RRGGBB format)
                rgb_packed = (r << 16) | (g << 8) | b
                rgb_float = rgb_packed.view(np.float32)
                
                # Create structured array for fast ROS serialization
                point_data = np.zeros(x_val.shape[0], dtype=[
                    ('x', np.float32),
                    ('y', np.float32),
                    ('z', np.float32),
                    ('rgb', np.float32)
                ])
                point_data['x'] = x_val
                point_data['y'] = y_val
                point_data['z'] = z_val
                point_data['rgb'] = rgb_float
                
                # Create Header
                pc_header = Header()
                pc_header.stamp = now
                pc_header.frame_id = "oakd_frame"
                
                # Create PointCloud2 message
                pc_msg = pc2.create_cloud(pc_header, self.point_fields, point_data)
                self.points_pub.publish(pc_msg)
            except Exception as pc_err:
                self.get_logger().error(f"Failed to generate point cloud: {pc_err}")
            
        if self.use_yolo and det_array_msg is not None:
            self.det_pub.publish(det_array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OakdYoloNode()
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

