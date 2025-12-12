import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import json
from std_msgs.msg import String
from vision_module.yolo_detector_ultralytics import YOLODetectorUltralytics as YoloDetector
from vision_module.publishers.object_publisher import ObjectPublisher

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.yolo_detector = YoloDetector()
        self.bridge = CvBridge()

        # 1. CAMERA INTRINSICS (Lens parameters)
        self.intrinsics = None
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_callback, 10)

        # 2. SUBSCRIBERS (RGB and Depth)
        # Note: In RealSense, use the "aligned_depth_to_color" topic!
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        
        self.object_publisher = ObjectPublisher(self)
        self.debug_publisher = self.create_publisher(Image, 'debug_image', 10)

        self.latest_depth_image = None

    def info_callback(self, msg):
        # We only need this once to get fx, fy, cx, cy
        if self.intrinsics is None:
            self.intrinsics = np.array(msg.k).reshape((3, 3))
            self.get_logger().info('Camera Intrinsics Received!')
    
    def depth_callback(self, msg):
        try:
            # 16UC1 means 16-bit unsigned integer (depth in millimeters)
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'Depth error: {e}')

    def image_callback(self, msg):
        if self.intrinsics is None or self.latest_depth_image is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError:
            return

        detections = self.yolo_detector.detect_objects(cv_image)
        results_3d = []

        # --- DRAWING LOOP ---
        debug_image = cv_image.copy()
        
        for obj in detections:
            # Draw Box
            box = obj['box'] # [x1, y1, x2, y2]
            x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw Label
            label = f"{obj['label']}"
            cv2.putText(debug_image, label, (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # ... (Existing 3D calculation logic) ...
            
            # (Optional) Add coordinate text to the image
            # label += f" ({x_robot:.2f}, {y_robot:.2f})"
        
        # --- PUBLISH DEBUG IMAGE ---
        try:
            ros_image = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_publisher.publish(ros_image)
        except CvBridgeError:
            pass
        
        # ... (Existing JSON publishing logic) ...

    def transform_to_robot(self, x_cam, y_cam, z_cam):
        """
        Converts RealSense Optical Frame -> Franka Base Frame
        """
        # --- CONFIGURATION (MEASURE THESE!) ---
        # Where is the camera relative to the robot base? (in meters)
        # Example: Camera is 1 meter to the right (y), 0.5m up (z), looking at the workspace
        CAMERA_X_OFFSET = 0.5  
        CAMERA_Y_OFFSET = -0.5 
        CAMERA_Z_OFFSET = 0.5

        # --- ROTATION ---
        # RealSense: X=Right, Y=Down, Z=Forward
        # Robot: X=Forward, Y=Left, Z=Up
        
        # Scenario: Camera is Perpendicular to Franka X axis.
        # This usually means looking from the SIDE (looking along Y axis).
        
        # Let's assume a simplified static transform for a side-view camera.
        # You effectively need to map axes.
        
        # Example Logic:
        # Camera Z (Depth) points towards Robot Y (Left)
        # Camera X (Right) points towards Robot -X (Backwards)
        # Camera Y (Down) points towards Robot -Z (Down)
        
        # This is a matrix multiplication. 
        # Point_Robot = Rotation_Matrix @ Point_Camera + Translation_Vector
        
        point_cam = np.array([x_cam, y_cam, z_cam])
        
        # Rotation Matrix (Example for Side View looking Left)
        # You must tune this based on your EXACT mounting
        # Rows are Robot X, Y, Z
        R = np.array([
            [ 0,  0, -1],  # Robot X comes from Camera -Z (or similar)
            [-1,  0,  0],  # Robot Y comes from Camera -X
            [ 0, -1,  0]   # Robot Z comes from Camera -Y
        ])
        
        # Apply Rotation
        point_rotated = R @ point_cam
        
        # Apply Translation (Offset)
        x_final = point_rotated[0] + CAMERA_X_OFFSET
        y_final = point_rotated[1] + CAMERA_Y_OFFSET
        z_final = point_rotated[2] + CAMERA_Z_OFFSET

        return float(x_final), float(y_final), float(z_final)

def main(argv=None):
    rclpy.init(args=argv)
    try:
        node = VisionNode()
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            print('error destroying node')
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()