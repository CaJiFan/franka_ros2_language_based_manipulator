import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import json
<<<<<<< Updated upstream
=======
import cv2
import cv2.aruco as aruco
>>>>>>> Stashed changes
from std_msgs.msg import String
from vision_module.yolo_detector_ultralytics import YOLODetectorUltralytics as YoloDetector
from vision_module.publishers.object_publisher import ObjectPublisher


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.class_names = [ 'bottle', 'fork', 'spoon']
        self.yolo_detector = YoloDetector(self.class_names)
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

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters()
        
        self.marker_length = 0.015

    def get_label_name(self, label_id):
        # Safety check: Is the ID inside the list range?
        if 0 <= label_id < len(self.class_names):
            return self.class_names[label_id]
        else:
            # Fallback if the model detects something weird
            return f"Unknown({label_id})"

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
<<<<<<< Updated upstream
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
        
=======
            label_id = obj['label']
            # Get bounding box (cx, cy, w, h) -> corners
            cx, cy, w, h = obj['position']
            x1 = int(cx - w/2)
            y1 = int(cy - h/2)
            x2 = int(cx + w/2)
            y2 = int(cy + h/2)
            
            # Clamp to image boundaries
            h_img, w_img = cv_image.shape[:2]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w_img, x2), min(h_img, y2)

            # --- HYBRID LOGIC START ---
            
            # 2. Crop the image to the YOLO box
            roi = cv_image[y1:y2, x1:x2]
            
            if roi.size == 0: continue

            # 3. Detect ArUco inside the ROI
            corners, ids, rejected = aruco.detectMarkers(
                roi, self.aruco_dict, parameters=self.aruco_params
            )

            final_x, final_y, final_z = None, None, None
            method = "YOLO" # Default if no marker found
            print(f"Detected ArUco IDs: {ids}")
            if ids is not None and len(ids) > 0:
                # MARKER FOUND!
                method = "ARUCO"
                
                # We usually take the first marker found in the box
                marker_corners = corners[0] 
                
                # IMPORTANT: The corners are relative to the ROI (Crop).
                # We must shift them back to the full image coordinates.
                marker_corners_full = marker_corners + [x1, y1]

                # Get the center of the marker
                c = marker_corners_full[0] # Shape (1, 4, 2)
                # Calculate centroid of the 4 corners
                cx_aruco = int(c[:, 0].mean())
                cy_aruco = int(c[:, 1].mean())

                # Draw the marker on debug image
                aruco.drawDetectedMarkers(debug_image, [marker_corners_full], ids[0])

                # --- OPTION A: Use Depth from Camera (Easiest) ---
                # Use the exact center pixel of the marker to look up depth
                depth_mm = self.latest_depth_image[cy_aruco, cx_aruco]
                z_cam = depth_mm / 1000.0

                # Deproject using lens intrinsics
                x_cam, y_cam = self.deproject(cx_aruco, cy_aruco, z_cam)

                if 0 <= cx_aruco < w_img and 0 <= cy_aruco < h_img:
                    depth_mm = self.latest_depth_image[cy_aruco, cx_aruco]
                    z_cam = depth_mm / 1000.0  # Convert to meters
                    x_cam, y_cam = self.deproject(cx_aruco, cy_aruco, z_cam)

                    # --- NEW: PRINT RAW COORDS ---
                    # print(f"[RAW CAM] ID:{ids[0][0]} | X: {x_cam:.3f} Y: {y_cam:.3f} Z: {z_cam:.3f}")

                    # --- NEW: DRAW RAW COORDS ON SCREEN ---
                    # Format: (Xc, Yc, Zc) in meters
                    raw_text = f"Cam: ({x_cam:.2f}, {y_cam:.2f}, {z_cam:.2f})"
                    
                    # Draw text slightly above the marker box
                    cv2.putText(debug_image, raw_text, (x1, y1 - 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                # --- OPTION B: Use ArUco Pose Estimation (More Precise Rotation) ---
                # This gives you precise Orientation too, but requires perfectly calibrated intrinsics.
                # rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
                
            else:
                # NO MARKER: Fallback to YOLO center
                depth_mm = self.latest_depth_image[int(cy), int(cx)]
                z_cam = depth_mm / 1000.0
                x_cam, y_cam = self.deproject(cx, cy, z_cam)

            # 4. Transform to Robot Frame (Common for both methods)
            if z_cam > 0:
                x_robot, y_robot, z_robot = self.transform_to_robot(x_cam, y_cam, z_cam)
                rob_text = f"Rob: ({x_robot:.2f}, {y_robot:.2f}, {z_robot:.2f})"
                cv2.putText(debug_image, rob_text, (x1, y1 - 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2) # Cyan color, slightly bigger
                
                # Add to results
                results_3d.append({
                    "label": self.get_label_name(label_id),
                    "id": int(ids[0][0]) if method == "ARUCO" else -1,
                    "method": method,
                    "x": x_robot, "y": y_robot, "z": z_robot
                })

                # Draw Text
                color = (0, 0, 255) if method == "ARUCO" else (0, 255, 0)
                cv2.rectangle(debug_image, (x1, y1), (x2, y2), color, 2)
                cv2.putText(debug_image, f"{method}", (x1, y1-10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


>>>>>>> Stashed changes
        # --- PUBLISH DEBUG IMAGE ---
        try:
            ros_image = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_publisher.publish(ros_image)
        except CvBridgeError:
            pass
        
        # ... (Existing JSON publishing logic) ...

    def transform_to_robot(self, x_cam, y_cam, z_cam):
        # HYPOTHESIS:
        # Robot X (Forward) <-> Camera -X (Left)
        # Robot Y (Left)    <-> Camera Y (Down)
        # Robot Z (Up)      <-> Camera -Z (Backwards)
        
        # 1. ROTATION
        x_rot = -x_cam  # Image Left is Robot Forward
        y_rot =  y_cam  # Image Down is Robot Left 
        z_rot = -z_cam  # Image Depth is Robot Down

        # 2. TRANSLATION (Recalculate these with the new rotation!)
        # Use your measured point (0.644, 0.060, 0.296) to solve:
        # 0.644 = -(-0.22) + X_offset  -> X_offset = 0.424
        # 0.060 = (-0.10) + Y_offset   -> Y_offset = 0.160
        # 0.196 = -(0.48) + Z_offset   -> Z_offset = 0.676
        
        cam_x_offset = 0.424
        cam_y_offset = 0.160
        cam_z_offset = 0.676

        x_final = x_rot + cam_x_offset
        y_final = y_rot + cam_y_offset
        z_final = z_rot + cam_z_offset

        return x_final, y_final, z_final


    def deproject(self, u, v, z):
        fx = self.intrinsics[0, 0]
        fy = self.intrinsics[1, 1]
        cx = self.intrinsics[0, 2]
        cy = self.intrinsics[1, 2]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return x, y

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