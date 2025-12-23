import cv2
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

def get_camera_transform_from_marker(color_image, camera_matrix, dist_coeffs):
    # --- CONFIGURATION ---
    marker_size = 0.05  # Size of the printed ArUco in meters (e.g., 5cm)
    
    # KNOWN POSE: Where did you stick the marker on the table?
    # Example: 30cm forward (x), centered (y), flat on table (z)
    T_base_marker = np.eye(4)
    T_base_marker[0:3, 3] = [0.30, 0.0, 0.0] 
    # If marker is flat on table, its Z-axis points UP, same as robot.
    # So rotation is identity (or aligned with how you stuck it).
    # 2. ROTATION:
    # If Marker "Top" points to Robot, and Marker "Face" is up:
    # Robot X (Forward) = Marker -Y
    # Robot Y (Left)    = Marker -X
    # Robot Z (Up)      = Marker Z
    
    # Rotation Matrix columns are the Marker's [X, Y, Z] axes expressed in Robot Frame:
    # Marker X axis points to Robot Right (-Y) -> [ 0, -1,  0]
    # Marker Y axis points to Robot Back (-X)  -> [-1,  0,  0]
    # Marker Z axis points Up (+Z)             -> [ 0,  0,  1]
    
    T_base_marker[0:3, 0:3] = np.array([
        [ 0, -1,  0],
        [-1,  0,  0],
        [ 0,  0,  1]
    ])

    # --- ARUCO DETECTION ---
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    corners, ids, rejected = detector.detectMarkers(color_image)

    if ids is not None and len(ids) > 0:
        # Assume we look for Marker ID 0
        index = np.where(ids == 0)[0][0]
        
        # Estimate Pose of Marker in Camera Frame (rvec, tvec)
        # Obj Points: Corners of the marker in its own frame
        obj_points = np.array([
            [-marker_size/2, marker_size/2, 0],
            [marker_size/2, marker_size/2, 0],
            [marker_size/2, -marker_size/2, 0],
            [-marker_size/2, -marker_size/2, 0]
        ], dtype=np.float32)

        success, rvec, tvec = cv2.solvePnP(obj_points, corners[index], camera_matrix, dist_coeffs)
        
        if success:
            # --- MATH: TRANSFORM CHAIN ---
            # We have:
            # 1. T_base_marker (Known physical measurement)
            # 2. T_cam_marker  (Calculated from Vision)
            # We want: T_base_cam
            
            # Convert rvec/tvec to 4x4 Matrix
            rmat, _ = cv2.Rodrigues(rvec)
            T_cam_marker = np.eye(4)
            T_cam_marker[0:3, 0:3] = rmat
            T_cam_marker[0:3, 3] = tvec.T[0]

            # Equation: T_base_marker = T_base_cam * T_cam_marker
            # Therefore: T_base_cam = T_base_marker * inverse(T_cam_marker)
            
            T_base_cam = np.dot(T_base_marker, np.linalg.inv(T_cam_marker))
            
            # Extract final Translation (x,y,z) and Rotation (quat) for ROS
            pos = T_base_cam[0:3, 3]
            rot = R.from_matrix(T_base_cam[0:3, 0:3]).as_quat() # x, y, z, w
            
            print(f"FOUND CAMERA! \nPosition: {pos} \nRotation: {rot}")
            return pos, rot
            
    print("Calibration Marker not found!")
    return None, None