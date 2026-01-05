#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image , CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraViewer(Node):
    def __init__(self):
        self.debug_camera = False
        self.debug_crates = True
        self.i = 0
        super().__init__('left_camera_viewer')

        self.subscription_info = self.create_subscription(CameraInfo, '/left_camera/camera_info',
                                                           self.set_camera_info,10)
        
        self.subscription = self.create_subscription(
            Image, '/left_camera/image_raw', self.image_callback, 10)
        
        self.bridge = CvBridge()
        

        # Camera intrinsic parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_matrix = None
        self.dist_coeffs = None

        # Camera pose in world frame
        self.final_world_to_cam_transform = None

        # ArUco marker parameters
        self.aruco_marker_size = 0.1  # meters
        self.aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self.aruco_detector_parameters = cv2.aruco.DetectorParameters_create()

        # Marker world positions (x, y, z) in meters
        self.world_marker_coordinates = {
            20: [0.6, 1.4, 0],
            21: [2.4, 1.4, 0], 
            22: [0.6, 0.6, 0],
            23: [2.4, 0.6, 0]
        }

        # Crate ArUco parameters
        self.crate_aruco_size = 0.0375  # meters
        self.crate_aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self.crate_detector_parameters = cv2.aruco.DetectorParameters_create()
    
    def detect_camera_pose(self,undistorted_image):
        corners, ids, _ = cv2.aruco.detectMarkers(undistorted_image, self.aruco_dictionary, parameters=self.aruco_detector_parameters)
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs
            )
            T_world_camera_list = []
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id not in [20, 21, 22, 23]:
                    continue
                
                cv2.aruco.drawDetectedMarkers(undistorted_image, corners)
                cv2.aruco.drawAxis(
                    undistorted_image,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvecs[i],
                    tvecs[i],
                    self.aruco_marker_size / 2
                )

                cam_to_marker_transform = self.compute_transformation_matrix(rvecs[i], tvecs[i])
                
                if (self.debug_camera):
                    print(f"Marker ID {marker_id} - T_camera_marker:")
                    print(f"{cam_to_marker_transform}\n")

                world_to_camera_transform = self.compute_world_camera_pose(marker_id, cam_to_marker_transform)
                    

                T_world_camera_list.append((marker_id, world_to_camera_transform))
                if (self.debug_camera):    
                    print(f"Marker ID {marker_id} - T_world_camera:")
                    print(f"{world_to_camera_transform}\n")
                    print("-" * 50)
            
            if len(T_world_camera_list) > 0:
                if (self.debug_camera):
                    print(f"Computed {len(T_world_camera_list)} camera poses in world frame")
                T_sum = np.zeros((4, 4))
                for marker_id, T in T_world_camera_list:
                    T_sum += T
                self.final_world_to_cam_transform = T_sum / len(T_world_camera_list)
                if (self.debug_camera):
                    print(f"{self.final_world_to_cam_transform}\n")

    def detect_crates_poses(self, undistorted_image):
        corners, ids, _ = cv2.aruco.detectMarkers(
            undistorted_image, self.crate_aruco_dictionary, parameters=self.crate_detector_parameters
        )
        
        crate_poses = []  # List of (marker_id, T_world_crate) tuples
        
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.crate_aruco_size, self.camera_matrix, self.dist_coeffs
            )
            
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id in [20, 21, 22, 23]: # Exclude playing arena ArUco 
                    continue
                
                cv2.aruco.drawDetectedMarkers(undistorted_image, corners)
                cv2.aruco.drawAxis(
                    undistorted_image,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvecs[i],
                    tvecs[i],
                    self.crate_aruco_size / 2
                )
                x, y = int(corners[i][0][0][0]), int(corners[i][0][0][1])
                cv2.putText(undistorted_image, f"Iter: {i}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                T_camera_crate = self.compute_transformation_matrix(rvecs[i], tvecs[i])
                
                T_world_crate = self.final_world_to_cam_transform @ T_camera_crate
                
                crate_poses.append((marker_id, T_world_crate))
                
                if self.debug_crates:
                    print(f"Crate ID {marker_id} - T_world_crate{i}:")
                    print(f"{T_world_crate}\n")
        
        return crate_poses      
          
    def compute_transformation_matrix(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        t = tvec.reshape((3,))
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    def compute_world_camera_pose(self, marker_id, cam_to_marker_transform):
        if marker_id not in self.world_marker_coordinates:
            return None

        # Get marker's world position
        p = self.world_marker_coordinates[marker_id]
        
        T_world_marker = np.eye(4)
        T_world_marker[0:3, 3] = p
        
        T_marker_camera = np.linalg.inv(cam_to_marker_transform)
        
        world_to_camera_transform = T_world_marker @ T_marker_camera
        
        return world_to_camera_transform
    
    def set_camera_info(self, msg):
        if(self.fx is None): # Initialize camera parameters once
            self.fx = msg.p[0]
            self.fy = msg.p[5]
            self.cx = msg.p[2]
            self.cy = msg.p[6]
            self.camera_matrix = np.array([[self.fx,     0,      self.cx],
                                        [  0,         self.fy,    self.cy],
                                        [  0,            0,        1    ]])
            self.dist_coeffs = np.array(msg.d)
            print("K:", self.camera_matrix)
            print("D:", self.dist_coeffs)

    def image_callback(self, msg):
        if(self.fx is not None):
            
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            undistorted_image = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            
            
            
            if(self.final_world_to_cam_transform is None): 
                self.detect_camera_pose(undistorted_image)
            else:
                self.detect_crates_poses(undistorted_image)
          
            cv2.imshow("Camera Feed", undistorted_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main():
    rclpy.init()
    camera_viewer_node = CameraViewer()
    rclpy.spin(camera_viewer_node)


if __name__ == '__main__':
    main()
