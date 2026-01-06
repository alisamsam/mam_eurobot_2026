#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

class GlobalEagleEye(Node):
    def __init__(self):
        super().__init__('global_eagle_eye')
        
        # --- 1. Configuration Caméra & Topics ---
        self.camera_topic = '/left_cam/image_raw'
        self.camera_info = '/left_cam/camera_info'
        
        # --- 2. Configuration ArUco (Paramètres physiques) ---
        self.aruco_marker_size = 0.1        # Taille des tags de référence (sol)
        self.crate_aruco_size = 0.0375      # Taille des tags sur les caisses
        self.world_marker_ids = [20, 21, 22, 23] # IDs fixes au sol
        
        # Coordonnées des tags de référence (Source: Perception Stack)
        self.world_marker_coordinates = {
            20: [0.6, 1.4, 0],
            21: [2.4, 1.4, 0], 
            22: [0.6, 0.6, 0],
            23: [2.4, 0.6, 0]
        }

        # --- 3. Initialisation Pose Caméra (Valeur par défaut) ---
        # On garde une valeur initiale au cas où les tags ref ne sont pas visibles
        self.T_world_cam = np.eye(4)
        self.T_world_cam[:3, 3] = [1.95, -1.55, 1.0] 
        r = R.from_euler('xyz', [0, 30, 143], degrees=True) 
        self.T_world_cam[:3, :3] = r.as_matrix()
        self.is_camera_calibrated = False # Flag pour savoir si on utilise la pose dynamique

        # --- 4. Configuration Détecteur ArUco ---
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        # PARAMÈTRES CRITIQUES POUR "EAGLE EYE" (Vue de loin)
        self.aruco_params.minMarkerPerimeterRate = 0.005 # Accepte les très petits tags
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 30
        self.aruco_params.polygonalApproxAccuracyRate = 0.03

        # --- 5. Configuration Couleur (RAL) ---
        self.hsv_ranges = {
            'blue': (np.array([90, 70, 40]), np.array([135, 255, 255])),
            'yellow': (np.array([20, 100, 100]), np.array([35, 255, 255]))
        }

        # Subscribers & Publishers
        self.sub_img = self.create_subscription(Image, self.camera_topic, self.img_cb, 10)
        self.sub_info = self.create_subscription(CameraInfo, self.camera_info, self.info_cb, 10)
        self.pub_global = self.create_publisher(PoseArray, '/perception/global_crates', 10)
        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

    def info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.destroy_subscription(self.sub_info)

    def img_cb(self, msg):
        if self.camera_matrix is None: return
        
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        undistorted = cv2.undistort(img, self.camera_matrix, self.dist_coeffs)
        hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)
        
        debug_img = undistorted.copy()
        
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "world"

        # ====================================================
        # ÉTAPE 1 : AUTO-LOCALISATION CAMÉRA (Reference Tags)
        # ====================================================
        corners, ids, _ = cv2.aruco.detectMarkers(undistorted, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            # On cherche d'abord à mettre à jour la position de la caméra
            self.update_camera_pose(corners, ids, undistorted)
        
        # Affichage axes Repère Monde (si calibré)
        if self.is_camera_calibrated:
            cv2.putText(debug_img, "CAM CALIBRATED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(debug_img, "USING DEFAULT POSE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # ====================================================
        # ÉTAPE 2 : DÉTECTION CAISSES via ARUCO
        # ====================================================
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.crate_aruco_size, self.camera_matrix, self.dist_coeffs)
            
            for i, marker_id in enumerate(ids.flatten()):
                # Si c'est un tag de CAISSE (36, 47, 41)
                if marker_id not in self.world_marker_ids: 
                    # Transformation T_camera -> Crate
                    T_cam_crate = self.compute_transformation_matrix(rvecs[i], tvecs[i])
                    
                    # Transformation T_world -> Crate = T_world_cam * T_cam_crate
                    T_world_crate = self.T_world_cam @ T_cam_crate
                    
                    pose_msg = self.transformation_matrix_to_pose_msg(T_world_crate)
                    pose_array.poses.append(pose_msg)

                    # Debug Visuel
                    cv2.aruco.drawAxis(debug_img, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
                    cv2.putText(debug_img, f"ID {marker_id}", (int(corners[i][0][0][0]), int(corners[i][0][0][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # ====================================================
        # ÉTAPE 3 : DÉTECTION CAISSES via COULEUR (Fallback)
        # ====================================================
        for color, (low, high) in self.hsv_ranges.items():
            mask = cv2.inRange(hsv, low, high)
            kernel = np.ones((3,3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 300: # Seuil ajusté pour Eagle Eye
                    rect = cv2.minAreaRect(cnt)
                    (center, (w, h), angle) = rect
                    rect_area = w * h
                    if rect_area > 0:
                        extent = float(area) / rect_area
                        aspect_ratio = float(w) / h if h > 0 else 0
                        
                        if extent > 0.50 and 0.2 < aspect_ratio < 5.0:
                            u, v = int(center[0]), int(center[1])
                            world_pos = self.project_pixel_to_ground(u, v)
                            
                            if world_pos is not None:
                                p = Pose()
                                p.position.x = world_pos[0]
                                p.position.y = world_pos[1]
                                p.position.z = 0.15 # Hauteur fixe pour la couleur
                                pose_array.poses.append(p)
                                
                                # Debug Visuel Couleur
                                box = np.int0(cv2.boxPoints(rect))
                                color_rgb = (0, 255, 255) if color == 'yellow' else (255, 0, 0)
                                cv2.drawContours(debug_img, [box], 0, color_rgb, 2)

        self.pub_global.publish(pose_array)
        cv2.imshow("Global Eagle Eye (ArUco + Color)", debug_img)
        cv2.waitKey(1)

    # --- MÉTHODES MATHÉMATIQUES (Importées de votre perception stack) ---

    def update_camera_pose(self, corners, ids, img_debug):
        """ Calcule T_world_cam en moyennant les tags de référence visibles """
        T_world_camera_list = []
        
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs)
        
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in self.world_marker_ids:
                # 1. T_cam -> Marker
                T_cam_marker = self.compute_transformation_matrix(rvecs[i], tvecs[i])
                
                # 2. T_world -> Camera (via ce marker)
                T_world_cam_est = self.compute_world_camera_pose(marker_id, T_cam_marker)
                
                if T_world_cam_est is not None:
                    T_world_camera_list.append(T_world_cam_est)
                    # Debug: dessiner les tags de ref en bleu
                    cv2.aruco.drawAxis(img_debug, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)

        # Moyenne des positions si on a vu des tags
        if T_world_camera_list:
            T_sum = np.mean(np.array(T_world_camera_list), axis=0)
            self.T_world_cam = T_sum
            self.is_camera_calibrated = True

    def compute_world_camera_pose(self, marker_id, T_cam_marker):
        if marker_id not in self.world_marker_coordinates: return None
        
        # Position du marker dans le monde (T_world_marker)
        p = self.world_marker_coordinates[marker_id]
        T_world_marker = np.eye(4)
        T_world_marker[0:3, 3] = p
        
        # T_world_cam = T_world_marker * inv(T_cam_marker)
        # Explication : Camera -> Marker -> World  (inversé)
        T_marker_cam = np.linalg.inv(T_cam_marker)
        return T_world_marker @ T_marker_cam

    def project_pixel_to_ground(self, u, v):
        uv_homog = np.array([u, v, 1.0])
        ray_cam = np.linalg.inv(self.camera_matrix) @ uv_homog 
        R_wc = self.T_world_cam[:3, :3]
        t_wc = self.T_world_cam[:3, 3]
        ray_world = R_wc @ ray_cam
        if abs(ray_world[2]) < 1e-6: return None
        scale = -t_wc[2] / ray_world[2]
        if scale < 0: return None 
        return t_wc + scale * ray_world

    def compute_transformation_matrix(self, rvec, tvec):
        R_mat, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = tvec.flatten()
        return T
    
    def transformation_matrix_to_pose_msg(self, T):
        pose_msg = Pose()
        pose_msg.position.x = T[0, 3]
        pose_msg.position.y = T[1, 3]
        pose_msg.position.z = T[2, 3]
        q = self.rotation_matrix_to_quaternion(T[:3, :3])
        pose_msg.orientation.x = q[0]
        pose_msg.orientation.y = q[1]
        pose_msg.orientation.z = q[2]
        pose_msg.orientation.w = q[3]
        return pose_msg

    def rotation_matrix_to_quaternion(self, R):
        q = np.empty((4,), dtype=np.float64)
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            q[3] = 0.25 * S
            q[0] = (R[2, 1] - R[1, 2]) / S
            q[1] = (R[0, 2] - R[2, 0]) / S
            q[2] = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            q[3] = (R[2, 1] - R[1, 2]) / S
            q[0] = 0.25 * S
            q[1] = (R[0, 1] + R[1, 0]) / S
            q[2] = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            q[3] = (R[0, 2] - R[2, 0]) / S
            q[0] = (R[0, 1] + R[1, 0]) / S
            q[1] = 0.25 * S
            q[2] = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            q[3] = (R[1, 0] - R[0, 1]) / S
            q[0] = (R[0, 2] + R[2, 0]) / S
            q[1] = (R[1, 2] + R[2, 1]) / S
            q[2] = 0.25 * S
        return q

def main(args=None):
    rclpy.init(args=args)
    node = GlobalEagleEye()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()