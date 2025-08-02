
#!/usr/bin/env python3

import moveit_msgs.msg
import geometry_msgs.msg
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import json
import os
from tf2_geometry_msgs import do_transform_point
import tf2_geometry_msgs
import tf2_ros
from sklearn.linear_model import LinearRegression
from scipy.spatial.distance import cdist
import logging
import moveit_commander 
from tf.transformations import quaternion_from_euler
import sys

class Detector:
    def __init__(self):
        self.model = YOLO('trained_yolov8n.pt')
        rospy.sleep(1)
        os.environ['YOLO_LOG_LEVEL'] = 'error'
        logging.getLogger("ultralytics").setLevel(logging.ERROR)

        self.camera_input_left = rospy.get_param('~input_topic_left', '/camera_gripper_left/image_raw')
        self.camera_info_left = rospy.get_param('~camera_info_topic', '/camera_gripper_left/camera_info')
        self.camera_input_right = rospy.get_param('~input_topic_right', '/camera_gripper_right/image_raw')
        self.camera_info_right = rospy.get_param('~camera_info_topic', '/camera_gripper_right/camera_info')
        
        self.camera_frame_left = rospy.get_param('~camera_frame_left', 'camera_link_gripper_left')
        self.camera_frame_right = rospy.get_param('~camera_frame_right', 'camera_link_gripper_right')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        self.subscriber_left = rospy.Subscriber(self.camera_input_left, Image, self.image_callback_left)
        self.subscriber_right = rospy.Subscriber(self.camera_input_right, Image, self.image_callback_right)
        self.camera_info_sub_left = rospy.Subscriber(self.camera_info_left, CameraInfo, self.camera_info_callback_left)
        self.camera_info_sub_right = rospy.Subscriber(self.camera_info_right, CameraInfo, self.camera_info_callback_right)
        
        # self.camera_frame = rospy.get_param('~camera_frame', 'camera_link')
        self.bridge = CvBridge()
        # self.subscriber = rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        # self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        self.publisher_left = rospy.Publisher('yoloL', Image, queue_size=10)
        self.publisher_right = rospy.Publisher('yoloR', Image, queue_size=10)
        self.publisher_depth = rospy.Publisher('stereo', Image, queue_size=10)
        self.class_names = [
            'accent', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'minus', 'plus', 'del', 'tab', 'q', 'w', 'e', 'r', 't', 
            'y', 'u', 'i', 'o', 'p', '[', ']', 'enter', 'caps', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ':', 
            '"', '\\', 'shift-left', 'less', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', 'shift-right', 
            'ctrl-left', 'alt-left', 'space', 'alt-right', 'ctrl-right', 'keyboard'
        ]
        self.keyboard_layout = {
            'row1': ['1', '2', '3', '4', '5', '6', '7', '8', '9', '0'],
            'row2': ['Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P'],
            'row3': ['A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L'],
            'row4': ['Z', 'X', 'C', 'V', 'B', 'N', 'M']
        }
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.img_left = None
        self.img_right = None
        self.dist_coeffs_right = None
        self.dist_coeffs_left = None
        self.camera_matrix_left = None
        self.camera_matrix_right = None
        self.resultL = None
        self.resultR = None
        self.points_left = None
        self.points_right = None
        
        
    def camera_info_callback_left(self, msg):
        try:
            self.camera_matrix_left = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs_left = np.array(msg.D)
        except Exception as e:
            rospy.logerr(f"Dum as fk cam_info_left : {e}")
        
    def camera_info_callback_right(self, msg):
        try:
            self.camera_matrix_right = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs_right = np.array(msg.D)
        except Exception as e:
            rospy.logerr(f"Dum as fk cam_info_right : {e}")
        
    def image_callback_left(self,msg):
        try:
            self.img_left, self.resultL = self.image_callback(msg)
            self.publisher_left.publish(self.bridge.cv2_to_imgmsg(self.img_left, encoding='bgr8'))
        except Exception as e:
            rospy.logerr(f"Dum as fk img_left : {e}")
        
    def image_callback_right(self,msg):
        try:
            self.img_right, self.resultR = self.image_callback(msg)
            self.publisher_right.publish(self.bridge.cv2_to_imgmsg(self.img_right, encoding='bgr8'))
        except Exception as e:
            rospy.logerr(f"Dum as fk img_right : {e}")
        
    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # img_copy = img.copy()
        img = cv2.resize(img, (640, 640))  # Adjust size as needed
        results = self.model(img,iou=0.7, conf=0.5)
        results = results[0]
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                label = self.class_names[class_id]
                confidence = box.conf[0]

                # Draw bounding box
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Draw label and confidence
                # cv2.putText(img, f'{label} {confidence:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(img, f'{label}', (int((x1+x2)/2),int((y1+y2)/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
                # Publish the image with bounding boxes
        return img,results
    
    def _get_points(self, results):
        points = {key: None for key in self.class_names}  # Initialize all keys with None
        confidences = {key: 0 for key in self.class_names}  # Track max confidence per key

        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                label = self.class_names[class_id]
                confidence = float(box.conf[0])  # Convert to float for comparison

                # Update only if confidence is higher
                if points[label] is None or confidence >= confidences[label]:
                    points[label] = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                    confidences[label] = confidence  # Store max confidence for this key
                    
        return points
    
    def _get_tf_stereo(self):
        transform = self.tf_buffer.lookup_transform(self.camera_frame_right, self.camera_frame_left, rospy.Time(0), rospy.Duration(3.0))
        if transform is None:
            return None, None
        # Extract translation and rotation from the transform
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        T = np.array([translation.x, translation.y, translation.z])
        quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        R = self._quaternion_to_rotation_matrix(quaternion)      
        return R,T
    
    def _quaternion_to_rotation_matrix(self,quat):
        """
        Convert a quaternion (x, y, z, w) to a 3x3 rotation matrix.
        """
        x, y, z, w = quat
        R = np.zeros((3, 3))

        # Calculate the rotation matrix
        R[0, 0] = 1 - 2 * (y**2 + z**2)
        R[0, 1] = 2 * (x * y - z * w)
        R[0, 2] = 2 * (x * z + y * w)
        R[1, 0] = 2 * (x * y + z * w)
        R[1, 1] = 1 - 2 * (x**2 + z**2)
        R[1, 2] = 2 * (y * z - x * w)
        R[2, 0] = 2 * (x * z - y * w)
        R[2, 1] = 2 * (y * z + x * w)
        R[2, 2] = 1 - 2 * (x**2 + y**2)

        return R
    
    
    
    # def depth_of_points(self):
        
    #     if self.img_left is None:
    #         rospy.logerr("Left image not received")
    #         return
        
    #     imgL = cv2.cvtColor(self.img_left, cv2.COLOR_BGR2GRAY)
    #     img_sizeL = (imgL.shape[1], imgL.shape[0])
        
    #     R,T = self._get_tf_stereo()
    #     if R is None or T is None:
    #         rospy.logerr("Failed to get transform")
    #         return 
        
    #     Q = cv2.stereoRectify(self.camera_matrix_left, self.dist_coeffs_left,
    #                                                 self.camera_matrix_right, self.dist_coeffs_right,
    #                                                 img_sizeL, R, T)\
    #         [4] # Q matrix at 4th index
        
    #     points_left = np.array(self._get_points(self.resultL), dtype=np.float32)
    #     points_right = np.array(self._get_points(self.resultR), dtype=np.float32)

    #     # Compute disparity (difference in x-coordinates)
    #     disparities = (points_left[:, 0] - points_right[:, 0]).reshape(-1, 1)

    #     # Stack x, y, disparity into a single Nx3 array
    #     points_3D_hom = cv2.perspectiveTransform(
    #         np.hstack((points_left, disparities)).reshape(-1, 1, 3), Q
    #     )

    #     # Convert from homogeneous coordinates to (X, Y, Z)
    #     points_3D = points_3D_hom[:, 0, :3]  # Drop homogeneous coordinate
        
    def depth_of_points(self):
        if self.img_left is None:
            rospy.logerr("Left image not received")
            return
        
        imgL = cv2.cvtColor(self.img_left, cv2.COLOR_BGR2GRAY)
        img_sizeL = (imgL.shape[1], imgL.shape[0])
        
        R, T = self._get_tf_stereo()
        if R is None or T is None:
            rospy.logerr("Failed to get transform")
            return 

        Q = cv2.stereoRectify(
            self.camera_matrix_left, self.dist_coeffs_left,
            self.camera_matrix_right, self.dist_coeffs_right,
            img_sizeL, R, T
        )[4]  # Q matrix at 4th index
        
        points_left_dict = self._get_points(self.resultL)  # {label: (x, y) or None}
        points_right_dict = self._get_points(self.resultR)

        points_3D_dict = {}  # Store final 3D points as {label: (X, Y, Z)}

        for label in points_left_dict.keys():
            if points_left_dict[label] is None or points_right_dict[label] is None:
                points_3D_dict[label] = None
                continue  # Skip if either left or right point is missing
                
            xL, yL = points_left_dict[label]
            xR, _ = points_right_dict[label]  # Only xR is needed for disparity
            
            
            disparity = np.array([[xL - xR]], dtype=np.float32)  # Shape (1,1)

            # Convert (x, y, disparity) to (X, Y, Z)
            point_3D_hom = cv2.perspectiveTransform(
                np.array([[[xL, yL, disparity[0][0]]]], dtype=np.float32), Q
            )

            X, Y, Z = point_3D_hom[0, 0, :3]  # Extract (X, Y, Z)

            points_3D_dict[label] = (X, Y, Z)


        # TODO : Enable this after testing
        # points_3D_dict = self._corrected_points(points_3D_dict)

        return points_3D_dict  # Returns a dictionary of {label: (X, Y, Z)}
    
    def _corrected_points(self, points):
        """
        Corrects detected keyboard key positions maintaining the points dictionary structure.
        Ensures all points lie in the same plane before correction.
        Args:
            points: Dictionary mapping key names to (x, y, z) coordinates or None
        Returns:
            Dictionary with corrected key positions
        """
        # Filter out None values and create working copy
        valid_points = {k: np.array(v) for k, v in points.items() if v is not None and k.lower() in [
            item.lower() for sublist in self.keyboard_layout.values() for item in sublist
        ]}

        if len(valid_points) < 4:  # Need minimum points for meaningful correction
            return points

        # Get 3D coordinates and project them to the same plane
        coords = np.array(list(valid_points.values()))
        keys = list(valid_points.keys())

        try:
            projected_coords, normal = self._get_plane(coords)

            # Find keyboard orientation in 2D (X, Y plane)
            reg = LinearRegression()
            X = projected_coords[:, 0].reshape(-1, 1)
            y = projected_coords[:, 1]
            reg.fit(X, y)
            angle = np.arctan(reg.coef_[0])

            # Create a 3D rotation matrix around the normal vector
            rotation_matrix_3D = np.array([
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1]  # Keep the Z-axis unchanged
            ])

            # Apply rotation to align the keyboard
            rotated_coords = np.dot(projected_coords, rotation_matrix_3D)

            # Group into rows based on y-coordinates
            y_coords = rotated_coords[:, 1]
            sorted_y_indices = np.argsort(y_coords)
            row_clusters = []
            current_cluster = [sorted_y_indices[0]]

            # Adaptive row clustering based on Y spacing
            y_threshold = np.std(y_coords) * 0.5
            for i in range(1, len(sorted_y_indices)):
                if abs(y_coords[sorted_y_indices[i]] - y_coords[sorted_y_indices[i-1]]) < y_threshold:
                    current_cluster.append(sorted_y_indices[i])
                else:
                    row_clusters.append(current_cluster)
                    current_cluster = [sorted_y_indices[i]]
            row_clusters.append(current_cluster)

            # Process each row
            corrected_points = points.copy()
            for row_cluster in row_clusters:
                row_keys = [keys[i] for i in row_cluster]

                # Find matching keyboard row
                matched_row = None
                max_overlap = 0
                for row_name, expected_row in self.keyboard_layout.items():
                    overlap = len(set(k.upper() for k in row_keys) & set(expected_row))
                    if overlap > max_overlap:
                        max_overlap = overlap
                        matched_row = expected_row

                if matched_row and max_overlap >= 3:  # Require at least 3 matching keys
                    row_x_coords = rotated_coords[row_cluster][:, 0]
                    sorted_x_indices = np.argsort(row_x_coords)

                    # Calculate average spacing
                    avg_spacing = np.mean(np.diff(row_x_coords[sorted_x_indices]))

                    # Correct positions for detected keys
                    for i, key in enumerate(matched_row):
                        key_lower = key.lower()
                        matching_keys = [k for k in row_keys if k.lower() == key_lower]

                        if matching_keys:
                            idx = row_keys.index(matching_keys[0])
                            base_point = coords[row_cluster[idx]]
                        else:
                            left_keys = [k for k in row_keys if k.lower() in [x.lower() for x in matched_row[:i]]]
                            right_keys = [k for k in row_keys if k.lower() in [x.lower() for x in matched_row[i+1:]]]

                            if left_keys and right_keys:
                                left_idx = row_keys.index(left_keys[-1])
                                right_idx = row_keys.index(right_keys[0])
                                left_point = coords[row_cluster[left_idx]]
                                right_point = coords[row_cluster[right_idx]]
                                ratio = (i - matched_row.index(left_keys[-1].upper())) / \
                                        (matched_row.index(right_keys[0].upper()) - matched_row.index(left_keys[-1].upper()))
                                base_point = left_point + ratio * (right_point - left_point)
                            elif left_keys:
                                left_idx = row_keys.index(left_keys[-1])
                                base_point = coords[row_cluster[left_idx]] + np.array([avg_spacing, 0, 0])
                            elif right_keys:
                                right_idx = row_keys.index(right_keys[0])
                                base_point = coords[row_cluster[right_idx]] - np.array([avg_spacing, 0, 0])
                            else:
                                continue

                        # Update position in corrected_points if key exists in class_names
                        key_variations = [key_lower, key.upper()]
                        for key_var in key_variations:
                            if key_var in self.class_names:
                                corrected_points[key_var] = tuple(map(int, base_point))
                                break

            return corrected_points

        except Exception as e:
            rospy.logerr(f"Error in _corrected_points: {e}")
            return points  # Return original points if correction fails
    def _get_plane(self, points):
        """
        Fits a plane to the given 3D points using least squares.
        Projects all points onto the plane to ensure alignment.
        Args:
            points: Nx3 numpy array of 3D points
        Returns:
            projected_points: Nx3 numpy array of points projected onto the plane
            normal: Normal vector of the fitted plane
        """
        # Extract X, Y, Z coordinates
        X, Y, Z = points[:, 0], points[:, 1], points[:, 2]
        
        # Fit a plane: Z = aX + bY + c
        A = np.c_[X, Y, np.ones(len(points))]
        coeffs, _, _, _ = np.linalg.lstsq(A, Z, rcond=None)  # Solve least squares
        a, b, c = coeffs

        # Normal vector of the plane
        normal = np.array([a, b, -1])
        normal /= np.linalg.norm(normal)  # Normalize

        # Project points onto the plane
        projected_points = points - (np.dot(points, normal)[:, None] * normal)

        return projected_points, normal
    
    
    def depth_map(self):
        # Extract left and right images
        if self.img_left is None or self.img_right is None:
            rospy.logerr("Images not received")
            return
        
        imgL = cv2.cvtColor(self.img_left, cv2.COLOR_BGR2GRAY)
        imgL = cv2.undistort(imgL, self.camera_matrix_left, self.dist_coeffs_left)
        img_sizeL = (imgL.shape[1], imgL.shape[0])
        
        imgR = cv2.cvtColor(self.img_right, cv2.COLOR_BGR2GRAY)
        imgR = cv2.undistort(imgR, self.camera_matrix_right, self.dist_coeffs_right)
        img_sizeR = (imgR.shape[1], imgR.shape[0])
        
        if img_sizeL != img_sizeR:
            rospy.logerr("Image sizes do not match")
            return
        
        R,T = self._get_tf_stereo()
        if R is None or T is None:
            rospy.logerr("Failed to get transform")
            return 
        
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(self.camera_matrix_left, self.dist_coeffs_left,
                                                    self.camera_matrix_right, self.dist_coeffs_right,
                                                    img_sizeL, R, T)
        
        # Rectification maps
        map1x, map1y = cv2.initUndistortRectifyMap(self.camera_matrix_left, self.dist_coeffs_left, R1, P1, img_sizeL, cv2.CV_32FC1)
        map2x, map2y = cv2.initUndistortRectifyMap(self.camera_matrix_right, self.dist_coeffs_right, R2, P2, img_sizeR, cv2.CV_32FC1)

        rectified_left = cv2.remap(imgL, map1x, map1y, cv2.INTER_LINEAR)
        rectified_right = cv2.remap(imgR, map2x, map2y, cv2.INTER_LINEAR)

        stereo = cv2.StereoBM_create(numDisparities=16, blockSize=5)
        
        stereo.setPreFilterCap(31)  # Reduces noise
        stereo.setMinDisparity(5)   # Default minimum disparity
        stereo.setSpeckleRange(8)  # Removes speckle noise
        stereo.setSpeckleWindowSize(50)  # Adjusts speckle filtering

        
        disparity = stereo.compute(rectified_left, rectified_right)
        disparity = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        disparity = np.uint8(disparity)
        
        # Publish the disparity image
        self.publisher_depth.publish(self.bridge.cv2_to_imgmsg(disparity, encoding='mono8'))
        
    def display_points_3D(self,points_3D_dict):
        print(f"{'Label':<10} {'X':<10} {'Y':<10} {'Z':<10}")
        print("=" * 40)
        for label, value in points_3D_dict.items():
            if value is None:
                print(f"{label:<10} {'None':<10} {'None':<10} {'None':<10}")
                continue
            print(f"{label:<10} {value[0]:<10.3f} {value[1]:<10.3f} {value[2]:<10.3f}")
          
    def get_transform(self, point):
        """Transform a point from the camera frame to the world frame."""
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        try:
            camera_point = PointStamped()
            camera_point.header.frame_id = self.camera_frame_left
            camera_point.point.x = point[2]
            camera_point.point.y = -point[0]
            camera_point.point.z = -point[1]
            # Wait for the transform to be available
            transform = tf_buffer.lookup_transform("world", self.camera_frame_left, rospy.Time(0), rospy.Duration(1.0))
            # Transform the point
            world_point = do_transform_point(camera_point, transform)
            return [world_point.point.x,world_point.point.y,world_point.point.z]
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
            return None      
      
    def control_flow(self):
        pass
    
    
        
def main():
    rospy.init_node('yolo_detector', anonymous=True)
    detector = Detector()
    
    while not rospy.is_shutdown():
        if detector.img_left is None or detector.img_right is None:
            rospy.loginfo("Waiting for images...")
            rospy.sleep(2)
            continue
        
        rospy.loginfo("Images received")
        pt = detector.depth_of_points()
        detector.display_points_3D(pt)
        
        rospy.sleep(0.1)
        
    rospy.spin()
    
if __name__ == '__main__':
    main()
        
        
