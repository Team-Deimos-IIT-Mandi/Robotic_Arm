import rospy
import logging
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point
import tf2_geometry_msgs
import tf2_ros
import cv2
import numpy as np
from ultralytics import YOLO
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import json
import os
from geometry_msgs.msg import PointStamped

class AutonomousTyping:
    def __init__(self, debug=False):
        self.debug = debug
        if self.debug:
            rospy.init_node("autonomous_typing", log_level=rospy.DEBUG)
            logging.basicConfig(level=logging.DEBUG)
        else:
            rospy.init_node("autonomous_typing", log_level=rospy.INFO)
            logging.basicConfig(level=logging.INFO)

        rospy.loginfo("Initializing AutonomousTyping")

        # Initialize variables
        self.KEYBOARD_LENGTH = 354.076
        self.KEYBOARD_WIDTH = 123.444
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_path = os.path.join(current_dir, 'keyboard_layout.json')
        with open(json_path, 'r') as f:
            rospy.loginfo(f"Loading keyboard points from {json_path}")
            self.keyboard_points = json.load(f)
            
        self.class_to_detect = 66
        self.camera_input_left = rospy.get_param('~input_topic_left', '/camera_gripper_left/image_raw')
        self.camera_info_left = rospy.get_param('~camera_info_topic', '/camera_gripper_left/camera_info')
        self.camera_input_right = rospy.get_param('~input_topic_right', '/camera_gripper_right/image_raw')
        self.camera_info_right = rospy.get_param('~camera_info_topic', '/camera_gripper_right/camera_info')
        self.camera_frame_left = rospy.get_param('~camera_frame_left', 'camera_link_gripper_left')
        self.camera_frame_right = rospy.get_param('~camera_frame_right', 'camera_link_gripper_right')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        rospy.logdebug("Loading YOLO model")
        self.model = YOLO('yolov8n-seg.pt')
        rospy.sleep(1)
        os.environ['YOLO_LOG_LEVEL'] = 'error'
        logging.getLogger("ultralytics").setLevel(logging.ERROR)
        
        # Subscribers
        self.bridge = CvBridge()
        self.subscriber_left = rospy.Subscriber(self.camera_input_left, Image, self.image_callback_left)
        self.subscriber_right = rospy.Subscriber(self.camera_input_right, Image, self.image_callback_right)
        self.camera_info_sub_left = rospy.Subscriber(self.camera_info_left, CameraInfo, self.camera_info_callback_left)
        self.camera_info_sub_right = rospy.Subscriber(self.camera_info_right, CameraInfo, self.camera_info_callback_right)
        
        # Publishers
        self.image_pub_l = rospy.Publisher("/output_image_l", Image, queue_size=10)
        self.image_pub_r = rospy.Publisher("/output_image_r", Image, queue_size=10)
        
        # Initialize variables
        self.left_img = None
        self.right_img = None
        self.left_scan = None
        self.right_scan = None
        self.camera_info_left = None
        self.camera_info_right = None
        self.result_left = None
        self.result_right = None
        self.keypoints_left = None
        self.keypoints_right = None
        self.keyboard_points_3d = None
        self.camera_matrix_left = None
        self.camera_matrix_right = None
        self.dist_coeffs_left = None
        self.dist_coeffs_right = None
        self.baseline = 0.1
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("body")
        rospy.loginfo("self initialized")
                
    def image_callback_left(self, msg):
        rospy.logdebug("Left image callback triggered")
        try:
            self.left_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.display_output_l()
        except Exception as e:
            rospy.logerr(f"Error in left image callback: {e}. Message type: {type(msg)}")


    def image_callback_right(self, msg):
        rospy.logdebug("Right image callback triggered")
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.display_output_r()
        except Exception as e:
            rospy.logerr(f"Error in left image callback: {e}. Message type: {type(msg)}")

    def camera_info_callback_left(self, msg):
        rospy.logdebug("Left camera info callback triggered")
        try:
            self.camera_info_left = msg
            self.camera_matrix_left = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs_left = np.array(msg.D)
        except Exception as e:
            rospy.logerr(f"Error in left camera info callback: {e}. Message type: {type(msg)}")

    def camera_info_callback_right(self, msg):
        rospy.logdebug("Right camera info callback triggered")
        try:
            self.camera_info_right = msg
            self.camera_matrix_right = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs_right = np.array(msg.D)
        except Exception as e:
            rospy.logerr(f"Error in right camera info callback: {e}. Message type: {type(msg)}")
            
        
    def processing_img(self,img):
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
            
            
    def get_oriented_box(mask):
        """
        Calculate the oriented bounding box from a segmentation mask.
        """
        # Convert the mask to a binary image
        mask = (mask * 255).astype(np.uint8)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Get the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            # Fit a rotated rectangle (oriented bounding box)
            rect = cv2.minAreaRect(largest_contour)
            box = cv2.boxPoints(rect)
            box = box.astype(np.int32)
            # box = np.int0(box)  # Convert to integer
            return box, rect
        else:
            return None, None
        
    def display_output_l(self):
        try:
            rospy.logdebug("Displaying left output")
            img = self.left_img.copy()
            bbox, keyboard_points = self.tracking_keyboard(self.result_left,{},self.camera_frame_left,self.camera_matrix_left,self.dist_coeffs_left)
            if bbox is not None:
                for i in range(4):
                    cv2.line(img, tuple(bbox[i]), tuple(bbox[(i+1)%4]), (0, 255, 0), 2)
            if keyboard_points is not None:
                for key, point in keyboard_points.items():
                    if point is None:
                        continue
                    cv2.circle(img, tuple(point), 3, (255, 0, 0), -1)
                    cv2.circle(img, tuple(point), 5, (0, 0, 255), -1)
                    cv2.putText(img, key, tuple(point), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            self.image_pub_l.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
            rospy.logdebug("Published annotated left image")
        except Exception as e:
            rospy.logerr(f"Error displaying left output: {e}. Message type: {type(e)}")
        
    def display_output_r(self):
        try:
            rospy.logdebug("Displaying right output")
            img = self.right_img.copy()
            bbox, keyboard_points = self.tracking_keyboard(self.result_right,{},self.camera_frame_right,self.camera_matrix_right,self.dist_coeffs_right)
            if bbox is not None:
                for i in range(4):
                    cv2.line(img, tuple(bbox[i]), tuple(bbox[(i+1)%4]), (0, 255, 0), 2)
            if keyboard_points is not None:
                for key, point in keyboard_points.items():
                    if point is None:
                        continue
                    cv2.circle(img, tuple(point), 3, (255, 0, 0), -1)
                    cv2.circle(img, tuple(point), 5, (0, 0, 255), -1)
                    cv2.putText(img, key, tuple(point), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            self.image_pub_r.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
            rospy.logdebug("Published annotated right image")
        except Exception as e:
            rospy.logerr(f"Error displaying right output: {e}. Message type: {type(e)}")
        
    def tracking_keyboard(self, bbox, keyboard_keys, camera_frame, camera_matrix, dist_coeffs):
        """
        Track keyboard in camera frame and project keyboard points
        
        Args:
            bbox: PoseStamped of keyboard bounding box in world frame
            keyboard_keys: Dictionary of keyboard key positions in world frame
            camera_frame: Current camera frame ID
            camera_matrix: Camera intrinsic matrix
            dist_coeffs: Distortion coefficients
            
        Returns:
            corners_2d: 2D coordinates of keyboard corners in image
            key_points_2d: Dictionary of 2D coordinates for keyboard keys
        """
        rospy.logdebug("Tracking keyboard")
        try:
            if bbox is None:
                rospy.logdebug("No keyboard detected")
                return None, None
            # Get transform from world to camera frame
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                'world',
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            # Transform keyboard bbox from world to camera frame
            bbox_transformed = tf2_geometry_msgs.do_transform_pose(
                bbox,
                transform
            )
            
            
            # Generate 3D corners of keyboard
            l, w = self.KEYBOARD_LENGTH/2, self.KEYBOARD_WIDTH/2
            corners_3d = np.float32([
                [-l, -w, 0],
                [ l, -w, 0],
                [ l,  w, 0],
                [-l,  w, 0]
            ])
            
            # Transform corners to camera frame
            corners_3d = self._transform_points(corners_3d, bbox_transformed)
            
            # Project 3D corners to 2D image plane
            corners_2d, _ = cv2.projectPoints(
                corners_3d, 
                np.zeros((3, 1)), 
                np.zeros((3, 1)), 
                camera_matrix, 
                dist_coeffs
            )
            if corners_2d is not None:
                corners_2d = corners_2d.reshape(-1, 2)
            else:
                rospy.logwarn("corners_2d is None, cannot reshape.")
                return None, None
            
            # Get image dimensions
            image_width = self.camera_info_left.width if camera_frame == self.camera_frame_left else self.camera_info_right.width
            if camera_frame == self.camera_frame_left and self.camera_info_left is not None:
                image_height = self.camera_info_left.height
            elif camera_frame == self.camera_frame_right and self.camera_info_right is not None:
                image_height = self.camera_info_right.height
            else:
                rospy.logwarn("Camera info is None for frame: {}".format(camera_frame))
                image_height = 0  # or set a default value as needed
            
            # Check if corners are within image bounds
            if np.any(corners_2d[:, 0] < 0) or np.any(corners_2d[:, 0] > image_width) or \
            np.any(corners_2d[:, 1] < 0) or np.any(corners_2d[:, 1] > image_height):
                rospy.logwarn("Corners exceed camera frame boundaries.")
                corners_2d[:, 0] = np.clip(corners_2d[:, 0], 0, image_width - 1)
                corners_2d[:, 1] = np.clip(corners_2d[:, 1], 0, image_height - 1)
            
            if keyboard_keys is None:
                rospy.logwarn("keyboard_keys is None, cannot project keyboard keys.")
                return corners_2d, None
            
            # Transform keyboard key positions
            keyboard_keys_transformed = {}
            for key, pose in keyboard_keys.items():
                key_pose_transformed = tf2_geometry_msgs.do_transform_pose(
                    pose,
                    transform
                )
                keyboard_keys_transformed[key] = key_pose_transformed
            # Project keyboard key positions to 2D
            key_points_2d = {}
            for key, pose in keyboard_keys_transformed.items():
                point_3d = np.float32([[pose.pose.position.x, 
                                    pose.pose.position.y, 
                                    pose.pose.position.z]])
                point_2d, _ = cv2.projectPoints(
                    point_3d, 
                    np.zeros((3, 1)), 
                    np.zeros((3, 1)), 
                    camera_matrix, 
                    dist_coeffs
                )
                key_points_2d[key] = point_2d.reshape(2)
                
                # Clip key points to image boundaries
                if key_points_2d[key][1] < 0 or key_points_2d[key][1] > image_height:
                    key_points_2d[key] = None
                if key_points_2d[key][0] < 0 or key_points_2d[key][0] > image_width:
                    key_points_2d[key] = None
                    
            return corners_2d, key_points_2d
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform lookup failed: {e}")
            return None, None

    def _transform_points(self, points, pose):
        """
        Transform points using pose information
        
        Args:
            points: Nx3 array of 3D points
            pose: PoseStamped message containing transform
            
        Returns:
            transformed_points: Nx3 array of transformed points
        """
        # Extract translation
        t = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        ])
        
        # Extract rotation from quaternion
        q = pose.pose.orientation
        R = self._quaternion_to_rotation_matrix(q)
        
        # Transform points
        transformed_points = np.dot(points, R.T) + t
        return transformed_points

    def _quaternion_to_rotation_matrix(self, q):
        """
        Convert quaternion to rotation matrix
        
        Args:
            q: Quaternion (geometry_msgs/Quaternion)
            
        Returns:
            R: 3x3 rotation matrix
        """
        # Convert to numpy array and normalize
        q_array = np.array([q.x, q.y, q.z, q.w])
        q_array = q_array / np.linalg.norm(q_array)
        
        # Compute rotation matrix
        R = np.array([
            [1 - 2*q_array[1]**2 - 2*q_array[2]**2,
            2*q_array[0]*q_array[1] - 2*q_array[2]*q_array[3],
            2*q_array[0]*q_array[2] + 2*q_array[1]*q_array[3]],
            [2*q_array[0]*q_array[1] + 2*q_array[2]*q_array[3],
            1 - 2*q_array[0]**2 - 2*q_array[2]**2,
            2*q_array[1]*q_array[2] - 2*q_array[0]*q_array[3]],
            [2*q_array[0]*q_array[2] - 2*q_array[1]*q_array[3],
            2*q_array[1]*q_array[2] + 2*q_array[0]*q_array[3],
            1 - 2*q_array[0]**2 - 2*q_array[1]**2]
        ])
        return R
    def _transform_points(self, points, pose):
        """Transform points using pose"""
        t = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        ])
        
        q = pose.pose.orientation
        R = self._quaternion_to_rotation_matrix(q)
        
        transformed_points = np.dot(points, R.T) + t
        return transformed_points
    
    def scanning_for_keyboard(self):
        # while self.result_left is None or self.result_right is None:
            rospy.loginfo("Keyboard detected")
            rospy.loginfo("Starting scanning for keyboard")
            self.result_left = self.processing_img(self.left_img)
            self.result_right = self.processing_img(self.right_img)
            rospy.loginfo("Keyboard scanned")
        
    
        
            
def main():
    at = AutonomousTyping(debug = True)
    rospy.loginfo("Autonomous Typing node started")
    rospy.loginfo("Waiting for camera info...")
    while not rospy.is_shutdown() and (at.camera_info_left is None or at.camera_info_right is None):
        rospy.sleep(1)
    rospy.loginfo("Camera info received. Starting main loop.")
    # input(">>> Press Enter to start scanning for keyboard...")
    start_time = None
    while not rospy.is_shutdown():
        at.scanning_for_keyboard()
        if at.result_left is not None and at.result_right is not None:
            if start_time is None:
                start_time = rospy.get_time()
            elif rospy.get_time() - start_time >= 5:
                rospy.logdebug("Images available for 5 seconds. Exiting loop.")
                break
        else:
            start_time = None
        rospy.sleep(0.1)
        
    rospy.loginfo("Scanning for keyboard completed.")
    
        
    rospy.spin()
    
if __name__ == "__main__":
    main()