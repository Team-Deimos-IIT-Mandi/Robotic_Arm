import rospy
import logging
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point
import tf2_ros
import cv2
import numpy as np
from ultralytics import YOLO
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import json
import os
from geometry_msgs.msg import PointStamped

class KeyboardTracker:
    def __init__(self):
        rospy.loginfo("Initializing KeyboardTracker")
        
        # Store the keyboard's pose in world frame
        self.keyboard_world_pose = None
        
        # Store keyboard dimensions (in meters)
        self.keyboard_dims = {
            'length': 0.354076,
            'width': 0.123444
        }
        
        # TF handling
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initialize camera matrices (to be set later)
        self.camera_matrix_left = None
        self.camera_matrix_right = None
        
    def set_initial_keyboard_pose(self, left_bbox, right_bbox, camera_info_left, camera_info_right):
        """Initialize the keyboard's pose in world coordinates using stereo vision"""
        world_pose = self._stereo_triangulate(left_bbox, right_bbox, 
                                            camera_info_left, camera_info_right)
        
        if world_pose is not None:
            self.keyboard_world_pose = world_pose
            rospy.loginfo("Initial keyboard pose set in world frame")
            return True
        return False
    
    def get_keyboard_camera_pose(self, camera_frame):
        """Get the keyboard's pose in the current camera frame"""
        if self.keyboard_world_pose is None:
            rospy.logwarn("Keyboard world pose not initialized")
            return None
            
        try:
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                'world',
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            keyboard_camera_pose = tf2_geometry_msgs.do_transform_pose(
                self.keyboard_world_pose,
                transform
            )
            
            return keyboard_camera_pose
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to transform keyboard pose: {e}")
            return None
    
    def project_keyboard_points(self, camera_matrix, camera_frame):
        """Project keyboard corners into the current camera view"""
        if self.keyboard_world_pose is None:
            return None
            
        keyboard_camera_pose = self.get_keyboard_camera_pose(camera_frame)
        if keyboard_camera_pose is None:
            return None
            
        l, w = self.keyboard_dims['length']/2, self.keyboard_dims['width']/2
        corners_3d = np.float32([
            [-l, -w, 0],
            [ l, -w, 0],
            [ l,  w, 0],
            [-l,  w, 0]
        ])
        
        corners_camera = self._transform_points(corners_3d, keyboard_camera_pose)
        
        corners_2d, _ = cv2.projectPoints(
            corners_camera,
            np.zeros(3),
            np.zeros(3),
            camera_matrix,
            None
        )
        
        return corners_2d.reshape(-1, 2)
    
    def _stereo_triangulate(self, left_bbox, right_bbox, camera_info_left, camera_info_right):
        """Triangulate the keyboard center point using stereo vision"""
        left_center = [(left_bbox[0] + left_bbox[2])/2, 
                      (left_bbox[1] + left_bbox[3])/2]
        right_center = [(right_bbox[0] + right_bbox[2])/2, 
                       (right_bbox[1] + right_bbox[3])/2]
        
        P1 = np.array(camera_info_left.P).reshape(3, 4)
        P2 = np.array(camera_info_right.P).reshape(3, 4)
        
        point_4d = cv2.triangulatePoints(P1, P2, 
                                       np.float32(left_center), 
                                       np.float32(right_center))
        
        point_3d = point_4d[:3] / point_4d[3]
        
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = point_3d[0]
        pose.pose.position.y = point_3d[1]
        pose.pose.position.z = point_3d[2]
        pose.pose.orientation.w = 1.0
        
        return pose
    
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
    
    def _quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix"""
        q = np.array([q.x, q.y, q.z, q.w])
        q = q / np.linalg.norm(q)
        
        R = np.array([
            [1 - 2*q[1]**2 - 2*q[2]**2,
             2*q[0]*q[1] - 2*q[2]*q[3],
             2*q[0]*q[2] + 2*q[1]*q[3]],
            [2*q[0]*q[1] + 2*q[2]*q[3],
             1 - 2*q[0]**2 - 2*q[2]**2,
             2*q[1]*q[2] - 2*q[0]*q[3]],
            [2*q[0]*q[2] - 2*q[1]*q[3],
             2*q[1]*q[2] + 2*q[0]*q[3],
             1 - 2*q[0]**2 - 2*q[1]**2]
        ])
        return R

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

        # Initialize keyboard tracker
        self.keyboard_tracker = KeyboardTracker()
        self.keyboard_initialized = False

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
        self.camera_info_left = None
        self.camera_info_right = None
        self.result_left = None
        self.result_right = None
        self.keyboard_points_left = None
        self.keyboard_points_right = None
        
        # Initialize MoveitCommander components
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("body")
        rospy.loginfo("AutonomousTyping initialized")

    def image_callback_left(self, msg):
        rospy.logdebug("Left image callback triggered")
        try:
            self.left_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # if self.keyboard_initialized:
            #     self.keyboard_points_left = self.keyboard_tracker.project_keyboard_points(
            #         self.camera_matrix_left,
            #         self.camera_frame_left
            #     )
            #     self.display_output_l()
        except Exception as e:
            rospy.logerr(f"Error in left image callback: {e}")

    def image_callback_right(self, msg):
        rospy.logdebug("Right image callback triggered")
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # if self.keyboard_initialized:
            #     self.keyboard_points_right = self.keyboard_tracker.project_keyboard_points(
            #         self.camera_matrix_right,
            #         self.camera_frame_right
            #     )
            #     self.display_output_r()
        except Exception as e:
            rospy.logerr(f"Error in right image callback: {e}")

    def camera_info_callback_left(self, msg):
        rospy.logdebug("Left camera info callback triggered")
        try:
            self.camera_info_left = msg
            self.camera_matrix_left = np.array(msg.K).reshape(3, 3)
            self.keyboard_tracker.camera_matrix_left = self.camera_matrix_left
        except Exception as e:
            rospy.logerr(f"Error in left camera info callback: {e}")

    def camera_info_callback_right(self, msg):
        rospy.logdebug("Right camera info callback triggered")
        try:
            self.camera_info_right = msg
            self.camera_matrix_right = np.array(msg.K).reshape(3, 3)
            self.keyboard_tracker.camera_matrix_right = self.camera_matrix_right
        except Exception as e:
            rospy.logerr(f"Error in right camera info callback: {e}")

    def processing_img(self, img):
        rospy.logdebug("Scanning for keyboard")
        try:
            results = self.model(img)[0]
            detected = results.boxes[results.boxes.cls == self.class_to_detect]
            if len(detected) > 0:
                result = detected[detected.conf.argmax()].xyxy[0].cpu().numpy()
                rospy.logdebug(f"Keyboard detected at {result}")
                return result
            else:
                rospy.logdebug("No keyboard detected.")
                return None
        except Exception as e:
            rospy.logerr(f"Error scanning for keyboard: {e}")
            return None

    def display_output_l(self):
        rospy.logdebug("Displaying left output")
        img = self.left_img.copy()
        if self.keyboard_points_left is not None:
            points = self.keyboard_points_left.astype(np.int32)
            cv2.polylines(img, [points], True, (0, 255, 0), 2)
            cv2.putText(img, "Tracked Keyboard", (points[0][0], points[0][1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        self.image_pub_l.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

    def display_output_r(self):
        rospy.logdebug("Displaying right output")
        img = self.right_img.copy()
        if self.keyboard_points_right is not None:
            points = self.keyboard_points_right.astype(np.int32)
            cv2.polylines(img, [points], True, (0, 255, 0), 2)
            cv2.putText(img, "Tracked Keyboard", (points[0][0], points[0][1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        self.image_pub_r.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

    def scanning_for_keyboard(self):
        if not self.keyboard_initialized:
            rospy.loginfo("Initial keyboard detection...")
            self.result_left = self.processing_img(self.left_img)
            self.result_right = self.processing_img(self.right_img)
            
            if self.result_left is not None and self.result_right is not None:
                success