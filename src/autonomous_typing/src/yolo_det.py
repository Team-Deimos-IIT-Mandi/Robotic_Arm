import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point
import tf2_ros
import cv2
import numpy as np
from ultralytics import YOLO
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface

class Controller:
    def __init__(self):
        # Parameters
        self.KEYBOARD_LENGTH = 354.076
        self.KEYBOARD_WIDTH = 123.444
        self.class_to_detect = 66
        self.camera_input_left = rospy.get_param('~input_topic_left', '/camera_base_left/image_raw')
        self.camera_info_left = rospy.get_param('~camera_info_topic', '/camera_base_left/camera_info')
        self.camera_input_right = rospy.get_param('~input_topic_right', '/camera_base_right/image_raw')
        self.camera_info_right = rospy.get_param('~camera_info_topic', '/camera_base_right/camera_info')
        self.camera_frame_left = rospy.get_param('~camera_frame_left', 'camera_link_base_left')
        self.camera_frame_right = rospy.get_param('~camera_frame_right', 'camera_link_base_right')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.baseline = 0.1
        # YOLO model
        self.model = YOLO('yolov8n-seg.pt')

        # Subscribers
        self.bridge = CvBridge()
        self.subscriber_left = rospy.Subscriber(self.camera_input_left, Image, self.image_callback_left)
        self.subscriber_right = rospy.Subscriber(self.camera_input_right, Image, self.image_callback_right)
        self.camera_info_sub_left = rospy.Subscriber(self.camera_info_left, CameraInfo, self.camera_info_callback_left)
        self.camera_info_sub_right = rospy.Subscriber(self.camera_info_right, CameraInfo, self.camera_info_callback_right)

        # Images from cameras
        self.left_img = None
        self.right_img = None
        self.camera_info_left = None
        self.camera_info_right = None
        
        # Results from YOLO model
        self.result_left = None
        self.result_right = None

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # MoveIt setup
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("arm")

    def image_callback_left(self, msg):
        try:
            self.left_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.result_left = self.model(self.left_img, stream=True)
        except Exception as e:
            rospy.logerr(f"Error in left image callback: {e}")

    def image_callback_right(self, msg):
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.result_right = self.model(self.right_img, stream=True)
        except Exception as e:
            rospy.logerr(f"Error in right image callback: {e}")
            
    def camera_info_callback_left(self, msg):
        self.camera_info_left = msg
    def camera_info_callback_right(self, msg):
        self.camera_info_right = msg

    def calculate_3d_position(self, key_2d_left, key_2d_right, camera_info_left, camera_info_right):
        """Calculate the 3D position of a key using stereo vision."""
        fx_left = camera_info_left.K[0]
        fy_left = camera_info_left.K[4]
        cx_left = camera_info_left.K[2]
        cy_left = camera_info_left.K[5]

        fx_right = camera_info_right.K[0]
        fy_right = camera_info_right.K[4]
        cx_right = camera_info_right.K[2]
        cy_right = camera_info_right.K[5]

        disparity = key_2d_left[0] - key_2d_right[0]
        if disparity == 0:
            rospy.logerr("Disparity is zero, cannot calculate depth")
            return None

        z = (fx_left * self.baseline) / disparity
        x = (key_2d_left[0] - cx_left) * z / fx_left
        y = (key_2d_left[1] - cy_left) * z / fy_left
        return np.array([x, y, z])

    def get_transform(self, camera_point , camera_frame="camera_link2"):
        """Transform a point from the camera frame to the world frame."""
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        try:
            # Wait for the transform to be available
            transform = tf_buffer.lookup_transform("world", camera_frame, rospy.Time(0), rospy.Duration(1.0))
            # Transform the point
            world_point = do_transform_point(camera_point, transform)
            return world_point
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
            return None
    
    def move_arm_to_position(self, position_3d):
        """Move the robotic arm to the calculated 3D position."""
        pose_target = PoseStamped()
        pose_target.header.frame_id = self.base_frame
        pose_target.pose.position.x = position_3d[0]
        pose_target.pose.position.y = position_3d[1]
        pose_target.pose.position.z = position_3d[2]
        pose_target.pose.orientation.w = 1.0

        self.arm_group.set_pose_target(pose_target)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

    def check_key_pressed(self, position_3d):
        """Verify if the key is pressed correctly."""
        # Check if the key's 3D position has changed
        rospy.sleep(1.0)  # Wait for potential movement
        key_new_position = self.get_key_position_3d()
        if key_new_position is None:
            return False

        movement_threshold = 0.01  # Threshold for detecting key press
        displacement = np.linalg.norm(position_3d - key_new_position)
        return displacement > movement_threshold

    def get_key_position_3d(self):
        """Placeholder for obtaining the new 3D position of the key."""
        # This can be implemented with the YOLO model and stereovision again.
        return np.array([0.0, 0.0, 0.0])  # Replace with actual calculation

    def main(self):
        rospy.loginfo("Waiting for images...")
        rospy.sleep(2.0)  # Wait for the images to populate

        # Process images
        results_left = self.model(self.left_img)
        results_right = self.model(self.right_img)

        # Extract key positions (replace with your detection logic)
        key_2d_left = [100, 200]  # Example detection
        key_2d_right = [95, 200]  # Example detection

        camera_info_left = CameraInfo()  # Populate with actual camera info
        camera_info_right = CameraInfo()

        key_3d_position = self.calculate_3d_position(key_2d_left, key_2d_right, camera_info_left, camera_info_right)
        if key_3d_position is None:
            rospy.logerr("Could not calculate 3D position")
            return

        rospy.loginfo(f"3D Position of key: {key_3d_position}")

        # Move the robotic arm
        if self.move_arm_to_position(key_3d_position):
            rospy.loginfo("Key reached successfully")

            # Verify key press
            if self.check_key_pressed(key_3d_position):
                rospy.loginfo("Key pressed successfully")
            else:
                rospy.logerr("Key press failed")
        else:
            rospy.logerr("Failed to move arm to position")


if __name__ == "__main__":
    rospy.init_node("autonomous_typing_controller")
    controller = Controller()
    controller.main()
