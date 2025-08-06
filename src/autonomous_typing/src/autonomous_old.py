#!/usr/bin/env python3
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

class Controller:
    def __init__(self, debug=False):
        self.debug = debug
        if self.debug:
            rospy.init_node("autonomous_typing", log_level=rospy.DEBUG)
            logging.basicConfig(level=logging.DEBUG)
        else:
            rospy.init_node("autonomous_typing", log_level=rospy.INFO)
            logging.basicConfig(level=logging.INFO)
        
        rospy.loginfo("Initializing self")
        # Initialize variables
        self.KEYBOARD_LENGTH = 354.076
        self.KEYBOARD_WIDTH = 123.444
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_path = os.path.join(current_dir, 'keyboard_layout.json')
        with open(json_path, 'r') as f:
            rospy.loginfo(f"Loading keyboard points from {json_path}")
            self.keyboard_points = json.load(f)
            # print(self.keyboard_points['E'])
        
        self.class_to_detect = 66
        # Parameters
        self.camera_input_left = rospy.get_param('~input_topic_left', '/camera_gripper_left/image_raw')
        self.camera_info_left = rospy.get_param('~camera_info_topic', '/camera_gripper_left/camera_info')
        self.camera_input_right = rospy.get_param('~input_topic_right', '/camera_gripper_right/image_raw')
        self.camera_info_right = rospy.get_param('~camera_info_topic', '/camera_gripper_right/camera_info')
        self.camera_frame_left = rospy.get_param('~camera_frame_left', 'camera_link_gripper_left')
        self.camera_frame_right = rospy.get_param('~camera_frame_right', 'camera_link_gripper_right')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        rospy.logdebug("Loading YOLO model")
        model_path = '/root/ros_ws/src/Robotic_Arm/trained_yolov8n.pt'
        rospy.loginfo(f"Loading YOLO model from {model_path}")
        self.model = YOLO(model_path) # <--- USE THE FULL PATH
        logging.getLogger("ultralytics").setLevel(logging.ERROR)
        rospy.sleep(1)


        # Subscribers
        self.bridge = CvBridge()
        self.subscriber_left = rospy.Subscriber(self.camera_input_left, Image, self.image_callback_left)
        self.subscriber_right = rospy.Subscriber(self.camera_input_right, Image, self.image_callback_right)
        self.camera_info_sub_left = rospy.Subscriber(self.camera_info_left, CameraInfo, self.camera_info_callback_left)
        self.camera_info_sub_right = rospy.Subscriber(self.camera_info_right, CameraInfo, self.camera_info_callback_right)
        
        # Publishers
        self.annotated_image_pub_l = rospy.Publisher("/annotated_image_l", Image, queue_size=10)
        self.annotated_image_pub_r = rospy.Publisher("/annotated_image_r", Image, queue_size=10)
        
        # Initialize variables
        self.left_img = None
        self.right_img = None
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
            self.keypoints_left, corners = self.processing_image(self.left_img)
            self.display_output_l(self.left_img, corners)
        except Exception as e:
            # rospy.logerr(f"Error in left image callback: {e}")
            pass

    def image_callback_right(self, msg):
        rospy.logdebug("Right image callback triggered")
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.keypoints_right, corners = self.processing_image(self.right_img)
            self.display_output_r(self.right_img, corners)
        except Exception as e:
            # rospy.logerr(f"Error in right image callback: {e}")
            pass
            
    def camera_info_callback_left(self, msg):
        rospy.logdebug("Left camera info callback triggered")
        self.camera_info_left = msg

    def camera_info_callback_right(self, msg):
        rospy.logdebug("Right camera info callback triggered")
        self.camera_info_right = msg
    
    def display_output_l(self, img, corners):
        rospy.logdebug("Displaying left output")
        for i in range(4):
            cv2.line(img, corners[i], corners[(i+1) % 4], (0, 255, 0), 2)
        # draw the keypoints
        for keypoint in self.keypoints_left:
            cv2.circle(img, (int(self.keypoints_left[keypoint][0]), int(self.keypoints_left[keypoint][1])), 5, (0, 0, 255), -1)
            
        annotated_image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        
        self.annotated_image_pub_l.publish(annotated_image_msg)
        rospy.logdebug("Published annotated left image")

    def display_output_r(self, img, corners):
        rospy.logdebug("Displaying right output")
        for i in range(4):
            cv2.line(img, corners[i], corners[(i+1) % 4], (0, 255, 0), 2)
        for keypoint in self.keypoints_right:
            cv2.circle(img, (int(self.keypoints_right[keypoint][0]), int(self.keypoints_right[keypoint][1])), 5, (0, 0, 255), -1)
            
        annotated_image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.annotated_image_pub_r.publish(annotated_image_msg)
        rospy.logdebug("Published annotated right image")
            
    def processing_image(self, img):
        rospy.logdebug("Processing image")
        scaled_points = {}
        corners = []
        if img is None:
            return scaled_points, corners

        try:
            results = self.model(img, stream=True, conf=0.4)
            
            all_points = []
            # This loop collects the corner points of ALL detected keys
            for result in results:
                if result.boxes:
                    for box in result.boxes:
                        # We don't need to check class_id, we assume any detection is a key
                        points = box.xyxy[0].cpu().numpy()
                        x1, y1, x2, y2 = points
                        all_points.extend([(x1, y1), (x2, y2)])

            # If we detected at least one key, proceed
            if all_points:
                # Find the min/max coordinates to create one large bounding box
                all_x = [p[0] for p in all_points]
                all_y = [p[1] for p in all_points]
                
                master_x1, master_y1 = min(all_x), min(all_y)
                master_x2, master_y2 = max(all_x), max(all_y)
                
                box_length = master_x2 - master_x1
                box_width = master_y2 - master_y1

                # Check for a valid bounding box size to avoid errors
                if box_length > 10 and box_width > 10:
                    rospy.logdebug(f"Master Box dimensions - Length: {box_length}, Width: {box_width}")
                    corners = [(master_x1, master_y1), (master_x2, master_y1), (master_x2, master_y2), (master_x1, master_y2)]

                    # Now, use this master box to scale the keys from the JSON layout
                    for key, value in self.keyboard_points.items():
                        try:
                            scaled_x = (float(value[0]) / self.KEYBOARD_LENGTH) * box_length + master_x1
                            scaled_y = (float(value[1]) / self.KEYBOARD_WIDTH) * box_width + master_y1
                            scaled_points[key] = [int(scaled_x), int(scaled_y)]
                        except ValueError:
                            rospy.logerr(f"Non-numeric value for key '{key}': {value}")
            
            return scaled_points, corners

        except StopIteration:
            rospy.logwarn("No YOLO results in this frame.")
            return {}, []
        except Exception as e:
            rospy.logerr(f"Error in processing_image: {e}")
            return {}, []


    def run(self):
        """
        The main control loop. It waits for the keyboard to be detected,
        maps it in 3D, and then enters the typing loop.
        """
        rospy.loginfo("Waiting for keyboard detection in both camera feeds...")
        
        # Loop until both keypoint dictionaries are NOT empty
        while (not self.keypoints_left or not self.keypoints_right) and not rospy.is_shutdown():
            rospy.loginfo("Waiting... Left Keys: %s, Right Keys: %s", 
                          "No" if not self.keypoints_left else "Yes", 
                          "No" if not self.keypoints_right else "Yes")
            rospy.sleep(0.5) # Wait half a second before checking again

        rospy.loginfo("✅ Keyboard detected in camera feeds. Proceeding to map.")

        # Attempt to create the 3D map
        if not self.map_the_keyboard():
             rospy.logerr("Failed to create a 3D map of the keyboard. Please check camera views and restart.")
             return # Exit if mapping fails

        rospy.loginfo("✅ Keyboard mapped successfully in 3D. Ready for user input.")

        # Now that the map exists, we can start the typing process
        self.execute_typing_sequence()


    def map_the_keyboard(self):
        """
        Calculates the 3D positions of all keys and stores them.
        Returns True on success, False on failure.
        """
        rospy.loginfo("Calculating 3D positions of keys...")
        # Use the most recent keypoints detected by the callbacks
        key_positions_in_camera_frame = self.calc_3d_pos(
            self.keypoints_left, self.keypoints_right, 
            self.camera_info_left, self.camera_info_right
        )

        if not key_positions_in_camera_frame:
            rospy.logerr("Could not calculate any 3D key positions from stereo.")
            return False

        # Transform points from camera frame to world/base frame
        transformed_points = {}
        for key, pos_3d_cam in key_positions_in_camera_frame.items():
            pos_3d_world = self.get_transform(pos_3d_cam)
            if pos_3d_world is not None:
                transformed_points[key] = np.array(pos_3d_world)
        
        if not transformed_points:
            rospy.logerr("Could not transform any key points to the world frame. Check TF tree.")
            return False

        self.keyboard_points_3d = transformed_points
        rospy.loginfo(f"Successfully mapped {len(self.keyboard_points_3d)} keys in 3D.")
        return True

        
    def calc_3d_pos(self, keypoints_left, keypoints_right, camera_info_left, camera_info_right):
        """
        Calculate the 3D position of keypoints using stereo vision.

        :param keypoints_left: Dictionary of keypoints from the left camera image (labels: positions).
        :param keypoints_right: Dictionary of keypoints from the right camera image (labels: positions).
        :param camera_info_left: CameraInfo message for the left camera.
        :param camera_info_right: CameraInfo message for the right camera.
        :param baseline: The baseline distance between the left and right cameras (in meters).
        :return: A dictionary of keypoints with their 3D positions (labels: [X, Y, Z]).
        """
        if keypoints_left is None or keypoints_right is None:
            rospy.logwarn("Keypoints from one or both cameras are missing.")
            return {}

        if camera_info_left is None or camera_info_right is None:
            rospy.logwarn("Camera info for one or both cameras is missing.")
            return {}
        rospy.loginfo(f"Left camera keypoints:\n{json.dumps(keypoints_left, indent=2)}")
        rospy.loginfo(f"Right camera keypoints:\n{json.dumps(keypoints_right, indent=2)}")

        # Extract camera intrinsic parameters
        fx_left = camera_info_left.K[0]  # Focal length in x-direction (left camera)
        fy_left = camera_info_left.K[4]  # Focal length in y-direction (left camera)
        cx_left = camera_info_left.K[2]  # Principal point x-coordinate (left camera)
        cy_left = camera_info_left.K[5]  # Principal point y-coordinate (left camera)

        fx_right = camera_info_right.K[0]  # Focal length in x-direction (right camera)
        fy_right = camera_info_right.K[4]  # Focal length in y-direction (right camera)
        cx_right = camera_info_right.K[2]  # Principal point x-coordinate (right camera)
        cy_right = camera_info_right.K[5]  # Principal point y-coordinate (right camera)

        keypoints_3d = {}

        for key in keypoints_left:
            if key not in keypoints_right:
                rospy.logwarn(f"Keypoint {key} not found in right camera image.")
                continue

            # Get pixel coordinates from left and right images
            x_left, y_left = keypoints_left[key]
            x_right, y_right = keypoints_right[key]

            # Calculate disparity (difference in x-coordinates)
            disparity = x_left - x_right

            if disparity == 0:
                rospy.logwarn(f"Disparity for keypoint {key} is zero.")
                continue

            # Calculate depth (Z) using the disparity
            Z = (fx_left * self.baseline) / disparity

            # Calculate X and Y in 3D space
            X = (x_left - cx_left) * Z / fx_left
            Y = (y_left - cy_left) * Z / fy_left

            # Store the 3D position
            keypoints_3d[key] = [X, Y, Z]

        return keypoints_3d
        
    def get_transform(self, point):
        """Transform a point from the camera frame to the world frame with proper tip offset."""
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
            
            # Apply tip offset - since camera is 21.4cm away from tip as per my calculation from urdf 
            # The tip is closer to the keyboard, so we need to move the target closer to the arm
            # Adjust based on your actual setup - this moves target 21.4cm closer in Y direction
            tip_offset_world = np.array([0, 0.214, 0])  # Move 21.4cm closer in Y direction (positive Y brings it closer to arm base)
            
            final_position = np.array([
                world_point.point.x + tip_offset_world[0],
                world_point.point.y + tip_offset_world[1], 
                world_point.point.z + tip_offset_world[2]
            ])
            
            rospy.loginfo(f"Original camera position: [{world_point.point.x:.3f}, {world_point.point.y:.3f}, {world_point.point.z:.3f}]")
            rospy.loginfo(f"Adjusted tip position: [{final_position[0]:.3f}, {final_position[1]:.3f}, {final_position[2]:.3f}]")
            
            return final_position.tolist()
            
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
            return None
            
    
    # Modify the control flow to use the new method
    # def get_keyboard_points(self):
    #     """Get keyboard points using PnP on both cameras"""
    #     if self.keypoints_left and self.camera_info_left:
    #         left_points = self.calculate_3d_position(
    #             self.keypoints_left, 
    #             self.camera_info_left
    #         )
        
    #     if self.keypoints_right and self.camera_info_right:
    #         right_points = self.calculate_3d_position(
    #             self.keypoints_right, 
    #             self.camera_info_right
    #         )
        
    #     # Merge or validate points from both cameras if needed
    #     self.keyboard_points_3d = left_points or right_points
    
    # def display_output_l(self, img, corners):
    #     # Create a box
    #     for i in range(4):
    #         cv2.line(img, corners[i], corners[(i+1) % 4], (0, 255, 0), 2)
        
    #     # Convert to ROS Image message and publish
    #     annotated_image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
    #     self.annotated_image_pub_l.publish(annotated_image_msg)
    #     # rospy.loginfo("Published annotated image.")
    # def display_output_r(self, img, corners):
    #     # Create a box
    #     for i in range(4):
    #         cv2.line(img, corners[i], corners[(i+1) % 4], (0, 255, 0), 2)
        
    #     # Convert to ROS Image message and publish
    #     annotated_image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
    #     self.annotated_image_pub_r.publish(annotated_image_msg)
    #     # rospy.loginfo("Published annotated image.")
        


    # def move_arm_to_position(self, position_3d, tolerance=0.001):
    #     """
    #     Move the robotic arm to the calculated 3D position with 1mm precision efficiently.
        
    #     Args:
    #         position_3d (np.array): Target 3D position [x, y, z]
    #         tolerance (float): Acceptable distance to target position (default 1mm)
        
    #     Returns:
    #         bool: Success of movement
    #     """
    #     try:
    #         current_pose = self.arm_group.get_current_pose().pose

    #         # Compute movement vector
    #         dx = position_3d[0] - current_pose.position.x
    #         dy = position_3d[1] - current_pose.position.y
    #         dz = position_3d[2] - current_pose.position.z
    #         distance = np.linalg.norm([dx, dy, dz])

    #         if distance < tolerance:
    #             rospy.loginfo("Already within tolerance.")
    #             return True  # Already at target

    #         # Number of steps required (1mm increments)
    #         num_steps = int(np.ceil(distance / 0.001))
    #         step_vector = np.array([dx, dy, dz]) / distance * 0.001  # 1mm step direction

    #         waypoints = []
    #         for i in range(1, num_steps + 1):
    #             target_pose = Pose()
    #             target_pose.position.x = current_pose.position.x + step_vector[0] * i
    #             target_pose.position.y = current_pose.position.y + step_vector[1] * i
    #             target_pose.position.z = current_pose.position.z + step_vector[2] * i
    #             target_pose.orientation = current_pose.orientation  # Maintain orientation
    #             waypoints.append(target_pose)

    #         # Plan and execute full trajectory in one go
    #         (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, eef_step=0.001) #removed jumped threshold

    #         if fraction < 0.99:  # Ensure most of the path is planned
    #             rospy.logerr("Path planning failed, only %.2f%% completed" % (fraction * 100))
    #             return False

    #         self.arm_group.execute(plan, wait=True)
    #         self.arm_group.stop()
    #         self.arm_group.clear_pose_targets()

    #         return True

    #     except Exception as e:
    #         rospy.logerr(f"Error in move_arm_to_position: {e}")
    #         return False
    def move_arm_to_position(self, position_3d, use_cartesian=False, timeout=10.0):
        """
        Moves the arm to a target position using MoveIt's planner.
        More conservative approach with better error handling.
        """
        rospy.loginfo(f"Planning and moving to: {position_3d}")
        
        # Get current position for comparison
        current_pose = self.arm_group.get_current_pose().pose
        current_pos = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
        distance = np.linalg.norm(position_3d - current_pos)
        
        rospy.loginfo(f"Current position: {current_pos}")
        rospy.loginfo(f"Target position: {position_3d}")
        rospy.loginfo(f"Distance to target: {distance:.4f}m")
        
        # More conservative safety threshold
        if distance > 0.8: #kee 80cm for accuracy
            rospy.logerr(f"Target position is too far ({distance:.3f}m). Aborting for safety.")
            return False
        
        try:
            if use_cartesian:
                rospy.loginfo("Attempting Cartesian path planning...")
                
                target_pose = Pose()
                target_pose.position.x = position_3d[0]
                target_pose.position.y = position_3d[1] 
                target_pose.position.z = position_3d[2]
                target_pose.orientation = current_pose.orientation
                
                waypoints = [target_pose]
                
                # Try Cartesian path with detailed logging
                rospy.loginfo("Computing Cartesian path...")
                (plan, fraction) = self.arm_group.compute_cartesian_path(
                    waypoints, 
                    eef_step=0.01  # Larger step size for better success observed in practice
                )
                
                rospy.loginfo(f"Cartesian path planning result: {fraction*100:.1f}% successful")
                
                if fraction >= 0.8:  # Higher threshold for Cartesian
                    rospy.loginfo("Cartesian path acceptable - executing...")
                    success = self.arm_group.execute(plan, wait=True)
                    if success:
                        rospy.loginfo("Cartesian execution successful!")
                    else:
                        rospy.logerr("Cartesian execution failed!")
                else:
                    rospy.logwarn(f"Cartesian path insufficient ({fraction*100:.1f}%) - falling back to standard planning")
                    success = self._standard_planning(position_3d, current_pose, timeout)
                            
            else:
                rospy.loginfo("Attempting standard planning...")
                success = self._standard_planning(position_3d, current_pose, timeout)
            
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            
            if success:
                # Verify final position
                final_pose = self.arm_group.get_current_pose().pose
                final_pos = np.array([final_pose.position.x, final_pose.position.y, final_pose.position.z])
                final_distance = np.linalg.norm(position_3d - final_pos)
                rospy.loginfo(f"Movement successful! Final distance to target: {final_distance:.4f}m")
            else:
                rospy.logerr("Movement failed!")
                
            return success
            
        except Exception as e:
            rospy.logerr(f"Error in move_arm_to_position: {e}")
            import traceback
            traceback.print_exc()
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            return False

    def _standard_planning(self, position_3d, current_pose, timeout):
        """Helper function for standard planning with more conservative settings"""
        target_pose = Pose()
        target_pose.position.x = position_3d[0]
        target_pose.position.y = position_3d[1]
        target_pose.position.z = position_3d[2]
        target_pose.orientation = current_pose.orientation
        
        rospy.loginfo("Setting up standard planning...")
        
        # More conservative planning settings
        rospy.loginfo(f"Setting planning time: {timeout}s")
        self.arm_group.set_planning_time(timeout)
        
        rospy.loginfo("Setting planning attempts: 5")  # Increased attempts to 5
        self.arm_group.set_num_planning_attempts(5)
        
        # Set planning constraints for better success
        self.arm_group.set_goal_position_tolerance(0.01)  # 1cm tolerance
        self.arm_group.set_goal_orientation_tolerance(0.1)  # Relaxed orientation tolerance
        
        rospy.loginfo("Setting pose target...")
        self.arm_group.set_pose_target(target_pose)
        
        rospy.loginfo("Starting motion planning...")
        success = self.arm_group.go(wait=True)
        
        if not success:
            rospy.logerr("Standard planning failed!")
            # Try with relaxed constraints
            rospy.loginfo("Trying with relaxed constraints...")
            self.arm_group.set_goal_position_tolerance(0.02)  # 2cm tolerance
            self.arm_group.set_goal_orientation_tolerance(0.2)  # More relaxed orientation
            success = self.arm_group.go(wait=True)
            
            if success:
                rospy.loginfo("Planning succeeded with relaxed constraints!")
            else:
                rospy.logerr("Planning failed even with relaxed constraints!")
        
        return success

    def disable_keyboard_collision(self):
        """
        Disable collision detection with keyboard to allow typing motions.
        """
        try:
            # Remove keyboard from collision objects if it exists
            rospy.loginfo("Disabling keyboard collision detection...")
            
            # Get all known objects in the planning scene
            known_objects = self.scene.get_known_object_names()
            
            # Remove any objects that might represent the keyboard
            keyboard_objects = [obj for obj in known_objects if 'keyboard' in obj.lower()]
            for obj in keyboard_objects:
                self.scene.remove_world_object(obj)
                rospy.loginfo(f"Removed collision object: {obj}")
            
            # Allow collision between end-effector and any remaining objects
            self.arm_group.set_support_surface_name("keyboard")
            
            rospy.sleep(1.0)  # Wait for scene update
            
        except Exception as e:
            rospy.logerr(f"Error disabling keyboard collision: {e}")

    def get_tip_position_from_camera_detection(self, camera_position_3d):
        """
        Convert camera-detected position to actual tip position.
        The tip is 21.4cm away from the camera.
        
        Args:
            camera_position_3d: 3D position detected by camera
            
        Returns:
            np.array: Adjusted position for the tip
        """
        ###########################################Todo#################################################


        # The tip is 21.4cm (0.214m) away from the camera
        # Assuming the tip is in the negative Y direction relative to camera frame from urdf i have top fix this also if it didnt worked
        tip_offset = np.array([0, -0.214, 0])  # Adjust this based on your actual tip orientation
        
        return camera_position_3d + tip_offset
    # def move_arm_to_position(self, position_3d, tolerance=0.02):
    #     """
    #     Move the robotic arm to the calculated 3D position with incremental steps.
        
    #     Args:
    #         position_3d (np.array): Target 3D position 
    #         tolerance (float): Acceptable distance to target position
        
    #     Returns:
    #         bool: Success of movement
    #     """
    #     try:
    #         # Get current pose
    #         current_pose = self.arm_group.get_current_pose().pose
            
    #         # Compute the distance to the goal
    #         dx = position_3d[0] - current_pose.position.x
    #         dy = position_3d[1] - current_pose.position.y
    #         dz = position_3d[2] - current_pose.position.z
    #         distance = np.linalg.norm([dx, dy, dz])

    #         # Move in small steps until within tolerance
    #         step_size = 0.01
    #         while distance > tolerance and not rospy.is_shutdown():
    #             # Calculate step sizes proportional to direction
    #             step_dx = dx * min(step_size / distance, 1.0)
    #             step_dy = dy * min(step_size / distance, 1.0)
    #             step_dz = dz * min(step_size / distance, 1.0)

    #             # Create target pose
    #             target_pose = Pose()
    #             target_pose.position.x = current_pose.position.x + step_dx
    #             target_pose.position.y = current_pose.position.y + step_dy
    #             target_pose.position.z = current_pose.position.z + step_dz
    #             target_pose.orientation = current_pose.orientation

    #             # Set and execute target pose
    #             self.arm_group.set_pose_target(target_pose)
    #             success = self.arm_group.go(wait=True)
    #             self.arm_group.stop()
                
    #             if not success:
    #                 rospy.logerr("Failed to move incrementally")
    #                 return False

    #             # Update current pose and recalculate distance
    #             current_pose = self.arm_group.get_current_pose().pose
    #             dx = position_3d[0] - current_pose.position.x
    #             dy = position_3d[1] - current_pose.position.y
    #             dz = position_3d[2] - current_pose.position.z
    #             distance = np.linalg.norm([dx, dy, dz])

    #         self.arm_group.clear_pose_targets()
    #         return True

    #     except Exception as e:
    #         rospy.logerr(f"Error in move_arm_to_position: {e}")
    #         return False

    def check_key_pressed(self, position_3d):
        """Verify if the key is pressed correctly."""
        # Check if the key's 3D position has changed
        rospy.sleep(1.0)  # Wait for potential movement
        key_new_position = self.get_key_position_3d()
        if key_new_position is None:
            return False

        movement_threshold = 0.02  # Threshold for detecting key press
        displacement = np.linalg.norm(position_3d - key_new_position)
        return displacement > movement_threshold

    def get_key_position_3d(self):
        """Placeholder for obtaining the new 3D position of the key."""
        # This can be implemented with the YOLO model and stereovision again.
        return np.array([0.0, 0.0, 0.0])  # Replace with actual calculation

    def string_to_keyboard_clicks(self,input_string):
        keyboard_clicks = []
        caps_active = False  # Track CAPS state

        for char in input_string:
            if char.isupper() and not caps_active:
                # Activate CAPS if the character is uppercase and CAPS is not active
                keyboard_clicks.append("CAPSLOCK")
                caps_active = True
            elif char.islower() and caps_active:
                # Deactivate CAPS if the character is lowercase and CAPS is active
                keyboard_clicks.append("CAPSLOCK")
                caps_active = False
            
            if char.isalnum() or char in {'-', '_'}:  # Letters, numbers, and some symbols
                keyboard_clicks.append(char.upper() if not caps_active else char)
            elif char.isspace():
                keyboard_clicks.append("SPACE")
            else:
                # Add any non-alphanumeric, non-space character as is
                keyboard_clicks.append(char)
        
        # End with ENTER
        keyboard_clicks.append("ENTER")
        
        return keyboard_clicks
    
    # def control_flow(self):
    #     """
    #     Execute keyboard clicks by moving the robotic arm to each key's position
        
    #     Args:
    #         input_string (str): The string to be typed
        
    #     Returns:
    #         bool: Success of the entire typing operation
    #     """
    #     # Prompt user for input string
    #     input_string = input("Enter the string to type: ")

    #     # while True:
    #     #     print(f"You entered: {input_string}")
    #     #     confirmation = input("Do you want to proceed with this string? (y/n): ").lower()
    #     #     if confirmation == 'y':
    #     #         break
    #     #     else:
    #     #         input_string = input("Please enter the new string to type: ")
        
    #     # Convert input string to keyboard clicks
    #     keyboard_clicks = self.string_to_keyboard_clicks(input_string)
    #     print(f"Keyboard clicks: {keyboard_clicks}")
        
    #     # Check if keyboard points are detected
    #     # if not self.keyboard_points_3d:
    #     #     print("Keyboard points not detected. Please ensure the keyboard is in view.")
    #     # self.get_key_position_3d()
    #     # print(self.keyboard_points_3d)
    #     ans = self.calc_3d_pos(self.keypoints_left, self.keypoints_right, self.camera_info_left, self.camera_info_right)
    #     # ans = zip(ans.keys(),self.get_transform(ans.values()))
    #     for key in ans:
    #         ans[key] = np.array(self.get_transform(ans[key]))
    #         # print(f"{key} : {ans[key]}")
    #     self.keyboard_points_3d = ans
        
    #     try:
    #         # Move to home position before starting
    #         home_pose = self.arm_group.get_current_pose().pose
            
    #         self.arm_group.set_pose_target(home_pose)
    #         self.arm_group.go(wait=True)
    #         self.arm_group.stop()
            
    #         # dummy_pose = Pose()
    #         # dummy_pose.position.x = 0.0
    #         # dummy_pose.position.y = -0.65
    #         # dummy_pose.position.z = 0.3
    #         # dummy_pose.orientation.w = 1.0
    #         # self.arm_group.set_pose_target(dummy_pose)
    #         # success = self.arm_group.go(wait=True)
    #         # input("Press Enter to continue...")
            
    #         # Iterate through keyboard clicks
    #         # for k in self.keyboard_points:
    #         #     print(k, self.keyboard_points[k])
    #         for click in keyboard_clicks:
    #             # Check if the click exists in keyboard points
    #             if click not in self.keyboard_points:
    #                 rospy.logwarn(f"Key {click} not found in keyboard points. Skipping.")
    #                 continue
                
    #             # Get the 3D position for this key
    #             key_position = self.keyboard_points_3d[click]
    #             # hover_position =   key_position + np.array([0, -0.1, 0])
                
    #             # # Move arm to the key position
    #             # print(hover_position)
    #             # success = self.move_arm_to_position(hover_position)
    #             # print(success)
    #             # Before moving to a key
    #             if click not in self.keyboard_points_3d:
    #                 rospy.logwarn(f"3D position for key {click} not found. Skipping.")
    #                 continue

    #             # When moving to position
    #             success = self.move_arm_to_position(key_position)
    #             if not success:
    #                 rospy.logerr(f"Failed to move to key {click}. Attempting to continue.")
    #                 continue  # Skip this key instead of stopping entire operation
                
    #             # success = self.move_arm_to_position(key_position)
    #             if not success:
    #                 rospy.logerr(f"Failed to move to key {click}")
    #                 return False
                
    #             # Simulate key press (you might want to add actual key press mechanism)
    #             rospy.sleep(0.5)  # Brief pause to simulate key press
                
    #             # Optional: Verify key press
    #             if not self.check_key_pressed(key_position):
    #                 rospy.logwarn(f"Key press verification failed for {click}")
                    
    #             if success:
    #                 rospy.loginfo(f"Successfully typed key {click}")
                
    #             # success = self.move_arm_to_position(home_pose)
    #             # succ
    #             rospy.sleep(0.5)
                
    #             self.arm_group.set_pose_target(home_pose)
    #             self.arm_group.go(wait=True)
    #             self.arm_group.stop()
    #             # if success:
    #             #     rospy.loginfo(f"Successfully returned to home position")
    #             # else:
    #             #     rospy.logerr(f"Failed to return to home position")
    #             #     return False
                
                
            
    #         # Return to home position after typing
    #         self.arm_group.set_pose_target(home_pose)
    #         self.arm_group.go(wait=True)
    #         self.arm_group.stop()
            
    #         return True
        
    #     except Exception as e:
    #         rospy.logerr(f"Error in control flow: {e}")
    #         return False
    
    # def control_flow(self):
    #     """
    #     Execute keyboard clicks by moving the robotic arm to each key's position
        
    #     Returns:
    #         bool: Success of the entire typing operation
    #     """
    #     # Prompt user for input string
    #     input_string = input("Enter the string to type: ")

    #     # Convert input string to keyboard clicks
    #     keyboard_clicks = self.string_to_keyboard_clicks(input_string)
    #     print(f"Keyboard clicks: {keyboard_clicks}")
        
    #     # Calculate 3D positions of keyboard points
    #     ans = self.calc_3d_pos(self.keypoints_left, self.keypoints_right, self.camera_info_left, self.camera_info_right)
    #     for key in ans:
    #         ans[key] = np.array(self.get_transform(ans[key]))
    #     self.keyboard_points_3d = ans

    #     try:
    #         # Move to home position before starting
    #         home_position = np.array([
    #             self.arm_group.get_current_pose().pose.position.x,
    #             self.arm_group.get_current_pose().pose.position.y,
    #             self.arm_group.get_current_pose().pose.position.z
    #         ])

    #         if not self.move_arm_to_position(home_position):
    #             rospy.logerr("Failed to move to the home position initially.")
    #             return False

    #         # Iterate through keyboard clicks
    #         for click in keyboard_clicks:
    #             # Check if the click exists in keyboard points
    #             if click not in self.keyboard_points:
    #                 rospy.logwarn(f"Key {click} not found in keyboard points. Skipping.")
    #                 continue

    #             # Get the 3D position for this key
    #             if click not in self.keyboard_points_3d:
    #                 rospy.logwarn(f"3D position for key {click} not found. Skipping.")
    #                 continue

    #             key_position = self.keyboard_points_3d[click]+np.array([0,0.1,0.0])

    #             # # Move arm to the key position
    #             # rospy.loginfo(f"Moving to key {click} at position {key_position}.")
    #             if not self.move_arm_to_position(key_position):
    #                 rospy.logerr(f"Failed to move to key {click}. Attempting to continue.")
    #             else:
    #                 rospy.loginfo(f"Successfully moved to key {click}.")
                
    #             # key_position = self.keyboard_points_3d[click]
                
    #             # When moving to position
    #             # success = self.move_arm_to_position(key_position)

    #             # Simulate key press
    #             rospy.sleep(0.1)  # Brief pause to simulate key press
                
    #             # Move back to the home pose after key press
    #             rospy.loginfo("Returning to home position...")
    #             if not self.move_arm_to_position(home_position):
    #                 rospy.logerr("Failed to return to home position. Stopping the operation.")
    #                 return False
    #             else:
    #                 rospy.loginfo("Successfully returned to home position.")
                
    #             # # Optional: Verify key press  
    #             # if not self.check_key_pressed(key_position):
    #             #     rospy.logwarn(f"Key press verification failed for {click}")


    #             # Move back to the home pose after key press
    #             # rospy.loginfo("Returning to home position...")
    #             # if not self.move_arm_to_position(home_position):
    #             #     rospy.logerr("Failed to return to home position. Stopping the operation.")
    #             #     return False

    #     except Exception as e:
    #         rospy.logerr(f"An error occurred: {e}")
    #         return False

    #     return True
    def execute_typing_sequence(self):
        """
        Prompts the user for a string and executes the typing motion.
        More conservative approach with intermediate waypoints.
        """
        try:
            input_string = input("Enter the string to type: ")
        except (EOFError, KeyboardInterrupt):
            rospy.loginfo("Exiting.")
            return

        keyboard_clicks = self.string_to_keyboard_clicks(input_string)
        print(f"Keyboard clicks: {keyboard_clicks}")
        
        if not self.keyboard_points_3d:
            rospy.logerr("Cannot execute typing, the 3D keyboard map is empty.")
            return

        try:
            # Disable keyboard collision detection
            self.disable_keyboard_collision()
            
            # Get home position helpful in returning after key presssin
            home_pose = self.arm_group.get_current_pose().pose
            home_position = np.array([home_pose.position.x, home_pose.position.y, home_pose.position.z])
            
            rospy.loginfo(f"Starting from home position: {home_position}")
            
            # Check which keys are available
            rospy.loginfo(f"Available keys in 3D map: {list(self.keyboard_points_3d.keys())}")
            
            # Use small height offset since tip is already close to keyboard
            press_height = 0.01  # 1cm above key for safety
            
            for click in keyboard_clicks:
                if click not in self.keyboard_points_3d:
                    rospy.logwarn(f"3D position for key {click} not found. Available keys: {list(self.keyboard_points_3d.keys())}")
                    continue

                rospy.loginfo(f"\n=== Typing key: '{click}' ===")
                
                # Use detected position with small offset
                key_position = self.keyboard_points_3d[click]
                press_position = key_position + np.array([0, 0, press_height])
                
                rospy.loginfo(f"Key base position: {key_position}")
                rospy.loginfo(f"Press position (with {press_height}m offset): {press_position}")

                # Create intermediate waypoint if distance is large
                distance_to_key = np.linalg.norm(press_position - home_position)
                if distance_to_key > 0.4:  # If more than 40cm away
                    # Create intermediate position halfway
                    intermediate_position = home_position + 0.5 * (press_position - home_position)
                    intermediate_position[2] += 0.05  # Add 5cm height for clearance
                    
                    rospy.loginfo(f"Large distance detected ({distance_to_key:.3f}m). Using intermediate waypoint.")
                    rospy.loginfo(f"Intermediate position: {intermediate_position}")
                    
                    # Move to intermediate position first
                    if not self.move_arm_to_position(intermediate_position, use_cartesian=False, timeout=30.0):
                        rospy.logerr(f"Failed to move to intermediate position for key {click}. Skipping.")
                        continue

                # Move directly to press position
                rospy.loginfo("=== Attempting to move to press position ===")
                
                # Try standard planning with longer timeout for far targets
                if not self.move_arm_to_position(press_position, use_cartesian=False, timeout=35.0):
                    rospy.logwarn(f"Standard planning failed for key {click}, trying Cartesian...")
                    if not self.move_arm_to_position(press_position, use_cartesian=True, timeout=20.0):
                        rospy.logerr(f"Both methods failed for key {click}. Skipping.")
                        continue
                
                rospy.loginfo(f"Successfully reached key {click}")
                rospy.sleep(0.2)  # Brief press simulation

            # Return home
            rospy.loginfo("\n=== Returning to home position ===")
            self.move_arm_to_position(home_position, use_cartesian=False, timeout=35.0)

        except Exception as e:
            rospy.logerr(f"An error occurred during the typing sequence: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            rospy.loginfo("Typing sequence completed.")
def main():
    controller = Controller(debug=False)
    # rospy.sleep(15)
    controller.run()

    # self.display_keyboard()
    # while not rospy.is_shutdown():
    # ans = controller.calc_3d_pos(controller.keypoints_left, controller.keypoints_right, controller.camera_info_left, controller.camera_info_right)
    # # ans = zip(ans.keys(),controller.get_transform(ans.values()))
    # for key in ans:
    #     print(f"3D position of {key}: {controller.get_transform(ans[key])}")
    # controller.control_flow()
    
    
    rospy.spin()

if __name__ == "__main__":
    main()