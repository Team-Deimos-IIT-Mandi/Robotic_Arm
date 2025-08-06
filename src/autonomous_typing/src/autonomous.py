import rospy
import logging
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PointStamped
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point
import tf2_ros
import cv2
import numpy as np
from ultralytics import YOLO
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import os
from visualization_msgs.msg import Marker, MarkerArray
import random


class Controller:
    def __init__(self, debug=False):
        # --- Basic ROS and Logging Setup ---
        log_level = rospy.DEBUG if debug else rospy.INFO
        logging.basicConfig(level=logging.DEBUG if debug else logging.INFO)
        rospy.init_node("autonomous_typing", log_level=log_level)

        rospy.loginfo("Initializing Controller...")

        # --- Key Detection Setup (Using individual key detection logic) ---
        self.class_to_key_map = {
            0: 'A', 1: 'B', 2: 'C', 3: 'D', 4: 'E', 5: 'F',
            6: 'G', 7: 'H', 8: 'I', 9: 'J', 10: 'K', 11: 'L',
            12: 'M', 13: 'N', 14: 'O', 15: 'P', 16: 'Q', 17: 'R',
            18: 'S', 19: 'T', 20: 'U', 21: 'V', 22: 'W', 23: 'X',
            24: 'Y', 25: 'Z', 'a': 'A', 'b': 'B',
            26: '1', 27: '2', 28: '3', 29: '4', 30: '5',
            31: '6', 32: '7', 33: '8', 34: '9', 35: '0',
            36: 'SPACE', 37: 'ENTER', 38: 'CAPSLOCK', 39: '-', 40: '_'
        }
        rospy.loginfo(f"Loaded {len(self.class_to_key_map)} key mappings.")

        # --- ROS Parameters ---
        self.camera_input_left = rospy.get_param('~input_topic_left', '/camera_gripper_left/image_raw')
        self.camera_info_left_topic = rospy.get_param('~camera_info_topic_left', '/camera_gripper_left/camera_info')
        self.camera_input_right = rospy.get_param('~input_topic_right', '/camera_gripper_right/image_raw')
        self.camera_info_right_topic = rospy.get_param('~camera_info_topic_right', '/camera_gripper_right/camera_info')
        self.camera_frame_left = rospy.get_param('~camera_frame_left', 'camera_link_gripper_left')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        # --- Load YOLO Model ---
        self.model = YOLO('/root/ros_ws/src/Robotic_Arm/trained_yolov8n.pt')

        # --- ROS Communication ---
        self.bridge = CvBridge()
        self.subscriber_left = rospy.Subscriber(self.camera_input_left, Image, self.image_callback_left)
        self.subscriber_right = rospy.Subscriber(self.camera_input_right, Image, self.image_callback_right)
        self.camera_info_sub_left = rospy.Subscriber(self.camera_info_left_topic, CameraInfo, self.camera_info_callback_left)
        self.camera_info_sub_right = rospy.Subscriber(self.camera_info_right_topic, CameraInfo, self.camera_info_callback_right)
        self.annotated_image_pub_l = rospy.Publisher("/annotated_image_l", Image, queue_size=10)
        self.annotated_image_pub_r = rospy.Publisher("/annotated_image_r", Image, queue_size=10)

        # --- State Variables ---
        self.left_img = None
        self.right_img = None
        self.camera_info_left = None
        self.camera_info_right = None
        self.keyboard_points_3d = None
        self.baseline = 0.1

        # --- TF and MoveIt ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.arm_group = MoveGroupCommander("body")
        
        # Enhanced MoveIt settings for better performance
        self.arm_group.set_planning_time(10.0)  # Increase planning time
        self.arm_group.set_num_planning_attempts(5)  # More attempts
        self.arm_group.set_goal_position_tolerance(0.005)  # 5mm tolerance
        self.arm_group.set_goal_orientation_tolerance(0.1)  # More lenient orientation
        self.arm_group.set_max_velocity_scaling_factor(0.3)  # Slower, more controlled movement
        self.arm_group.set_max_acceleration_scaling_factor(0.3)
        #marker for keys in gazebo
        self.marker_pub = rospy.Publisher("/keyboard_markers", MarkerArray, queue_size=10)

        
        rospy.loginfo("✅ Controller initialized successfully.")

    # Callbacks just store the latest data
    def image_callback_left(self, msg): self.left_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    def image_callback_right(self, msg): self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    def camera_info_callback_left(self, msg): self.camera_info_left = msg
    def camera_info_callback_right(self, msg): self.camera_info_right = msg

    def detect_keys_in_image(self, img):
        detected_keys = {}
        if img is None: return detected_keys
        results = self.model(img, conf=0.4, iou=0.6, verbose=False)
        for box in results[0].boxes:
            class_id = int(box.cls.item())
            key_char = self.class_to_key_map.get(class_id)
            if key_char:
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                detected_keys[key_char] = (center_x, center_y)
        return detected_keys
    
    def run(self):
        rospy.loginfo("Waiting for camera feeds...")
        while self.left_img is None or self.right_img is None or self.camera_info_left is None:
            rospy.sleep(0.5)
        rospy.loginfo("✅ Camera feeds are active.")

        while not rospy.is_shutdown():
            if not self.scan_and_map_keyboard():
                rospy.logerr("Failed to create a 3D map. Retrying in 5s...")
                rospy.sleep(5)
                continue
            
            # Display all 3D coordinates
            self.display_3d_coordinates()
            
            try:
                input_string = raw_input("Enter the string to type (or Ctrl+C to exit): ")
            except NameError:
                input_string = input("Enter the string to type (or Ctrl+C to exit): ")

            if not input_string: continue

            clicks = self.string_to_keyboard_clicks(input_string)
            self.execute_typing(clicks)

    def display_3d_coordinates(self):
        """Display all detected keys and their 3D coordinates"""
        rospy.loginfo("🗺️  KEYBOARD 3D COORDINATE MAP:")
        rospy.loginfo("="*60)
        
        if not self.keyboard_points_3d:
            rospy.logwarn("No 3D coordinates available!")
            return
            
        # Sort keys for better readability
        sorted_keys = sorted(self.keyboard_points_3d.keys())
        
        for key in sorted_keys:
            pos = self.keyboard_points_3d[key]
            rospy.loginfo(f"Key '{key:>8}': X={pos[0]:+8.4f}, Y={pos[1]:+8.4f}, Z={pos[2]:+8.4f}")
        
        rospy.loginfo("="*60)
        
        # Calculate workspace bounds
        if len(self.keyboard_points_3d) > 0:
            positions = list(self.keyboard_points_3d.values())
            min_x, max_x = min(p[0] for p in positions), max(p[0] for p in positions)
            min_y, max_y = min(p[1] for p in positions), max(p[1] for p in positions)
            min_z, max_z = min(p[2] for p in positions), max(p[2] for p in positions)
            
            rospy.loginfo(f"📐 WORKSPACE BOUNDS:")
            rospy.loginfo(f"   X: {min_x:+8.4f} to {max_x:+8.4f} (range: {max_x-min_x:8.4f})")
            rospy.loginfo(f"   Y: {min_y:+8.4f} to {max_y:+8.4f} (range: {max_y-min_y:8.4f})")
            rospy.loginfo(f"   Z: {min_z:+8.4f} to {max_z:+8.4f} (range: {max_z-min_z:8.4f})")
            rospy.loginfo("="*60)

    # def visualize_keys_3d(self):
    #     """Visualize detected keys as 3D markers in RViz"""
    #     if not self.keyboard_points_3d:
    #         rospy.logwarn("No 3D coordinates to visualize!")
    #         return
        
    #     marker_array = MarkerArray()
        
    #     # Clear previous markers first
    #     clear_marker = Marker()
    #     clear_marker.action = Marker.DELETEALL
    #     marker_array.markers.append(clear_marker)
        
    #     rospy.loginfo("🎨 Creating 3D visualization markers for detected keys...")
        
    #     # Add keyboard base visualization
    #     keyboard_base = Marker()
    #     keyboard_base.header.frame_id = self.base_frame
    #     keyboard_base.header.stamp = rospy.Time.now()
    #     keyboard_base.ns = "keyboard_base"
    #     keyboard_base.id = 999  # Unique ID for keyboard
    #     keyboard_base.type = Marker.CUBE
    #     keyboard_base.action = Marker.ADD
        
    #     # Calculate keyboard center from detected keys
    #     if self.keyboard_points_3d:
    #         positions = list(self.keyboard_points_3d.values())
    #         center_x = sum(p[0] for p in positions) / len(positions)
    #         center_y = sum(p[1] for p in positions) / len(positions)
    #         center_z = min(p[2] for p in positions) - 0.01  # Slightly below keys
            
    #         keyboard_base.pose.position.x = center_x
    #         keyboard_base.pose.position.y = center_y
    #         keyboard_base.pose.position.z = center_z
    #         keyboard_base.pose.orientation.w = 1.0
            
    #         # Keyboard dimensions
    #         keyboard_base.scale.x = 0.45
    #         keyboard_base.scale.y = 0.15
    #         keyboard_base.scale.z = 0.02
            
    #         # Dark gray color
    #         keyboard_base.color.r = 0.2
    #         keyboard_base.color.g = 0.2
    #         keyboard_base.color.b = 0.2
    #         keyboard_base.color.a = 0.6
            
    #         marker_array.markers.append(keyboard_base)
        
    #     # Add detected key markers
    #     for i, (key, pos) in enumerate(self.keyboard_points_3d.items()):
    #         # Text marker for key label
    #         text_marker = Marker()
    #         text_marker.header.frame_id = self.base_frame
    #         text_marker.header.stamp = rospy.Time.now()
    #         text_marker.ns = "keyboard_keys_text"
    #         text_marker.id = i * 2  # Even IDs for text
    #         text_marker.type = Marker.TEXT_VIEW_FACING
    #         text_marker.action = Marker.ADD
            
    #         # Position
    #         text_marker.pose.position.x = pos[0]
    #         text_marker.pose.position.y = pos[1]
    #         text_marker.pose.position.z = pos[2] + 0.03  # Above the key
    #         text_marker.pose.orientation.w = 1.0
            
    #         # Scale and color
    #         text_marker.scale.z = 0.02  # Larger text
    #         text_marker.color.r = 1.0
    #         text_marker.color.g = 1.0
    #         text_marker.color.b = 0.0  # Bright yellow text
    #         text_marker.color.a = 1.0
    #         text_marker.text = key
            
    #         # Cube marker for key (more realistic than sphere)
    #         key_marker = Marker()
    #         key_marker.header.frame_id = self.base_frame
    #         key_marker.header.stamp = rospy.Time.now()
    #         key_marker.ns = "keyboard_keys"
    #         key_marker.id = i * 2 + 1  # Odd IDs for keys
    #         key_marker.type = Marker.CUBE
    #         key_marker.action = Marker.ADD
            
    #         # Position
    #         key_marker.pose.position.x = pos[0]
    #         key_marker.pose.position.y = pos[1]
    #         key_marker.pose.position.z = pos[2]
    #         key_marker.pose.orientation.w = 1.0
            
    #         # Key dimensions (more realistic)
    #         key_marker.scale.x = 0.012  # 12mm key width
    #         key_marker.scale.y = 0.012  # 12mm key depth
    #         key_marker.scale.z = 0.008  # 8mm key height
            
    #         # Bright colors for visibility
    #         hue = (i * 137.5) % 360  # Golden angle for good color distribution
    #         if hue < 60:
    #             key_marker.color.r = 1.0
    #             key_marker.color.g = hue / 60.0
    #             key_marker.color.b = 0.0
    #         elif hue < 120:
    #             key_marker.color.r = (120 - hue) / 60.0
    #             key_marker.color.g = 1.0
    #             key_marker.color.b = 0.0
    #         elif hue < 180:
    #             key_marker.color.r = 0.0
    #             key_marker.color.g = 1.0
    #             key_marker.color.b = (hue - 120) / 60.0
    #         elif hue < 240:
    #             key_marker.color.r = 0.0
    #             key_marker.color.g = (240 - hue) / 60.0
    #             key_marker.color.b = 1.0
    #         elif hue < 300:
    #             key_marker.color.r = (hue - 240) / 60.0
    #             key_marker.color.g = 0.0
    #             key_marker.color.b = 1.0
    #         else:
    #             key_marker.color.r = 1.0
    #             key_marker.color.g = 0.0
    #             key_marker.color.b = (360 - hue) / 60.0
            
    #         key_marker.color.a = 0.9  # Slightly transparent
            
    #         marker_array.markers.append(text_marker)
    #         marker_array.markers.append(key_marker)
        
    #     # Publish the markers
    #     self.marker_pub.publish(marker_array)
    #     rospy.loginfo(f"✨ Published keyboard + {len(self.keyboard_points_3d)} key markers to /keyboard_markers")
    #     rospy.loginfo("   💡 Open RViz and add MarkerArray topic '/keyboard_markers' to see the keys!")
        
    def scan_and_map_keyboard(self):
        rospy.loginfo("📸 Scanning keyboard with both cameras...")
        keypoints_left = self.detect_keys_in_image(self.left_img)
        keypoints_right = self.detect_keys_in_image(self.right_img)
        rospy.loginfo(f"   Left Cam detected: {list(keypoints_left.keys())}")
        rospy.loginfo(f"   Right Cam detected: {list(keypoints_right.keys())}")

        positions_3d_cam = self.calc_3d_pos(keypoints_left, keypoints_right)
        if not positions_3d_cam: return False

        rospy.loginfo("🌍 Transforming key positions to robot's coordinate frame...")
        self.keyboard_points_3d = {}
        for key, pos3d in positions_3d_cam.items():
            transformed_pos = self.get_transform(pos3d)
            if transformed_pos:
                self.keyboard_points_3d[key] = np.array(transformed_pos)
        
        rospy.loginfo(f"✅ Mapped {len(self.keyboard_points_3d)} keys in 3D.")
        rospy.loginfo(f"   Keys: {list(self.keyboard_points_3d.keys())}")
        
        # Add 3D visualization after successful mapping
        # self.visualize_keys_3d()
        
        return len(self.keyboard_points_3d) > 0

    def calc_3d_pos(self, keypoints_left, keypoints_right):
        common_keys = set(keypoints_left.keys()) & set(keypoints_right.keys())
        if not common_keys:
            rospy.logwarn("No common keys found between views.")
            return {}
        
        fx, fy = self.camera_info_left.K[0], self.camera_info_left.K[4]
        cx, cy = self.camera_info_left.K[2], self.camera_info_left.K[5]
        
        keypoints_3d = {}
        for key in common_keys:
            x_l, y_l = keypoints_left[key]
            x_r, _ = keypoints_right[key]
            disparity = float(x_l - x_r)
            if disparity > 1.0:
                Z = (fx * self.baseline) / disparity
                X = (x_l - cx) * Z / fx
                Y = (y_l - cy) * Z / fy
                keypoints_3d[key] = [X, Y, Z]
        return keypoints_3d

    def get_transform(self, position_3d):
        try:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.camera_frame_left
            point_stamped.header.stamp = rospy.Time(0)
            point_stamped.point.x, point_stamped.point.y, point_stamped.point.z = position_3d
            
            transformed_point_stamped = do_transform_point(point_stamped, self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame_left, rospy.Time(0), rospy.Duration(1.0)))
            
            p = transformed_point_stamped.point
            return [p.x, p.y, p.z]
        except Exception as e:
            rospy.logerr(f"Transform error in get_transform: {e}")
            return None

    def string_to_keyboard_clicks(self, input_string):
        clicks = []
        is_caps = False
        for char in input_string:
            if char.isupper() and not is_caps:
                clicks.append("CAPSLOCK")
                is_caps = True
            elif char.islower() and is_caps:
                clicks.append("CAPSLOCK")
                is_caps = False
            
            if char.isspace():
                clicks.append("SPACE")
            else:
                clicks.append(char)
        clicks.append("ENTER")
        return clicks

    # def check_arm_reachability(self, target_pos):
    #     """Check if the target position is reachable before attempting to move"""
    #     try:
    #         # Get current pose
    #         current_pose = self.arm_group.get_current_pose().pose
    #         current_pos = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
            
    #         # Calculate distance
    #         distance = np.linalg.norm(np.array(target_pos) - np.array(current_pos))
            
    #         # Check workspace limits (adjust these based on your robot's specifications)
    #         # workspace_limits = {
    #         #     'x_min': -1.0, 'x_max': 1.0,
    #         #     'y_min': -1.0, 'y_max': 1.0,
    #         #     'z_min': 0.0, 'z_max': 1.5
    #         # }
            
    #         # Check if target is within workspace
    #         # if not (workspace_limits['x_min'] <= target_pos[0] <= workspace_limits['x_max'] and
    #         #         workspace_limits['y_min'] <= target_pos[1] <= workspace_limits['y_max'] and
    #         #         workspace_limits['z_min'] <= target_pos[2] <= workspace_limits['z_max']):
    #         #     rospy.logwarn(f"🚫 Target position {target_pos} is outside workspace limits")
    #         #     return False
            
    #         # Check if distance is reasonable
    #         # if distance > 2.0:  # 2 meters maximum reach
    #         #     rospy.logwarn(f"🚫 Target too far: {distance:.3f}m from current position")
    #         #     return False
                
    #         rospy.loginfo(f"✅ Target position is reachable (distance: {distance:.3f}m)")
    #         return True
            
    #     except Exception as e:
    #         rospy.logerr(f"Error checking reachability: {e}")
    #         return False

    # def execute_typing(self, clicks_to_perform):
    #     rospy.loginfo(f"🤖 Starting typing for: {''.join(clicks_to_perform)}")
    #     if not self.keyboard_points_3d:
    #         rospy.logerr("No 3D map available.")
    #         return

    #     # Get current position as safe return point
    #     try:
    #         current_pose = self.arm_group.get_current_pose().pose
    #         safe_return_pos = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
    #         rospy.loginfo(f"🏠 Safe return position: {safe_return_pos}")
    #     except Exception as e:
    #         rospy.logerr(f"Failed to get current pose: {e}")
    #         return

    #     # Calculate workspace bounds for hover positions
    #     positions = list(self.keyboard_points_3d.values())
    #     max_z = np.max([p[2] for p in positions])
    #     safe_hover_height = max_z + 0.10  # 10cm above highest key
    #     rospy.loginfo(f"🔝 Safe hover height calculated: {safe_hover_height:.4f}m")
        
    #     for i, click in enumerate(clicks_to_perform):
    #         key_to_press = click.upper() if click.isalpha() else click

    #         rospy.loginfo(f"🎯 [{i+1}/{len(clicks_to_perform)}] Pressing '{key_to_press}'")
            
    #         if key_to_press not in self.keyboard_points_3d:
    #             rospy.logwarn(f"⚠️ Key '{key_to_press}' not found in map. Skipping.")
    #             continue

    #         target_pos = self.keyboard_points_3d[key_to_press]
    #         rospy.loginfo(f"📍 Target 3D coordinates: X={target_pos[0]:+8.4f}, Y={target_pos[1]:+8.4f}, Z={target_pos[2]:+8.4f}")
            
    #         # Define movement sequence
    #         press_hover_pos = np.array([target_pos[0], target_pos[1], safe_hover_height])
    #         press_pos = np.array([target_pos[0], target_pos[1], target_pos[2] + 0.01])  # 1cm above key

    #         movement_sequence = [
    #             (press_hover_pos, "Hovering above key"),
    #             (press_pos, "Moving to press position"),
    #             (press_hover_pos, "Retreating to hover")
    #         ]

    #         success = True
    #         for pos, action in movement_sequence:
    #             rospy.loginfo(f"   {action}... Target: [{pos[0]:+6.3f}, {pos[1]:+6.3f}, {pos[2]:+6.3f}]")
                
    #             if not self.move_arm_to_position(pos, timeout=15.0):  # Increased timeout
    #                 rospy.logerr(f"❌ Failed while {action} for key '{key_to_press}'. Aborting sequence.")
    #                 success = False
    #                 break
                    
    #             # Small delay between movements
    #             rospy.sleep(0.2)
            
    #         if not success:
    #             rospy.logwarn("🔄 Attempting to return to safe position...")
    #             self.move_arm_to_position(safe_return_pos, timeout=10.0)
    #             break
        
    #     rospy.loginfo("🏠 Returning to safe position...")
    #     self.move_arm_to_position(safe_return_pos, timeout=10.0)
    #     rospy.loginfo("✅ Typing sequence complete.")
    def execute_typing(self, clicks_to_perform):
        rospy.loginfo(f"🤖 Starting typing for: {''.join(clicks_to_perform)}")
        if not self.keyboard_points_3d:
            rospy.logerr("No 3D map available.")
            return

        for i, click in enumerate(clicks_to_perform):
            key_to_press = click.upper() if click.isalpha() else click

            rospy.loginfo(f"🎯 [{i+1}/{len(clicks_to_perform)}] Moving to '{key_to_press}'")

            if key_to_press not in self.keyboard_points_3d:
                rospy.logwarn(f"⚠️ Key '{key_to_press}' not found in map. Skipping.")
                continue

            target_pos = self.keyboard_points_3d[key_to_press]
            rospy.loginfo(f"📍 Target 3D coordinates: X={target_pos[0]:+8.4f}, Y={target_pos[1]:+8.4f}, Z={target_pos[2]:+8.4f}")

            if not self.move_arm_to_position(target_pos, timeout=15.0):
                rospy.logerr(f"❌ Failed to move to key '{key_to_press}'. Aborting sequence.")
                break

            rospy.sleep(0.2)

        rospy.loginfo("✅ Typing sequence complete.")

    def move_arm_to_position(self, position_3d, timeout=10.0):
        """Enhanced arm movement with better error handling and detailed debugging"""
        plan_success = False
        execution_success = False
        
        try:
            # Clear any previous targets
            self.arm_group.clear_pose_targets()
            
            # Create target pose
            target_pose = Pose()
            target_pose.position.x, target_pose.position.y, target_pose.position.z = position_3d
            
            # Keep current orientation or set a safe default
            try:
                current_pose = self.arm_group.get_current_pose().pose
                target_pose.orientation = current_pose.orientation
                rospy.loginfo(f"🧭 Using current orientation: [{target_pose.orientation.x:.3f}, {target_pose.orientation.y:.3f}, {target_pose.orientation.z:.3f}, {target_pose.orientation.w:.3f}]")
            except Exception as e:
                rospy.logwarn(f"⚠️ Could not get current orientation, using default: {e}")
                # Fallback orientation (pointing down)
                target_pose.orientation.w = 1.0
                target_pose.orientation.x = 0.0
                target_pose.orientation.y = 0.0
                target_pose.orientation.z = 0.0
            
            # Set planning parameters
            self.arm_group.set_planning_time(timeout)
            self.arm_group.set_pose_target(target_pose)
            
            # Plan and execute with detailed debugging
            rospy.loginfo(f"⚙️  Planning movement to [{position_3d[0]:+6.3f}, {position_3d[1]:+6.3f}, {position_3d[2]:+6.3f}]...")
            rospy.loginfo(f"📋 Planning parameters:")
            rospy.loginfo(f"   • Planning time: {timeout}s")
            try:
                rospy.loginfo(f"   • Position tolerance: {self.arm_group.get_goal_position_tolerance():.4f}m")
                rospy.loginfo(f"   • Orientation tolerance: {self.arm_group.get_goal_orientation_tolerance():.4f}rad")
            except AttributeError:
                rospy.loginfo("   • Tolerance info not available in this MoveIt version")
            
            # Get current state for debugging
            try:
                current_pose = self.arm_group.get_current_pose().pose
                current_pos = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
                distance = np.linalg.norm(np.array(position_3d) - np.array(current_pos))
                rospy.loginfo(f"📍 Current position: [{current_pos[0]:+6.3f}, {current_pos[1]:+6.3f}, {current_pos[2]:+6.3f}]")
                rospy.loginfo(f"📏 Distance to target: {distance:.4f}m")
            except Exception as e:
                rospy.logwarn(f"⚠️ Could not get current pose for debugging: {e}")
            
            # Plan with detailed result checking
            rospy.loginfo("🔄 Starting planning...")
            plan_result = self.arm_group.plan()
            rospy.loginfo("✅ Planning call completed")
            
            # Handle different MoveIt versions
            trajectory = None
            planning_time = 0.0
            error_code = None
            
            if isinstance(plan_result, tuple):
                # Newer MoveIt versions return (success, trajectory, planning_time, error_code)
                plan_success, trajectory, planning_time, error_code = plan_result
                rospy.loginfo(f"📊 Planning result (tuple format):")
                rospy.loginfo(f"   • Success: {plan_success}")
                rospy.loginfo(f"   • Planning time: {planning_time:.3f}s")
                rospy.loginfo(f"   • Error code: {error_code}")
            else:
                # Older MoveIt versions return just the trajectory
                trajectory = plan_result
                # Check if trajectory is valid
                if trajectory and hasattr(trajectory, 'joint_trajectory') and len(trajectory.joint_trajectory.points) > 0:
                    plan_success = True
                    rospy.loginfo(f"📊 Planning result (trajectory format):")
                    rospy.loginfo(f"   • Trajectory points: {len(trajectory.joint_trajectory.points)}")
                else:
                    plan_success = False
                    rospy.loginfo("📊 Planning result: No valid trajectory")
            
            # Execute if planning was successful
            if plan_success and trajectory and hasattr(trajectory, 'joint_trajectory') and len(trajectory.joint_trajectory.points) > 0:
                rospy.loginfo(f"✅ Plan successful with {len(trajectory.joint_trajectory.points)} waypoints")
                rospy.loginfo("🚀 Executing trajectory...")
                
                # Execute without calling stop first
                execution_success = self.arm_group.execute(trajectory, wait=True)
                
                if execution_success:
                    rospy.loginfo("✅ Movement completed successfully")
                    return True
                else:
                    rospy.logerr("❌ Execution failed")
                    return False
            else:
                rospy.logerr("❌ Planning failed:")
                if not plan_success:
                    rospy.logerr(f"   • Planning unsuccessful")
                    if error_code is not None:
                        rospy.logerr(f"   • Error code: {error_code}")
                elif not trajectory:
                    rospy.logerr("   • No trajectory returned")
                elif not hasattr(trajectory, 'joint_trajectory'):
                    rospy.logerr("   • Invalid trajectory format")
                else:
                    rospy.logerr(f"   • Empty trajectory (points: {len(trajectory.joint_trajectory.points) if hasattr(trajectory, 'joint_trajectory') else 'N/A'})")
                return False
                    
        except Exception as e:
            rospy.logerr(f"❌ Exception in move_arm_to_position: {e}")
            import traceback
            rospy.logerr("🔍 Full traceback:")
            rospy.logerr(traceback.format_exc())
            return False
        finally:
            # Only stop if we haven't successfully executed
            if not execution_success:
                rospy.loginfo("🛑 Stopping arm movement due to failure")
                self.arm_group.stop()
            self.arm_group.clear_pose_targets()

def main():
    try:
        controller = Controller(debug=True)  # Enable debug for better logging
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted by user")
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted by user")
    except Exception as e:
        rospy.logfatal(f"An unhandled exception occurred: {e}")

if __name__ == "__main__":
    main()