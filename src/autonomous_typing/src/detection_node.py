#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
import cv2
import os

class Detector:
    def __init__(self):
        print("🚀 INITIALIZING DETECTION NODE...")
        rospy.init_node('detector', anonymous=True)
        
        print("🤖 Loading YOLO model...")
        self.model = self.load_yolo_model()
        if self.model is None:
            print("❌ Failed to load YOLO model, shutting down")
            return
        print("✅ YOLO model loaded successfully")
        
        # Camera intrinsics
        self.K = np.array([
            [360.01333, 0.0, 243.87228],
            [0.0, 360.013366699, 137.9218444],
            [0.0, 0.0, 1.0]
        ])
        self.K_inv = np.linalg.inv(self.K)
        print("📷 Camera intrinsics configured")
        
        # TF setup
        print("🔗 Setting up TF buffer...")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # CV Bridge
        self.bridge = CvBridge()
        print("🌉 CV Bridge initialized")

        # Publishers
        print("📡 Setting up publishers...")
        self.poses_pub = rospy.Publisher("/keyboard_key_poses", PoseArray, queue_size=1)
        self.ids_pub = rospy.Publisher("/keyboard_key_ids", Int32MultiArray, queue_size=1)
        print("✅ Publishers configured")

        # Check available camera topics
        print("📹 Checking available camera topics...")
        available_camera = self.find_best_camera_topic()

        # FIXED: Subscribe to the camera that actually exists
        print(f"📡 Setting up subscriber for: {available_camera}")
        if available_camera:
            rospy.Subscriber(available_camera, Image, self.single_image_callback)
            print(f"✅ Subscribed to {available_camera}")
        else:
            print("❌ No suitable camera topic found!")
            return

        # Class names
        self.class_names = [
            'accent', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'minus', 'plus', 'del', 'tab', 'q', 'w', 'e', 'r', 't', 
            'y', 'u', 'i', 'o', 'p', '[', ']', 'enter', 'caps', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ':', 
            '"', '\\', 'shift-left', 'less', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', 'shift-right', 
            'ctrl-left', 'alt-left', 'space', 'alt-right', 'ctrl-right', 'keyboard'
        ]
        print(f"🔤 Loaded {len(self.class_names)} key classes")
        
        # Debug counters
        self.callback_count = 0
        self.detection_count = 0
        
        print("🎉 Detection node initialized successfully!")
        print("=" * 60)

    def find_best_camera_topic(self):
        """Find the best available camera topic."""
        try:
            import subprocess
            result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True, timeout=5)
            topics = result.stdout.split('\n')
            
            print("🔍 Available camera topics:")
            camera_topics = [t for t in topics if 'camera' in t and 'image_raw' in t and 'compressed' not in t]
            for topic in camera_topics[:10]:  # Show first 10
                print(f"   📹 {topic}")
            if len(camera_topics) > 10:
                print(f"   ... and {len(camera_topics) - 10} more")
            
            # Priority order for camera selection
            preferred_cameras = [
                '/camera_gripper_left/image_raw',
                '/camera_gripper_right/image_raw', 
                '/camera_base_left/image_raw',
                '/camera_base_right/image_raw',
                '/camera/color/image_raw'
            ]
            
            print("\n🎯 Camera selection priority:")
            for camera in preferred_cameras:
                if camera in topics:
                    print(f"   ✅ SELECTED: {camera}")
                    return camera
                else:
                    print(f"   ❌ Not available: {camera}")
            
            # Fallback: use any camera topic that contains 'image_raw'
            raw_image_topics = [t for t in topics if 'image_raw' in t and 'compressed' not in t and 'parameter' not in t]
            if raw_image_topics:
                fallback = raw_image_topics[0]
                print(f"   🔄 FALLBACK: Using {fallback}")
                return fallback
                
            print("   ❌ No suitable camera found!")
            return None
                
        except Exception as e:
            print(f"⚠️  Could not check topics: {e}")
            return None

    def load_yolo_model(self):
        """Load YOLO model with extensive debugging."""
        possible_paths = [
            'trained_yolov8n.pt',
            os.path.join(os.path.dirname(__file__), 'trained_yolov8n.pt'),
            '/root/ros_ws/src/Robotic_Arm/trained_yolov8n.pt',
            os.path.join(os.path.dirname(__file__), '../../../../trained_yolov8n.pt')
        ]
        
        print("🔍 Searching for YOLO model...")
        for i, model_path in enumerate(possible_paths):
            print(f"   {i+1}. Checking: {model_path}")
            if os.path.exists(model_path):
                print(f"   ✅ Found model at: {model_path}")
                try:
                    model = YOLO(model_path)
                    print(f"   🎯 Model loaded successfully")
                    return model
                except Exception as e:
                    print(f"   ❌ Failed to load model: {e}")
            else:
                print(f"   ❌ Not found")
        
        print("❌ No valid YOLO model found")
        return None

    def single_image_callback(self, color_msg):
        """Process single camera image."""
        self.callback_count += 1
        print(f"\n📸 IMAGE CALLBACK #{self.callback_count}")
        print(f"   📏 Image size: {color_msg.width}x{color_msg.height}")
        print(f"   🕐 Timestamp: {color_msg.header.stamp}")
        print(f"   🗺️  Frame: {color_msg.header.frame_id}")
        
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            print(f"   ✅ Successfully converted image")
            
            # Process with YOLO
            self.process_frame_simple(color_image, color_msg.header)
            
        except Exception as e:
            print(f"   ❌ Image conversion error: {e}")

    def process_frame_simple(self, color_image, header):
        """Process frame with simple 3D positioning (no real depth)."""
        print(f"   🤖 Running YOLO detection...")
        
        try:
            # FIXED: Lower confidence threshold and add more debugging
            results = self.model(color_image, iou=0.5, conf=0.1, verbose=False)  # Lowered confidence from 0.3 to 0.1
            print(f"   📊 YOLO returned {len(results)} result(s)")
            
            poses = []
            class_ids = []
            
            if len(results) > 0:
                result = results[0]
                print(f"   🔍 Result type: {type(result)}")
                print(f"   🔍 Has boxes: {hasattr(result, 'boxes')}")
                
                if hasattr(result, 'boxes') and result.boxes is not None:
                    boxes = result.boxes
                    print(f"   📦 Boxes object: {type(boxes)}")
                    print(f"   📦 Boxes length: {len(boxes)}")
                    
                    # FIXED: Add more detailed box debugging
                    if len(boxes) > 0:
                        print(f"   🔍 First box type: {type(boxes[0]) if len(boxes) > 0 else 'None'}")
                        print(f"   🔍 Box attributes: {dir(boxes[0]) if len(boxes) > 0 else 'None'}")
                    
                    for i, box in enumerate(boxes):
                        try:
                            class_id = int(box.cls[0])
                            confidence = float(box.conf[0])
                            x1, y1, x2, y2 = box.xyxy[0].tolist()
                            
                            print(f"     📦 Box {i+1}: class_id={class_id}, conf={confidence:.3f}, bbox=[{x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}]")
                            
                            # FIXED: Check if class_id is valid and confidence is reasonable
                            if class_id < len(self.class_names) and confidence > 0.05:  # Very low threshold for debugging
                                key_name = self.class_names[class_id]
                                print(f"     ✅ Valid detection: '{key_name}' (conf={confidence:.3f})")
                                
                                # Create 3D pose with reasonable keyboard positioning
                                center_x, center_y = (x1 + x2) / 2, (y1 + y2) / 2
                                
                                pose = Pose()
                                
                                # Fixed keyboard position in front of robot
                                keyboard_distance = 0.6  # 60cm in front
                                keyboard_width = 0.45    # 45cm wide
                                keyboard_height = 0.15   # 15cm tall
                                
                                # Convert pixel coordinates to keyboard coordinates
                                img_center_x, img_center_y = color_image.shape[1] / 2, color_image.shape[0] / 2
                                
                                # Normalized position on keyboard (-1 to 1)
                                norm_x = (center_x - img_center_x) / img_center_x
                                norm_y = (center_y - img_center_y) / img_center_y
                                
                                # World position
                                pose.position.x = keyboard_distance
                                pose.position.y = -norm_x * keyboard_width / 2  # Negative for proper mapping
                                pose.position.z = 0.5 - norm_y * keyboard_height / 2  # Keyboard at 50cm height
                                pose.orientation.w = 1.0
                                
                                poses.append(pose)
                                class_ids.append(class_id)
                                
                                print(f"     📍 Pos: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
                            else:
                                if class_id >= len(self.class_names):
                                    print(f"     ❌ Invalid class_id {class_id} >= {len(self.class_names)}")
                                if confidence <= 0.05:
                                    print(f"     ❌ Low confidence {confidence:.3f}")
                                    
                        except Exception as box_error:
                            print(f"     ❌ Error processing box {i}: {box_error}")
                    
                    if poses:
                        # Publish all detections at once
                        pose_array = PoseArray()
                        pose_array.header = header
                        pose_array.header.frame_id = "base_link"  # Use robot base frame
                        pose_array.poses = poses
                        
                        id_array = Int32MultiArray()
                        id_array.data = class_ids
                        
                        self.poses_pub.publish(pose_array)
                        self.ids_pub.publish(id_array)
                        self.detection_count += 1
                        
                        detected_keys = [self.class_names[cid] for cid in class_ids]
                        print(f"   📤 Published {len(poses)} detections: {', '.join(detected_keys)}")
                    else:
                        print(f"   📦 No valid detections to publish")
                else:
                    print(f"   📦 No boxes object or boxes is None")
                    # FIXED: Try alternative result access methods
                    print(f"   🔍 Result attributes: {dir(result)}")
                    if hasattr(result, 'pred') and result.pred is not None:
                        print(f"   🔍 Trying .pred access: {len(result.pred) if result.pred else 'None'}")
            else:
                print(f"   📦 No results returned from YOLO")
                
        except Exception as e:
            print(f"   ❌ YOLO processing error: {e}")
            import traceback
            traceback.print_exc()

    def print_status(self, event):
        """Print periodic status updates."""
        print(f"\n📊 DETECTION NODE STATUS:")
        print(f"   📞 Callbacks received: {self.callback_count}")
        print(f"   🎯 Real detections: {self.detection_count}")
        print("-" * 40)

    def run(self):
        print("🎬 Detection node running and waiting for images...")
        print("📊 Status updates every 10 seconds")
        
        # Status timer
        status_timer = rospy.Timer(rospy.Duration(10.0), self.print_status)
        
        rospy.spin()


    def single_image_callback(self, color_msg):
        """Process single camera image."""
        self.callback_count += 1
        print(f"\n📸 IMAGE CALLBACK #{self.callback_count}")
        print(f"   📏 Image size: {color_msg.width}x{color_msg.height}")
        print(f"   🕐 Timestamp: {color_msg.header.stamp}")
        print(f"   🗺️  Frame: {color_msg.header.frame_id}")

        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            print(f"   ✅ Successfully converted image")

            # FIXED: Save every 10th image for debugging
            if self.callback_count % 10 == 0:
                debug_path = f"/tmp/debug_image_{self.callback_count}.jpg"
                cv2.imwrite(debug_path, color_image)
                print(f"   💾 Saved debug image: {debug_path}")

            # Process with YOLO
            self.process_frame_simple(color_image, color_msg.header)

        except Exception as e:
            print(f"   ❌ Image conversion error: {e}")

if __name__ == '__main__':
    try:
        Detector().run()
    except rospy.ROSInterruptException:
        print("🛑 Detection node interrupted")
        pass