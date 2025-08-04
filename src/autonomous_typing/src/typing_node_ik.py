#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class AutonomousTypingIK:
    def __init__(self):
        print("🚀 INITIALIZING AUTONOMOUS TYPING NODE (IK MODE)...")
        rospy.init_node('autonomous_typing_ik', anonymous=True)
        
        # Publishers for your IK system
        print("📡 Setting up IK command publisher...")
        self.joint_pub = rospy.Publisher('/body_controller/command', JointTrajectory, queue_size=10)
        print("✅ Joint trajectory publisher configured")
        
        # Detection data storage
        self.key_pose_map = {}
        self.pose_map_updated = False
        self.command_sequence = "hello"
        print(f"📝 Target sequence: '{self.command_sequence}'")
        
        self.class_names = [
            'accent', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'minus', 'plus', 'del', 'tab', 'q', 'w', 'e', 'r', 't', 
            'y', 'u', 'i', 'o', 'p', '[', ']', 'enter', 'caps', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ':', 
            '"', '\\', 'shift-left', 'less', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', 'shift-right', 
            'ctrl-left', 'alt-left', 'space', 'alt-right', 'ctrl-right', 'keyboard'
        ]
        
        # Detection subscribers
        print("📡 Setting up detection subscribers...")
        self.latest_poses = None
        self.latest_ids = None
        self.poses_received = False
        self.ids_received = False
        self.callback_count = 0
        
        rospy.Subscriber("/keyboard_key_poses", PoseArray, self.poses_callback)
        rospy.Subscriber("/keyboard_key_ids", Int32MultiArray, self.ids_callback)
        print("✅ Detection subscribers configured")
        
        rospy.loginfo("🎉 Autonomous typing IK node initialized!")
        print("=" * 60)

    def poses_callback(self, msg):
        """Handle pose array messages."""
        print(f"\n📍 POSES CALLBACK - Received {len(msg.poses)} poses")
        self.latest_poses = msg
        self.poses_received = True
        self.try_process_detection()

    def ids_callback(self, msg):
        """Handle ID array messages."""
        print(f"\n🔢 IDS CALLBACK - Received {len(msg.data)} IDs")
        self.latest_ids = msg
        self.ids_received = True
        self.try_process_detection()

    def try_process_detection(self):
        """Process detection if we have both poses and IDs."""
        if not (self.poses_received and self.ids_received):
            return
        
        if self.latest_poses is None or self.latest_ids is None:
            return
        
        self.keyboard_callback(self.latest_poses, self.latest_ids)
        self.poses_received = False
        self.ids_received = False

    def keyboard_callback(self, poses_msg, ids_msg):
        """Process detected keyboard keys."""
        self.callback_count += 1
        print(f"\n📨 PROCESSING DETECTION #{self.callback_count}...")
        
        temp_map = {}
        class_ids = ids_msg.data
        poses = poses_msg.poses

        if len(class_ids) != len(poses):
            print(f"❌ MISMATCH: {len(class_ids)} IDs vs {len(poses)} poses")
            return

        detected_keys = []
        for i, class_id in enumerate(class_ids):
            if class_id < len(self.class_names):
                key_name = self.class_names[class_id]
                pose = poses[i]
                temp_map[key_name] = pose
                detected_keys.append(key_name)
                
                if key_name in self.command_sequence:
                    print(f"   🎯 KEY '{key_name}': x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
        
        print(f"   🔤 Detected keys: {', '.join(detected_keys[:10])}")
        
        missing_keys = [char for char in self.command_sequence if char not in temp_map]
        if missing_keys:
            print(f"   ⚠️  Missing keys: {missing_keys}")
        else:
            print(f"   ✅ All keys for '{self.command_sequence}' detected!")
        
        self.key_pose_map = temp_map
        if not self.pose_map_updated and self.key_pose_map:
            print(f"🗺️  KEYBOARD MAP CREATED: {len(self.key_pose_map)} keys")
            self.pose_map_updated = True

    def send_joint_command(self, joint_positions, description=""):
        """Send joint command to your IK system."""
        print(f"🎯 SENDING JOINT COMMAND: {description}")
        print(f"   🔧 Joints: {[f'{j:.3f}' for j in joint_positions]}")
        
        try:
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6']
            
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start = rospy.Duration(2.0)  # 2 seconds to reach position
            
            trajectory_msg.points = [point]
            self.joint_pub.publish(trajectory_msg)
            
            rospy.sleep(2.5)  # Wait for movement to complete
            print(f"   ✅ Joint command executed")
            return True
            
        except Exception as e:
            print(f"   ❌ Failed to send joint command: {e}")
            return False

    def go_to_home_position(self):
        """Move to home position using joint commands."""
        print("\n🏠 MOVING TO HOME POSITION...")
        # Define home joint positions (all zeros or a safe configuration)
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return self.send_joint_command(home_joints, "to home position")

    def move_to_key_position(self, target_pose, key_name):
        """Move to a key position - simplified version without real IK."""
        print(f"\n🎯 MOVING TO KEY '{key_name}'...")
        print(f"   📍 Target: x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}")
        
        # Simple heuristic joint positions based on target position
        # This is a simplified approach - you'd need proper IK for real implementation
        x, y, z = target_pose.position.x, target_pose.position.y, target_pose.position.z
        
        # Very basic joint calculations (this is just for demonstration)
        joint_1 = np.arctan2(y, x)  # Base rotation towards target
        joint_2 = -0.5  # Shoulder down
        joint_3 = 1.0   # Elbow up
        joint_4 = 0.0   # Wrist roll
        joint_5 = 0.5   # Wrist pitch down
        joint_6 = 0.0   # Wrist yaw
        
        joints = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
        
        # Hover position (5cm above)
        print("   🚁 Moving to hover position...")
        if not self.send_joint_command(joints, f"hover over {key_name}"):
            return False
        
        # Press position (move down)
        print("   ⌨️  Pressing key...")
        joints[5] += 0.1  # Small adjustment to "press"
        if not self.send_joint_command(joints, f"press {key_name}"):
            return False
        
        # Return to hover
        print("   📈 Returning to hover...")
        joints[5] -= 0.1
        return self.send_joint_command(joints, f"return from {key_name}")

    def wait_for_keyboard_detection(self, timeout=30.0):
        """Wait for keyboard detection."""
        print(f"\n⏳ WAITING FOR KEYBOARD DETECTION (timeout: {timeout}s)...")
        start_time = rospy.Time.now()
        
        while not self.pose_map_updated and not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            
            if int(elapsed) % 5 == 0 and int(elapsed) > 0:
                print(f"   ⏰ Waiting... {elapsed:.0f}/{timeout:.0f}s (callbacks: {self.callback_count})")
            
            if elapsed > timeout:
                print(f"   ⏰ TIMEOUT after {timeout}s")
                return False
            rospy.sleep(0.5)
        
        print(f"   ✅ Keyboard detected!")
        return True

    def type_sequence(self):
        """Type the target sequence."""
        print(f"\n⌨️  TYPING SEQUENCE: '{self.command_sequence}'")
        
        success_count = 0
        for i, char in enumerate(self.command_sequence):
            print(f"\n📝 TYPING CHARACTER {i+1}/{len(self.command_sequence)}: '{char}'")
            
            if char in self.key_pose_map:
                key_pose = self.key_pose_map[char]
                if self.move_to_key_position(key_pose, char):
                    success_count += 1
                    print(f"   ✅ Successfully typed '{char}'")
                else:
                    print(f"   ❌ Failed to type '{char}'")
            else:
                print(f"   ❌ Key '{char}' not found in detection map")
            
            rospy.sleep(1.0)  # Pause between characters
        
        print(f"\n🎉 TYPING COMPLETE! Success: {success_count}/{len(self.command_sequence)}")

    def run(self):
        """Main execution loop."""
        print("\n🚀 STARTING AUTONOMOUS TYPING (IK MODE)")
        print("=" * 60)
        
        try:
            # Step 1: Home position
            if not self.go_to_home_position():
                print("❌ Failed to reach home position")
                return
            print("✅ Home position reached")
            
            # Step 2: Wait for detection
            if not self.wait_for_keyboard_detection():
                print("❌ No keyboard detected")
                return
            print("✅ Keyboard detected")
            
            # Step 3: Type sequence
            self.type_sequence()
            
            # Step 4: Return home
            self.go_to_home_position()
            print("✅ Returned home")
            
            print("🎉 MISSION ACCOMPLISHED!")
            
        except KeyboardInterrupt:
            print("🛑 User interrupt")
        except Exception as e:
            print(f"❌ Error: {e}")
        finally:
            print("👋 Goodbye!")

if __name__ == '__main__':
    try:
        import numpy as np
        AutonomousTypingIK().run()
    except rospy.ROSInterruptException:
        pass