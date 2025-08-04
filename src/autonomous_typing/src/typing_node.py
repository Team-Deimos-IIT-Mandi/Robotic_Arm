#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
from moveit_commander import MoveGroupCommander, RobotCommander
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Int32MultiArray
from copy import deepcopy
import message_filters
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class AutonomousTyping:
    def __init__(self):
        print("🚀 INITIALIZING AUTONOMOUS TYPING NODE...")
        rospy.init_node('autonomous_typing', anonymous=True)
        
        print("🔧 Setting up MoveIt commander...")
        try:
            moveit_commander.roscpp_initialize(sys.argv)
            self.robot = RobotCommander()
            self.group = MoveGroupCommander("body")
            print("✅ MoveIt commander initialized successfully")
        except Exception as e:
            print(f"❌ Failed to initialize MoveIt: {e}")
            sys.exit(1)
        
        # Configuration with debug info
        print("⚙️  Configuring motion planning parameters...")
        self.group.set_planning_time(10.0)
        self.group.set_num_planning_attempts(10)
        self.group.set_max_velocity_scaling_factor(0.5)
        self.group.set_max_acceleration_scaling_factor(0.5)
        print(f"   📊 Planning time: {self.group.get_planning_time()}s")
        print(f"   🎯 Planning attempts: 10")
        print(f"   🚄 Velocity scaling: 0.5")
        print(f"   ⚡ Acceleration scaling: 0.5")
        
        # Debug: Print available planning groups
        print("📋 Available planning groups:")
        for group_name in self.robot.get_group_names():
            print(f"   • {group_name}")
        
        # Debug: Print current joint values
        print("🦾 Current joint state:")
        current_joints = self.group.get_current_joint_values()
        joint_names = self.group.get_active_joints()
        for name, value in zip(joint_names, current_joints):
            print(f"   • {name}: {value:.3f}")
        
        # This dictionary will map a character (e.g., 'h') to its 3D pose
        self.key_pose_map = {}
        self.pose_map_updated = False
        
        # Command sequence to type
        self.command_sequence = "hello"
        print(f"📝 Target sequence: '{self.command_sequence}'")
        
        self.class_names = [
            'accent', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'minus', 'plus', 'del', 'tab', 'q', 'w', 'e', 'r', 't', 
            'y', 'u', 'i', 'o', 'p', '[', ']', 'enter', 'caps', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ':', 
            '"', '\\', 'shift-left', 'less', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', 'shift-right', 
            'ctrl-left', 'alt-left', 'space', 'alt-right', 'ctrl-right', 'keyboard'
        ]
        
        # Setup TF buffer for coordinate transformations (debugging)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Debug: Check available frames
        rospy.sleep(2.0)  # Wait for TF buffer to fill
        print("🗺️  Available TF frames:")
        try:
            all_frames = self.tf_buffer.all_frames_as_string()
            frames = all_frames.split('\n')[:10]  # Show first 10 frames
            for frame in frames:
                if frame.strip():
                    print(f"   • {frame.strip()}")
            if len(frames) >= 10:
                print("   • ... (and more)")
        except Exception as e:
            print(f"   ⚠️  Could not list TF frames: {e}")
        
        # FIXED: Use separate subscribers instead of message filters to avoid synchronization issues
        print("📡 Setting up subscribers...")
        self.latest_poses = None
        self.latest_ids = None
        self.poses_received = False
        self.ids_received = False
        
        # Use separate subscribers
        rospy.Subscriber("/keyboard_key_poses", PoseArray, self.poses_callback)
        rospy.Subscriber("/keyboard_key_ids", Int32MultiArray, self.ids_callback)
        print("✅ Subscribers configured")
        
        # Debug counters
        self.callback_count = 0
        self.successful_detections = 0
        
        rospy.loginfo("🎉 Autonomous typing node initialized successfully!")
        print(f"🎯 Ready to type: '{self.command_sequence}'")
        print("=" * 60)

    def poses_callback(self, msg):
        """Handle pose array messages."""
        print(f"\n📍 POSES CALLBACK - Received {len(msg.poses)} poses")
        print(f"   🕐 Timestamp: {msg.header.stamp}")
        print(f"   🗺️  Frame ID: {msg.header.frame_id}")
        
        self.latest_poses = msg
        self.poses_received = True
        
        # Try to process if we have both poses and IDs
        self.try_process_detection()

    def ids_callback(self, msg):
        """Handle ID array messages."""
        print(f"\n🔢 IDS CALLBACK - Received {len(msg.data)} IDs")
        print(f"   📋 IDs: {msg.data[:10]}{'...' if len(msg.data) > 10 else ''}")
        
        self.latest_ids = msg
        self.ids_received = True
        
        # Try to process if we have both poses and IDs
        self.try_process_detection()

    def try_process_detection(self):
        """Process detection if we have both poses and IDs."""
        if not (self.poses_received and self.ids_received):
            print("   ⏳ Waiting for both poses and IDs...")
            return
        
        if self.latest_poses is None or self.latest_ids is None:
            print("   ❌ Missing pose or ID data")
            return
        
        # Process the detection
        self.keyboard_callback(self.latest_poses, self.latest_ids)
        
        # Reset flags for next detection
        self.poses_received = False
        self.ids_received = False

    def keyboard_callback(self, poses_msg, ids_msg):
        """Correctly maps detected key characters to their 3D poses with extensive debugging."""
        self.callback_count += 1
        
        print(f"\n📨 PROCESSING DETECTION #{self.callback_count}...")
        print(f"   🔢 Processing {len(ids_msg.data)} class IDs")
        print(f"   📍 Processing {len(poses_msg.poses)} poses")
        
        temp_map = {}
        class_ids = ids_msg.data
        poses = poses_msg.poses

        if len(class_ids) != len(poses):
            print(f"❌ MISMATCH: {len(class_ids)} IDs vs {len(poses)} poses - skipping")
            rospy.logwarn("Mismatched number of poses and class IDs received. Skipping update.")
            return

        detected_keys = []
        for i, class_id in enumerate(class_ids):
            if class_id < len(self.class_names):
                key_name = self.class_names[class_id]
                pose = poses[i]
                temp_map[key_name] = pose
                detected_keys.append(key_name)
                
                # Debug: Print pose details for important keys
                if key_name in self.command_sequence:
                    print(f"   🎯 KEY '{key_name}': x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
            else:
                print(f"   ⚠️  Invalid class ID: {class_id}")
        
        print(f"   🔤 Detected keys: {', '.join(detected_keys[:10])}{'...' if len(detected_keys) > 10 else ''}")
        
        # Check if we have all needed keys for our sequence
        missing_keys = [char for char in self.command_sequence if char not in temp_map]
        if missing_keys:
            print(f"   ⚠️  Missing keys for sequence: {missing_keys}")
        else:
            print(f"   ✅ All keys for '{self.command_sequence}' detected!")
        
        self.key_pose_map = temp_map
        if not self.pose_map_updated and self.key_pose_map:
            self.successful_detections += 1
            print(f"🗺️  KEYBOARD MAP CREATED: {len(self.key_pose_map)} keys detected")
            self.pose_map_updated = True
            self.print_sequence_availability()

    def print_sequence_availability(self):
        """Debug function to show which keys are available for the sequence."""
        print("\n📋 SEQUENCE ANALYSIS:")
        print(f"   Target: '{self.command_sequence}'")
        for i, char in enumerate(self.command_sequence):
            status = "✅" if char in self.key_pose_map else "❌"
            pose_info = ""
            if char in self.key_pose_map:
                pose = self.key_pose_map[char]
                pose_info = f" (x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f})"
            print(f"   {i+1}. '{char}' {status}{pose_info}")

    def find_key_pose(self, target_char):
        """Finds the pose for a specific character from the internal map with debug info."""
        print(f"\n🔍 SEARCHING for key '{target_char}'...")
        pose = self.key_pose_map.get(target_char)
        if pose is None:
            print(f"   ❌ Key '{target_char}' not found in map")
            print(f"   📋 Available keys: {list(self.key_pose_map.keys())}")
            rospy.logwarn(f"Key '{target_char}' not found in detected poses.")
        else:
            print(f"   ✅ Found key '{target_char}'")
            print(f"   📍 Position: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
            print(f"   🧭 Orientation: x={pose.orientation.x:.3f}, y={pose.orientation.y:.3f}, z={pose.orientation.z:.3f}, w={pose.orientation.w:.3f}")
            rospy.loginfo(f"Found pose for key '{target_char}'.")
        return pose

    def move_to_pose_safe(self, target_pose, description=""):
        """Safely plans and executes a movement to a target pose with extensive debugging."""
        print(f"\n🎯 PLANNING MOVEMENT {description}...")
        print(f"   📍 Target position: x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}")
        print(f"   🧭 Target orientation: x={target_pose.orientation.x:.3f}, y={target_pose.orientation.y:.3f}, z={target_pose.orientation.z:.3f}, w={target_pose.orientation.w:.3f}")
        
        # Debug: Show current pose
        try:
            current_pose = self.group.get_current_pose().pose
            print(f"   📍 Current position: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
            
            # Calculate distance
            dx = target_pose.position.x - current_pose.position.x
            dy = target_pose.position.y - current_pose.position.y
            dz = target_pose.position.z - current_pose.position.z
            distance = (dx**2 + dy**2 + dz**2)**0.5
            print(f"   📏 Distance to target: {distance:.3f}m")
        except Exception as e:
            print(f"   ⚠️  Could not get current pose: {e}")
        
        self.group.set_pose_target(target_pose)
        rospy.loginfo(f"Planning movement {description}...")
        
        print("   🧠 Computing trajectory...")
        plan = self.group.plan()
        
        # Handle different MoveIt versions
        if isinstance(plan, tuple):
            success, trajectory = plan[0], plan[1]
            planning_time = plan[2] if len(plan) > 2 else 0
            error_code = plan[3] if len(plan) > 3 else 0
            print(f"   ⏱️  Planning time: {planning_time:.2f}s")
            if len(plan) > 3:
                print(f"   🔢 Error code: {error_code}")
        else:
            success = len(plan.joint_trajectory.points) > 0
            trajectory = plan
            print(f"   📊 Trajectory points: {len(plan.joint_trajectory.points)}")

        if success:
            print(f"   ✅ Planning successful!")
            print(f"   🚀 Executing movement {description}...")
            execution_success = self.group.execute(trajectory, wait=True)
            print(f"   🏁 Execution result: {'✅ Success' if execution_success else '❌ Failed'}")
            
            self.group.stop()
            self.group.clear_pose_targets()
            
            # Verify final position
            try:
                final_pose = self.group.get_current_pose().pose
                final_dx = target_pose.position.x - final_pose.position.x
                final_dy = target_pose.position.y - final_pose.position.y
                final_dz = target_pose.position.z - final_pose.position.z
                final_distance = (final_dx**2 + final_dy**2 + final_dz**2)**0.5
                print(f"   📏 Final position error: {final_distance:.4f}m")
                if final_distance > 0.01:  # 1cm tolerance
                    print(f"   ⚠️  Large position error detected!")
            except Exception as e:
                print(f"   ⚠️  Could not verify final position: {e}")
            
            return execution_success
        else:
            print(f"   ❌ Planning failed for {description}")
            rospy.logerr(f"Failed to plan movement {description}")
            self.group.clear_pose_targets()
            return False

    def move_to_hover(self, key_pose):
        """Move to a hover position safely above the key."""
        print(f"\n🚁 PREPARING HOVER POSITION...")
        hover_pose = deepcopy(key_pose)
        hover_pose.position.z += 0.05  # 5 cm above key
        print(f"   ⬆️  Adding 5cm to z-coordinate: {key_pose.position.z:.3f} → {hover_pose.position.z:.3f}")
        
        # Point gripper downwards (Roll: 0, Pitch: 90, Yaw: 0)
        hover_pose.orientation.x = 0.0
        hover_pose.orientation.y = 0.7071
        hover_pose.orientation.z = 0.0
        hover_pose.orientation.w = 0.7071
        print(f"   🧭 Setting downward orientation (pitch=90°)")
        
        return self.move_to_pose_safe(hover_pose, f"to hover over key")

    def press_key(self, key_pose):
        """Perform the press and retract motion with detailed debugging."""
        print(f"\n⌨️  PREPARING KEY PRESS...")
        press_pose = deepcopy(key_pose)
        original_z = press_pose.position.z
        press_pose.position.z -= 0.005  # Move 5mm below detected surface
        print(f"   ⬇️  Adjusting z for press: {original_z:.3f} → {press_pose.position.z:.3f} (-5mm)")
        
        press_pose.orientation.x = 0.0
        press_pose.orientation.y = 0.7071
        press_pose.orientation.z = 0.0
        press_pose.orientation.w = 0.7071
        
        print("   📉 Moving down to press key...")
        if self.move_to_pose_safe(press_pose, "to press key"):
            print("   ⏸️  Holding press for 0.25s...")
            rospy.sleep(0.25)  # Brief pause for the press
            print("   📈 Moving back to hover...")
            return self.move_to_hover(key_pose) # Move back to the hover position
        else:
            print("   ❌ Failed to press key")
            return False

    def go_to_home_position(self):
        """Move to a safe, predefined home position with debug info."""
        print(f"\n🏠 MOVING TO HOME POSITION...")
        try:
            # Debug: Show available named targets
            try:
                named_targets = self.group.get_named_targets()
                print(f"   📋 Available named targets: {list(named_targets.keys())}")
            except Exception as e:
                print(f"   ⚠️  Could not get named targets: {e}")
            
            rospy.loginfo("Moving to home position using named target 'home'...")
            
            # Check if 'home' target exists
            if 'home' not in self.group.get_named_targets():
                print("   ⚠️  'home' target not found, trying alternative methods...")
                
                # Try some common alternative names
                alternatives = ['Home', 'HOME', 'ready', 'init', 'start']
                for alt in alternatives:
                    if alt in self.group.get_named_targets():
                        print(f"   🔄 Using alternative target: '{alt}'")
                        self.group.set_named_target(alt)
                        break
                else:
                    print("   ❌ No suitable home position found")
                    return False
            else:
                print("   ✅ Using 'home' target")
                self.group.set_named_target("home")
            
            print("   🚀 Executing movement to home...")
            success = self.group.go(wait=True)
            
            if success:
                print("   ✅ Successfully reached home position")
            else:
                print("   ❌ Failed to reach home position")
            
            self.group.stop()
            return success
            
        except Exception as e:
            print(f"   ❌ Exception during home movement: {e}")
            rospy.logerr(f"Failed to move to home position: {e}")
            return False

    def wait_for_keyboard_detection(self, timeout=30.0):
        """Waits until the keyboard pose map is populated with progress updates."""
        print(f"\n⏳ WAITING FOR KEYBOARD DETECTION (timeout: {timeout}s)...")
        start_time = rospy.Time.now()
        last_update = 0
        
        while not self.pose_map_updated and not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            
            # Progress updates every 5 seconds
            if int(elapsed) > last_update + 4:
                last_update = int(elapsed)
                print(f"   ⏰ Waiting... {elapsed:.0f}/{timeout:.0f}s (callbacks: {self.callback_count})")
                
                # Show some topics status
                if self.callback_count == 0:
                    print("   📡 No detection callbacks received yet")
                    print("   💡 Check if detection node is running: rostopic echo /keyboard_key_poses")
                else:
                    print(f"   📊 Received {self.callback_count} detection callbacks")
            
            if elapsed > timeout:
                print(f"   ⏰ TIMEOUT after {timeout}s")
                rospy.logerr("Timeout waiting for keyboard detection.")
                return False
            rospy.sleep(0.5)
        
        if self.pose_map_updated:
            print(f"   ✅ Keyboard detected after {elapsed:.1f}s!")
        return self.pose_map_updated

    def type_sequence(self):
        """Executes the full typing sequence for the given command with progress tracking."""
        print(f"\n⌨️  STARTING TYPING SEQUENCE: '{self.command_sequence}'")
        print("=" * 50)
        
        success_count = 0
        for i, char_to_type in enumerate(self.command_sequence):
            if rospy.is_shutdown(): 
                print("   🛑 ROS shutdown requested, stopping...")
                break
                
            print(f"\n📝 TYPING CHARACTER {i+1}/{len(self.command_sequence)}: '{char_to_type}'")
            print("-" * 40)
            
            key_pose = self.find_key_pose(char_to_type)
            if key_pose is None:
                print(f"   ❌ Cannot type '{char_to_type}', key not detected. Skipping.")
                rospy.logerr(f"Cannot type '{char_to_type}', key not detected. Skipping.")
                continue
            
            if not self.move_to_hover(key_pose):
                print(f"   ❌ Failed to move to hover for key '{char_to_type}'. Skipping.")
                rospy.logerr(f"Failed to move to hover for key '{char_to_type}'. Skipping.")
                continue
            
            if not self.press_key(key_pose):
                print(f"   ❌ Failed to press key '{char_to_type}'. Skipping.")
                rospy.logerr(f"Failed to press key '{char_to_type}'. Skipping.")
                continue
            
            success_count += 1
            print(f"   ✅ Successfully typed '{char_to_type}' ({success_count}/{len(self.command_sequence)})")
            rospy.loginfo(f"Successfully typed '{char_to_type}'.")
            
            print(f"   ⏸️  Pausing for 1s before next character...")
            rospy.sleep(1.0)
        
        print(f"\n🎉 TYPING SEQUENCE COMPLETE!")
        print(f"   📊 Success rate: {success_count}/{len(self.command_sequence)} ({100*success_count/len(self.command_sequence):.1f}%)")

    def print_system_status(self):
        """Print comprehensive system status for debugging."""
        print("\n🔍 SYSTEM STATUS CHECK:")
        print("-" * 40)
        
        # ROS topics
        print("📡 Key ROS Topics:")
        import subprocess
        try:
            result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True, timeout=5)
            topics = result.stdout.split('\n')
            key_topics = ['/keyboard_key_poses', '/keyboard_key_ids', '/joint_states', '/move_group']
            for topic in key_topics:
                status = "✅" if topic in topics else "❌"
                print(f"   {status} {topic}")
        except Exception as e:
            print(f"   ⚠️  Could not check topics: {e}")
        
        # MoveIt status
        print("\n🦾 MoveIt Status:")
        try:
            print(f"   📋 Planning group: {self.group.get_name()}")
            print(f"   🎯 End effector: {self.group.get_end_effector_link()}")
            print(f"   🔗 Planning frame: {self.group.get_planning_frame()}")
            print(f"   ⚡ Current pose valid: {self.group.get_current_pose() is not None}")
        except Exception as e:
            print(f"   ❌ MoveIt error: {e}")
        
        # Detection status
        print("\n👁️  Detection Status:")
        print(f"   📨 Callbacks received: {self.callback_count}")
        print(f"   🗺️  Pose map updated: {self.pose_map_updated}")
        print(f"   🔤 Keys detected: {len(self.key_pose_map)}")
        if self.key_pose_map:
            sample_keys = list(self.key_pose_map.keys())[:5]
            print(f"   📋 Sample keys: {sample_keys}")

    def run(self):
        """Main execution loop with comprehensive error handling and status reporting."""
        print("\n🚀 STARTING AUTONOMOUS TYPING SYSTEM")
        print("=" * 60)
        
        try:
            # System status check
            self.print_system_status()
            
            # Step 1: Go to home position
            print(f"\n📍 STEP 1: Moving to home position...")
            if not self.go_to_home_position():
                print("❌ Could not reach home position. Aborting.")
                rospy.logerr("Could not reach home position. Aborting.")
                return
            print("✅ Home position reached")
            
            # Step 2: Wait for keyboard detection
            print(f"\n👁️  STEP 2: Waiting for keyboard detection...")
            if not self.wait_for_keyboard_detection():
                print("❌ Could not detect keyboard. Aborting.")
                rospy.logerr("Could not detect keyboard. Aborting.")
                self.print_system_status()  # Debug info on failure
                return
            print("✅ Keyboard detected successfully")
            
            # Step 3: Execute typing sequence
            print(f"\n⌨️  STEP 3: Executing typing sequence...")
            self.type_sequence()
            print("✅ Typing sequence completed")
            
            # Step 4: Return home
            print(f"\n🏠 STEP 4: Returning to home position...")
            if self.go_to_home_position():
                print("✅ Returned to home position")
            else:
                print("⚠️  Could not return to home position")
            
            print(f"\n🎉 MISSION ACCOMPLISHED!")
            print("=" * 60)
            
        except KeyboardInterrupt:
            print(f"\n🛑 USER INTERRUPT - Stopping gracefully...")
            self.group.stop()
            self.group.clear_pose_targets()
        except Exception as e:
            print(f"\n❌ CRITICAL ERROR during execution: {e}")
            rospy.logerr(f"An error occurred during main execution: {e}")
            self.print_system_status()  # Debug info on failure
        finally:
            print(f"\n🔄 SHUTTING DOWN...")
            try:
                moveit_commander.roscpp_shutdown()
                print("✅ MoveIt shutdown complete")
            except:
                print("⚠️  MoveIt shutdown had issues")
            rospy.loginfo("🔚 Autonomous typing node shut down.")
            print("👋 Goodbye!")

if __name__ == '__main__':
    try:
        AutonomousTyping().run()
    except rospy.ROSInterruptException:
        print("🛑 ROS interrupted")
        pass