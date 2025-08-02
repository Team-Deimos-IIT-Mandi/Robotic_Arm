#!/usr/bin/env python3

import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseArray, Pose

class AutonomousTyping:
    def __init__(self):
        rospy.init_node('autonomous_typing', anonymous=True)
        
        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("body")
        
        # Subscribe to detected key positions
        rospy.Subscriber("/keyboard_key_poses", PoseArray, self.keyboard_callback)
        
        # Command sequence to type
        self.command_sequence = "hello"
        self.current_index = 0
        self.keyboard_keys = None
        self.class_names = [
            'accent', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'minus', 'plus', 'del', 'tab', 'q', 'w', 'e', 'r', 't', 
            'y', 'u', 'i', 'o', 'p', '[', ']', 'enter', 'caps', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ':', 
            '"', '\\', 'shift-left', 'less', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', 'shift-right', 
            'ctrl-left', 'alt-left', 'space', 'alt-right', 'ctrl-right', 'keyboard'
        ]

    def keyboard_callback(self, msg):
        """Update detected keyboard key positions."""
        try:
            self.keyboard_keys = msg.poses
            rospy.logdebug("Updated key positions")
        except Exception as e:
            rospy.logerr(f"Error updating key positions: {e}")

    def move_to_hover(self, key_pose):
        """Move to hover position above the key."""
        hover_pose = Pose()
        hover_pose.position.x = key_pose.position.x
        hover_pose.position.y = key_pose.position.y
        hover_pose.position.z = key_pose.position.z + 0.02  # 2 cm above key
        self.group.set_pose_target(hover_pose)
        self.group.go(wait=True)
        rospy.sleep(1)

    def press_key(self, key_pose):
        """Perform key press movement."""
        press_pose = Pose()
        press_pose.position.x = key_pose.position.x
        press_pose.position.y = key_pose.position.y
        press_pose.position.z = key_pose.position.z  # Move down to key
        self.group.set_pose_target(press_pose)
        self.group.go(wait=True)
        rospy.sleep(1)
        self.move_to_hover(key_pose)  # Return to hover

    def type_sequence(self):
        """Execute typing sequence."""
        while self.current_index < len(self.command_sequence):
            key_char = self.command_sequence[self.current_index]
            key_pose = self.keyboard_keys.get(self.current_index)
            
            if key_pose:
                rospy.loginfo(f"Pressing key: {key_char}")
                self.move_to_hover(key_pose)
                self.press_key(key_pose)
                self.current_index += 1
            else:
                rospy.logwarn(f"Key {key_char} not found, retrying...")
                rospy.sleep(1)
        
        rospy.loginfo("Typing complete!")
    
    def run(self):
        """Main execution loop."""
        rospy.sleep(2)  # Allow time for key positions to update
        self.type_sequence()

if __name__ == '__main__':
    try:
        node = AutonomousTyping()
        node.run()
    except rospy.ROSInterruptException:
        pass
