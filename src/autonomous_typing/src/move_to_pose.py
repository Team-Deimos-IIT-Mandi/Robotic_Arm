#!/usr/bin/env python3
import rospy
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose

rospy.init_node("move_to_pose_cli")
group = MoveGroupCommander("body")

pose = Pose()
pose.position.x = 0.0561
pose.position.y = -0.5375
pose.position.z = 0.3167
pose.orientation.w = 1.0  # Keep orientation simple

group.set_pose_target(pose)
success = group.go(wait=True)
group.stop()
group.clear_pose_targets()

if success:
    print("✅ Successfully moved to target pose.")
else:
    print("❌ Failed to move to target pose.")