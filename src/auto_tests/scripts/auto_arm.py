#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander

def move_arm_to(x, y, z):
    current_pose = move_group.get_current_pose().pose

    # if (abs(current_pose.position.x - x ) < 0.6 or )
    # Set position target
    move_group.set_position_target([x, y, z])

    # Plan
    my_plan = move_group.plan()
    success = my_plan[0]
    if success:
        # Execute the plan
        move_group.go()
        return True
    else:
        return False

# if __name__ == "__main__":
#     # Initialize ROS node
#     rospy.init_node('arm_testing', anonymous=True)
#     # Initialize move group
#     move_group = MoveGroupCommander("arm")
#     move_arm_to(0.15, 0.15, 0.25)