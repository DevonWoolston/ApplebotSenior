#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import MoveGroupActionResult
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import JointTrajectoryPoint  # Add this import
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import math

class TBBaseNode:
    def __init__(self):

        # Initialize ROS Node
        rospy.init_node('tb_base_node')
        rospy.loginfo("<BASE NODE> Node online...")

        # Initialize variables to track statuses
        self.arm_status = "resting"
        self.base_status = "home"
        self.gripper_status = "open"

        # Set up subscribers and publisher
        self.base_pub = rospy.Publisher('/tb_base_status', String, queue_size=10)
        rospy.Subscriber('/tb_arm_status', String, self.arm_status_callback)
        rospy.Subscriber('/tb_gripper_status', String, self.gripper_status_callback)
        rospy.Subscriber('/apple_coords', Float32MultiArray, self.apple_coords_callback)
        
        # Set up client to move base
        self.client_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client_base.wait_for_server()

    # Base finished its motion plan execution
    def base_goal_reached(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("<BASE NODE> Goal succeeded!")
            next_status = ""
            if self.base_status == "home":
                next_status = "tree"
            elif self.base_status == "tree":
                next_status = "basket"
            elif self.base_status == "basket":
                next_status = "home"
            self.base_status = next_status
            self.base_pub.publish(next_status)
        else:
            rospy.logwarn("<BASE NODE> Goal failed with status: %d", status)


    # New message was published to "/arm_status"
    def arm_status_callback(self, msg):
        self.arm_status = msg.data  # Update arm status
        rospy.loginfo("<BASE NODE> Received new arm status: %s", self.arm_status)
        
        if self.arm_status == "resting":
            if self.base_status == "tree":
                rospy.loginfo("<BASE NODE> Moving to basket...")
                self.send_goal(-0.13, 0.64, 0.38, 0.92) # BASKET LOCATION
            elif self.base_status == "basket":
                rospy.loginfo("<BASE NODE> Moving to home...")
                self.send_goal(0.0, 0.0, 1.0, 0.0) # HOME LOCATION

    # New message was published to "/gripper_status"
    def gripper_status_callback(self, msg):
        self.gripper_status = msg.data  # Update gripper status
        rospy.loginfo("<BASE NODE> Received new gripper status: %s", self.gripper_status)

    # New message was published to "/apple_coords"
    def apple_coords_callback(self, msg):
        self.apple_coords = msg.data
        rospy.loginfo("<BASE NODE> Received new apple coordinates: %s", self.apple_coords)
        goal_x = self.apple_coords[0]
        goal_y = self.apple_coords[1] - 0.35
        goal_w = 1.0
        goal_z = 1.0
        rospy.loginfo("<BASE NODE> Moving to apple tree...")
        self.send_goal(goal_x, goal_y, goal_w, goal_z)

    # Creates and executes a motion plan
    def send_goal(self, x, y, w, z):
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = 'map'  # Assuming the goal is in the map frame
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y
        goal_msg.target_pose.pose.orientation.w = w
        goal_msg.target_pose.pose.orientation.z = z
        rospy.loginfo("<BASE NODE> Goal coordinates: x=%s, y=%s, w=%s, z=%s", x, y, w, z)
        self.client_base.send_goal(goal_msg, self.base_goal_reached)

def main():
    try:
        node = TBBaseNode()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()