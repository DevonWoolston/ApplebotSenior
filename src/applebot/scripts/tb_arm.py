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

class TBArmNode:
    def __init__(self):
        rospy.init_node('tb_arm_node')
        rospy.loginfo("<ARM NODE> Node online...")

        # Initialize variables to track statuses
        self.arm_status = "resting"
        self.base_status = "home"
        self.gripper_status = "open"

        # Set up subscribers and publisher
        self.arm_status_pub = rospy.Publisher('/tb_arm_status', String, queue_size=10)
        rospy.Subscriber('/tb_base_status', String, self.base_status_callback)
        rospy.Subscriber('/tb_arm_status', String, self.arm_status_callback)
        rospy.Subscriber('/tb_gripper_status', String, self.gripper_status_callback)
        rospy.Subscriber('/apple_coords', Float32MultiArray, self.apple_coords_callback)

        # Set up client to move arm
        self.move_group = MoveGroupCommander("arm")

    # New message was published to "/base_status"
    def base_status_callback(self, msg):
        self.base_status = msg.data
        rospy.loginfo(f"<ARM NODE> Received new base status: {self.base_status}")
        if (self.base_status == "tree"):
            self.move_arm_to_apple()
        elif (self.base_status == "basket"):
            self.move_arm_to_basket()

    # New message was published to "/arm_status"
    def arm_status_callback(self, msg):
        self.arm_status = msg.data
        rospy.loginfo(f"<ARM NODE> Received new arm status: {self.arm_status}")

    # New message was published to "/gripper_status"
    def gripper_status_callback(self, msg):
        self.gripper_status = msg.data  # Update gripper status
        rospy.loginfo(f"<ARM NODE> Received new gripper status: {self.gripper_status}")
        self.move_arm_to_resting()
        rospy.sleep(3)

    # New message was published to "/apple_coords"
    def apple_coords_callback(self, msg):
        self.apple_coord = msg.data
        rospy.loginfo("<ARM NODE> Received new apple coordinates: %s", self.apple_coord)

        self.goal_x = self.apple_coord[0]
        self.goal_y = self.apple_coord[1]
        self.goal_z = self.apple_coord[2]   

    # Executes a motion plan for the arm to reach the apple
    def move_arm_to_apple(self):
        rospy.loginfo("<ARM NODE> Calculating apple coordinates wrt robot...")
        amcl_pose_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        position = amcl_pose_msg.pose.pose.position
        orientation = amcl_pose_msg.pose.pose.orientation 
        self.robot_w = orientation.w
        self.robot_z = orientation.z
        self.robot_yaw = 1.57 - self.quaternion_to_yaw(self.robot_w, self.robot_z)
        error_x = position.x - self.goal_x
        error_y = self.goal_y - position.y

        arm_x = error_y - 0.15
        #arm_x = math.cos(self.robot_yaw) * math.sqrt(((error_x)**2) + ((error_y)**2)) - 0.15

        arm_y = error_x
        #arm_y = error_x + (math.sin(self.robot_yaw) * (math.sqrt(((error_x)**2) + ((error_y)**2))+.02))
        arm_z = self.goal_z 

        if arm_x > 0.25:
            arm_x = 0.25
        
        rospy.loginfo("------------------------------------------------")
        rospy.loginfo("Errors: ")
        rospy.loginfo(error_x)
        rospy.loginfo(error_y)
        rospy.loginfo("------------------------------------------------")

        rospy.loginfo("Robot position: ")
        rospy.loginfo(position)
        rospy.loginfo("Robot orientation: ")
        rospy.loginfo(orientation)
        rospy.loginfo(self.quaternion_to_yaw(self.robot_w, self.robot_z))
        rospy.loginfo("------------------------------------------------")
        rospy.loginfo("<ARM NODE> Moving arm to apple position x, y, z:")
        rospy.loginfo(arm_x)
        rospy.loginfo(arm_y)
        rospy.loginfo(arm_z)
        rospy.loginfo("------------------------------------------------")
        self.move_arm_to(arm_x, arm_y, arm_z)
        # self.move_arm_to(self.goal_x, self.goal_y, self.goal_z)
        rospy.sleep(6)
        self.arm_status = "reaching"
        self.arm_status_pub.publish(self.arm_status)
        rospy.loginfo(f"<ARM NODE> Published arm status: {self.arm_status}")

    # Executes a motion plan for the arm to reach the basket
    def move_arm_to_basket(self):
        rospy.loginfo("<ARM NODE> Moving arm to basket position...")
        self.move_arm_to(0.23, 0.0, 0.26)
        rospy.sleep(5)
        self.arm_status = "basket"
        self.arm_status_pub.publish(self.arm_status)
        rospy.loginfo(f"<ARM NODE> Published arm status: {self.arm_status}")

    # Executes a motion plan for the arm to go to resting position
    def move_arm_to_resting(self):
        rospy.loginfo("<ARM NODE> Moving arm to resting position...")
        self.move_arm_to(-0.03, 0.0, 0.28)
        rospy.sleep(3)
        self.arm_status = "resting"
        self.arm_status_pub.publish(self.arm_status)
        rospy.loginfo(f"<ARM NODE> Published arm status: {self.arm_status}")

    # Executes a motion plan for the arm to go to any position
    def move_arm_to(self, x, y, z):
        # Set position target
        self.move_group.set_position_target([x, y, z])

        # Plan
        my_plan = self.move_group.plan()
        success = my_plan[0]
        if success:
            # Execute the plan
            self.move_group.go(wait=True)
            # Stop any residual movement
            self.move_group.stop()

            # Clear targets after movement
            self.move_group.clear_pose_targets()

            return True
        else:
            return False

    # Calculates yaw angle (rad) from quartnerion values
    def quaternion_to_yaw(self, w, z):
        return (2 * math.atan2(z, w))
    

def main():
    try:
        node = TBArmNode()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
