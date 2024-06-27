#!/usr/bin/python3

import sys
import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import JointTrajectoryPoint  # Add this import

class ArmControlNode:
    def __init__(self):
        #rospy.init_node('arm_control_node', anonymous=True)

        # Create an action client for the gripper trajectory
        self.gripper_client = actionlib.SimpleActionClient(
            '/gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )

        # Wait for the action server to come up
        self.gripper_client.wait_for_server()

        # Repeat the process indefinitely
        while not rospy.is_shutdown():
            # Move the gripper to the initial position
            self.set_gripper_position([-0.1])

            # Wait for 5 seconds
            rospy.sleep(5)

            # Move the gripper to a different position
            self.set_gripper_position([0.005])  # Corrected gripper position

            # Wait for 5 seconds
            rospy.sleep(5)

    def set_gripper_position(self, joint_values):
        # Print the current gripper position
        rospy.loginfo(f"Setting gripper position to: {joint_values}")

        # Create a goal for the gripper trajectory action
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["gripper"]  # Replace with your gripper joint name
        goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=joint_values,
                time_from_start=rospy.Duration(5.0)  # Adjust duration as needed
            )
        )

        # Send the goal to the action server
        self.gripper_client.send_goal(goal)

        # Wait for the action to complete
        self.gripper_client.wait_for_result()

if __name__ == '__main__':
    try:
        arm_control_node = ArmControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
