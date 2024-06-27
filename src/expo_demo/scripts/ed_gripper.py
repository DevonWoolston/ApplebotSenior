#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String, Float64
import signal

class TBGripperNode:
    def __init__(self):
        rospy.init_node('tb_gripper_node')
        rospy.loginfo("<GRIPPER NODE> Node online...")

        # Create an action client for the gripper trajectory
        self.gripper_client = actionlib.SimpleActionClient(
            '/gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )

        # Wait for the action server to come up
        self.gripper_client.wait_for_server()

        # Subscribe to force sensor topics
        self.force_sensor_values = [0.0, 0.0, 0.0, 0.0]  # Initialize force sensor values
        rospy.Subscriber('/force_1', Float64, lambda msg: self.force_sensor_callback(msg, 0))
        rospy.Subscriber('/force_2', Float64, lambda msg: self.force_sensor_callback(msg, 1))
        rospy.Subscriber('/force_3', Float64, lambda msg: self.force_sensor_callback(msg, 2))
        rospy.Subscriber('/force_4', Float64, lambda msg: self.force_sensor_callback(msg, 3))
        
        # Set the initial gripper position
        rospy.loginfo("<GRIPPER NODE> Setting initial gripper position.")
        self.set_gripper_position([-0.1])
        rospy.sleep(5)  # Adjust sleep duration as needed

        # Initialize variables to track statuses
        self.arm_status = "resting"
        self.base_status = "tree"
        self.gripper_status = "open"

        # Set up subscibers and publisher
        self.gripper_status_pub = rospy.Publisher('/tb_gripper_status', String, queue_size=10)
        rospy.Subscriber('/tb_arm_status', String, self.arm_status_callback)
        rospy.Subscriber('/tb_gripper_status', String, self.gripper_status_callback)
        rospy.Subscriber('/tb_base_status', String, self.base_status_callback)

    # Control the gripper by giving it a position
    def set_gripper_position(self, joint_values):
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

    # Close the gripper
    def move_gripper_to_closed(self):
        gripper_position = -0.1
        max_gripper_position = 0.5
        max_force = 4.3
        step = 1
        steps_per_revolution = 256
        increment = 1.0 / steps_per_revolution

        while gripper_position < max_gripper_position and not any(value > max_force for value in self.force_sensor_values):
            gripper_position += step * increment
            self.set_gripper_position([gripper_position])
            rospy.loginfo(f"<GRIPPER NODE> Force values: {self.force_sensor_values}")
            rospy.sleep(0.1)
        
        # Wait for the action to complete before publishing the status
        self.gripper_client.wait_for_result()
        self.gripper_status_pub.publish("closed")
        
    # Open the gripper
    def move_gripper_to_open(self):
        rospy.loginfo("Moving gripper to open")
        self.set_gripper_position([-0.1])
        
        # Wait for the action to complete before publishing the status
        rospy.sleep(2)
        self.gripper_client.wait_for_result()
        self.gripper_status_pub.publish("open")

    # New message was published to "/force_sensor"
    def force_sensor_callback(self, data, sensor_index):
        # Callback function for force sensor values
        self.force_sensor_values[sensor_index] = data.data

    # New message was published to "/arm_status"
    def arm_status_callback(self, msg):
        self.prev_arm_status = self.arm_status
        self.arm_status = msg.data
        rospy.loginfo(self.arm_status)
        rospy.loginfo(self.base_status)
        if self.arm_status == "reaching" and self.base_status == "tree":
            rospy.loginfo("Moving gripper to grasp")
            self.move_gripper_to_closed()
        elif self.arm_status == "basket":
            self.move_gripper_to_open()

    # New message was published to "/base_status"
    def base_status_callback(self, msg):
        self.base_status = msg.data

    # New message was published to "/gripper_status"
    def gripper_status_callback(self, msg):
        self.gripper_status = msg.data


if __name__ == '__main__':
    try:
        arm_control_node = TBGripperNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass