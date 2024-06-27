# !/usr/bin/python3

import sys
import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64
import signal


class ArmControlNode:
    def __init__(self):
        rospy.init_node('arm_control_node', anonymous=True)

        # Create an action client for the gripper trajectory
        self.gripper_client = actionlib.SimpleActionClient(
            '/gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )

        # Wait for the action server to come up
        self.gripper_client.wait_for_server()

        # Subscribe to force sensor topics
        self.force_sensor_values = [0.0, 0.0, 0.0, 0.0]  # Initialize force sensor values
        rospy.Subscriber('/force_0', Float64, lambda msg: self.force_sensor_callback(msg, 0))
        rospy.Subscriber('/force_1', Float64, lambda msg: self.force_sensor_callback(msg, 1))
        rospy.Subscriber('/force_2', Float64, lambda msg: self.force_sensor_callback(msg, 2))
        rospy.Subscriber('/force_3', Float64, lambda msg: self.force_sensor_callback(msg, 3))

        # Set up signal handler for KeyboardInterrupt
        signal.signal(signal.SIGINT, self.signal_handler)

        # Set the initial gripper position
        self.set_gripper_position([-0.1])
        rospy.sleep(5)  # Adjust sleep duration as needed

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

    def force_sensor_callback(self, data, sensor_index):
        # Callback function for force sensor values
        self.force_sensor_values[sensor_index] = data.data

    def signal_handler(self, sig, frame):
        # Signal handler for KeyboardInterrupt
        rospy.loginfo("Received SIGINT signal. Shutting down...")
        sys.exit(0)

    def gripper_control_loop(self):
        # Check force sensor values at the beginning
        if any(value > 4.0 for value in self.force_sensor_values):
            rospy.loginfo("Force sensors exceeded 4.0 at the beginning. Exiting gripper control loop.")
            return

        # Repeat the process indefinitely
        while not rospy.is_shutdown():
            # First inner loop: Gripper closes until any sensor exceeds 1.0
            for position in range(1, 4096):
                gripper_position = position / 4095.0  # Scale position to [0, 1]
                self.set_gripper_position([gripper_position])

                # Print force sensor values
                rospy.loginfo(f"Force sensor values: {self.force_sensor_values}")

                rospy.sleep(0.01)  # Adjust sleep duration as needed

                # Check force sensor values during closing
                if any(value > 1.0 for value in self.force_sensor_values):
                    rospy.loginfo("Force sensors exceeded 1.0. Gripper stopped closing.")
                    break

                # Second inner loop: Gripper closes further until any sensor exceeds 4.5
            for position in range(1, 4096):
                gripper_position = position / 4095.0  # Scale position to [0, 1]
                self.set_gripper_position([gripper_position])

                # Print force sensor values
                rospy.loginfo(f"Force sensor values: {self.force_sensor_values}")

                rospy.sleep(0.05)  # Adjust sleep duration as needed

                # Check force sensor values during closing
                if any(value > 4.5 for value in self.force_sensor_values):
                    rospy.loginfo("Force sensors exceeded 4.5. Exiting gripper control loop.")
                    return

            # Wait for 5 seconds before repeating the loop
            rospy.sleep(5)


if __name__ == '__main__':
    try:
        arm_control_node = ArmControlNode()
        arm_control_node.gripper_control_loop()  # Start gripper control loop
        rospy.spin()
    except rospy.ROSInterruptException:
        pass