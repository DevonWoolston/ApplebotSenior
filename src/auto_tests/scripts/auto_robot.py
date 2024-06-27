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
import math


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
        #while not rospy.is_shutdown():
        # Move the gripper to the initial position
        self.set_gripper_position([-0.1])

        # Wait for 5 seconds
        rospy.sleep(3)

        # Move the gripper to a different position
        self.set_gripper_position([0.002])  # Corrected gripper position

        # Wait for 5 seconds
        rospy.sleep(3)

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




def check_base_current_pose():
    rospy.loginfo("I am in check_base_current_pose")

    # Wait for a message to be received on the /amcl_pose topic
    rospy.loginfo("Waiting for message on /amcl_pose topic...")
    pose_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
    rospy.loginfo("Received TurtleBot location from /amcl_pose topic.")

     # Extract the position information from the received message
    position = pose_msg.pose.pose.position
    x = position.x
    y = position.y

    orientation = pose_msg.pose.pose.orientation
    w = orientation.w
    z = orientation.z

    # Home base position
    home_position = [0, 0, 1.0, 0.0]  # x, y, w, z

    # Basket location
    tree_position = [1.0, 0.55, 1.0, 1.0]  # x, y, w, z

    # Tolerance for position comparison
    tolerance = 0.2

    # Compare the robot's position with home and basket positions
    if abs(x - home_position[0]) < tolerance and abs(y - home_position[1]) < tolerance:
        rospy.loginfo("Robot is at home base.")

        #new goal position: Tree location 
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = 'map'  # Assuming the goal is in the map frame
        goal_msg.target_pose.pose.position.x = 1.0  # Example new position
        goal_msg.target_pose.pose.position.y = 0.55
        goal_msg.target_pose.pose.orientation.w = 1.0  # Quaternion representing no rotation
        goal_msg.target_pose.pose.orientation.z = 1.0

        rospy.loginfo("Sending tree goal from check_position_callback...")
        client_base.send_goal(goal_msg, position_reached_callback)
        rospy.wait_for_message('/reached_new_position', String)

        rospy.sleep(3)
        move_arm_to(0.1, 0.1, 0.35)
        rospy.sleep(6)
        gripper_node = ArmControlNode()
        move_arm_to(0.1, 0.0, 0.25)
        rospy.sleep(3)
        check_base_current_pose()


    elif abs(x - tree_position[0]) < tolerance and abs(y - tree_position[1]) < tolerance:
        rospy.loginfo("Robot is at the tree location.")

        #new goal position: Basket location 
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = 'map'  # Assuming the goal is in the map frame
        goal_msg.target_pose.pose.position.x = 0.0  # Example new position
        goal_msg.target_pose.pose.position.y = 0.5
        goal_msg.target_pose.pose.orientation.w = 0.0  # Quaternion representing no rotation
        goal_msg.target_pose.pose.orientation.z = 1.0

        rospy.loginfo("Sending basket goal from check_position_callback...")
        client_base.send_goal(goal_msg, position_reached_callback)
        rospy.wait_for_message('/reached_new_position', String)
        rospy.sleep(1)
        move_arm_to(0.1, 0.1, 0.35)
        rospy.sleep(6)
        gripper_node = ArmControlNode()
        move_arm_to(0.1, 0.0, 0.25)
        rospy.sleep(3)
        check_base_current_pose()


    else:
        rospy.loginfo("Robot is at an basket location.")
        #new goal position: Home location 
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = 'map'  # Assuming the goal is in the map frame
        goal_msg.target_pose.pose.position.x = 0.0  # Example new position
        goal_msg.target_pose.pose.position.y = 0.0
        goal_msg.target_pose.pose.orientation.w = 1.0  # Quaternion representing no rotation
        goal_msg.target_pose.pose.orientation.z = 0.0

        rospy.loginfo("Sending home goal from check_position_callback...")
        client_base.send_goal(goal_msg, position_reached_callback)
        rospy.wait_for_message('/reached_new_position', String)
        rospy.sleep(1)
        check_base_current_pose()
    



def check_arm_current_position():
    #home position
    arm_home = [0.1, 0.0, 0.25] #x, y, z
    #tolerance to see where it is
    tolerance = 0.05

    current_arm_position = move_group.get_current_pose().pose

    # Extract x, y, z coordinates from the current arm position
    current_x = current_arm_position.position.x
    current_y = current_arm_position.position.y
    current_z = current_arm_position.position.z

    # Compare the current arm position with the home position
    if (abs(current_x - arm_home[0]) < tolerance and
        abs(current_y - arm_home[1]) < tolerance and
        abs(current_z - arm_home[2]) < tolerance):
        rospy.loginfo("Arm is at home position.")
        move_arm_to(0.1, 0.1, 0.35)
    else:
        rospy.loginfo("Arm is at an unknown location.")
        move_arm_to(arm_home[0], arm_home[1], arm_home[2])
    



def move_arm_to(x, y, z):
    current_pose = move_group.get_current_pose().pose

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



def position_reached_callback(status, result):
    client_base.cancel_goals_at_and_before_time(rospy.Time.now())
    # This function will be called when the goal is reached
    if status == GoalStatus.SUCCEEDED:
        rospy.loginfo("Base goal reached!")
        rospy.Publisher('/reached_new_position', String, queue_size=10).publish("new position")

        #check_arm_current_position()
        
        

def main():
    rospy.init_node('auto_robot_node', anonymous=True)
    rospy.Publisher('/reached_new_position', String, queue_size=10).publish("initial Position == Home")


    # global gripper_node 
    # gripper_node = ArmControlNode()


    # Initialize move group
    global move_group
    move_group = MoveGroupCommander("arm")

    # Create a SimpleActionClient for move_base

    rospy.loginfo("before client")

    global client_base
    client_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client_base.wait_for_server()


    # Subscribe to move_group result topic
    #rospy.Subscriber('/move_group/result', MoveGroupActionResult, arm_position_reached_callback)

    
    check_base_current_pose()
    


    rospy.spin()  # Keep the node running

if __name__ == '__main__':
    main()



