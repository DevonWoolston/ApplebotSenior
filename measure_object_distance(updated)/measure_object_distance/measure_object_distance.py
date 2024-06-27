#!/usr/bin/env python3

import rospy
import cv2
from realsense_camera import RealsenseCamera
from mask_rcnn import MaskRCNN
from std_msgs.msg import Bool, Float32MultiArray

class AppleDetectionNode:
    def __init__(self):

        # Initialize ROS node
        rospy.init_node('tb_vision_node', anonymous=True)
        rospy.loginfo("<VISION NODE> Node online...")

        # Initiliaze publisher for apple coordinates (m)
        self.pub = rospy.Publisher('/apple_coords', Float32MultiArray, queue_size=10)

        # Load Realsense camera and Mask R-CNN
        self.rs = RealsenseCamera()
        self.mrcnn = MaskRCNN()

        # If anything is published to "/start_harvesting" start running detection code
        rospy.Subscriber('/start_harvesting', Bool, self.start_harvesting_callback)

        # Offsets from the camera's origin to the robot's origin (m)
        self.x_offset = -0.18 #.24
        self.y_offset = -0.08 #.08
        self.z_offset = 0.33 #34.2

    # Runs whenever a message is published to "/start_harvesting"
    def start_harvesting_callback(self, msg):

        # Keep checking environment until an apple is detected
        rospy.loginfo("<VISION NODE> Starting vision processing.")
        appleFound = False
        while(not appleFound):

            # Get image frame from the camera
            ret, bgr_frame, depth_frame = self.rs.get_frame_stream()
            rospy.loginfo("<VISION NODE> Captured image frame.")

            if not ret:
                rospy.logerr("<VISION NODE> Failed to capture image frame.")
                return

            # Get object info
            boxes, classes, contours, centers = self.mrcnn.detect_objects_mask(bgr_frame)
            rospy.loginfo(f"<VISION NODE> Detected {len(boxes)} objects.")
            bgr_frame = self.mrcnn.draw_object_mask(bgr_frame)
            self.mrcnn.draw_object_info(bgr_frame, depth_frame)
            cv2.imshow("BGR Frame", bgr_frame)

            cv2.waitKey(0)

            cv2.destroyAllWindows()

            # Iterate through detected objects
            for box, class_id, center in zip(boxes, classes, centers):
                rospy.loginfo(f"<VISION NODE> Object detected with class_id: {class_id}")

                # Check if the detected object belongs to the class of interest (e.g., apple)
                if class_id == 52:

                    # Extract pixel coordinates of the object's center
                    x, y = int(center[0]), int(center[1])
                    
                    # Get depth value at the object's center
                    depth = depth_frame[y, x]
                    
                    # Convert pixel coordinates and depth value to physical coordinates wrt the Camera (mm)
                    # x (mm): Right of the camera (+), left of the camera (-)
                    # y (mm): Below the camera (+), above the camera (-)
                    # depth (mm): Distance in front of camera lens (+)
                    phys_coords = self.rs.convert_depth_to_phys_coord_using_realsense(x, y, depth)
                    
                    # Convert coordinates to (m)
                    phys_coords_divided = tuple(coord / 1000 for coord in phys_coords)
                    rospy.loginfo("<VISION NODE> Apple coords wrt CAMERA (x, y, depth)") 
                    rospy.loginfo(phys_coords_divided[0]) 
                    rospy.loginfo(phys_coords_divided[1]) 
                    rospy.loginfo(phys_coords_divided[2])

                    # Transformation from camera reference frame to odom origin reference frame
                    new_y = self.y_offset - phys_coords_divided[0] 
                    new_x = self.x_offset + phys_coords_divided[2]
                    new_z = self.z_offset - phys_coords_divided[1]
                    phys_coords_robot_frame = tuple([new_x, new_y, new_z])       
                    rospy.loginfo("<VISION NODE> Apple coords wrt ROBOT (x, y, z)")
                    rospy.loginfo(phys_coords_robot_frame[0])
                    rospy.loginfo(phys_coords_robot_frame[1])
                    rospy.loginfo(phys_coords_robot_frame[2])
                    
                    # Publish the coordinates to the "/apple_coords" topic
                    coords_msg = Float32MultiArray(data=phys_coords_robot_frame)
                    self.pub.publish(coords_msg)
                    rospy.loginfo("<VISION NODE> Published coordinates.")

                    appleFound = True
                    break

            if not appleFound:
                rospy.loginfo("<VISION NODE> Apple not found. Retrying...")

    def run(self):
        try:
            # Keep the node running
            rospy.spin()
        finally:
            # Clean up
            self.rs.release()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = AppleDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")
