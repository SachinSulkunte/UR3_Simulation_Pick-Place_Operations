#!/usr/bin/env python

import sys
import copy
import time
import rospy
import rospkg

# messages for student to use
from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_driver.msg import gripper_input

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from find_block_pos import *
from kinematic_functions import *

########## Predefined functions below, no changes needed ##########

# UR3 home: position for UR3 not blocking the camera
home = [np.radians(255), np.radians(-45), np.radians(45), np.radians(-90), np.radians(-90), np.radians(90)]

SPIN_RATE = 20
current_position = copy.deepcopy(home)
thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
digital_in_0 = 0
analog_in_0 = 0.0
suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

def input_callback(msg):
    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 

def position_callback(msg):
    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True

def gripper(pub_cmd, loop_rate, io_0):
    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

def move_arm(pub_cmd, loop_rate, dest, vel, accel):
    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

class ImageConverter:
    def __init__(self, SPIN_RATE):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera1/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")

    def image_callback(self, data):
        try:
          # Convert ROS image to OpenCV image
            self.raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

########## Predefined functions above, no changes needed ##########


def MoveBlock(pub_cmd, loop_rate, start_xy, target_xy, vel, accel):

    # ToDo: this function moves one block from the starting (x, y) coord
    #       to destination (x, y) coord
    # 
    # Hint: - use 'inverse_kinematics()', 'move_arm()', 'gripper()' functions 
    #       - UR3 should move upward first after gripping the block in order to have
    #       a clear space to move around
    #
    # Note: - 'inverse_kinematics()' requires (x, y, z, yaw_angle), (x, y) is
    #       given by inputs, yaw is set as 0 in this lab, so proper 'z' values should
    #       be defined.
    #       - height of base plate is 0.01 meters, and height of the block is 0.0318 meters

    yaw_gripper = 0

    ##### Your Code Starts Here #####
	
    z_lower = 0.0318
    z_upper = 0.0636

    orientation_init_upper = inverse_kinematics(start_xy[0], start_xy[1], z_upper, yaw_gripper)
    orientation_init_lower = inverse_kinematics(start_xy[0], start_xy[1], z_lower, yaw_gripper)

    orientation_final_upper = inverse_kinematics(target_xy[0], target_xy[1], z_upper, yaw_gripper)
    orientation_final_lower = inverse_kinematics(target_xy[0], target_xy[1], z_lower, yaw_gripper)

    move_arm(pub_cmd, loop_rate, orientation_init_upper, vel, accel)
    move_arm(pub_cmd, loop_rate, orientation_init_lower, vel, accel)
    gripper(pub_cmd, loop_rate, suction_on)
    move_arm(pub_cmd, loop_rate, orientation_init_upper, vel, accel)

    move_arm(pub_cmd, loop_rate, orientation_final_upper, vel, accel)
    move_arm(pub_cmd, loop_rate, orientation_final_lower, vel, accel)
    gripper(pub_cmd, loop_rate, suction_off)
    move_arm(pub_cmd, loop_rate, orientation_final_upper, vel, accel)

	
	##### Your Code Ends Here #####

    print("Move one block.")
    

def main():
    global home

    # Initialize ROS node
    rospy.init_node('lab5node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 0.2
    accel = 0.1
    move_arm(pub_command, loop_rate, home, vel, accel)

    # Destination positions - red, yellow, green
    dest_block_xy = np.array([[0.3, -0.1], [0.2, -0.1], [0.1, -0.1]])

    # The following lines provide an image named 'current_img' from
    # the camera simulated in Gazebo

    ic = ImageConverter(SPIN_RATE)
    time.sleep(2)
    current_img = ic.raw_image

    ##### Your Code Starts Here #####
    
    # ToDo: Use defined functions to move the red, yellow and green blocks to 
    #       destination positions 'dest_block_xy' using 'current_img'

    red = FindColorBlockWorldCoord(current_img, "RED")
    yellow = FindColorBlockWorldCoord(current_img, "YELLOW")
    green = FindColorBlockWorldCoord(current_img, "GREEN")
	
    MoveBlock(pub_command, loop_rate, red, dest_block_xy[0], vel, accel)
    MoveBlock(pub_command, loop_rate, yellow, dest_block_xy[1], vel, accel)
    MoveBlock(pub_command, loop_rate, green, dest_block_xy[2], vel, accel)

	
	##### Your Code Ends Here #####

    print("\n")
    print("******************************")
    print("Task completed!")
    print("Move UR3 back to home position")
    print("******************************")
    print("\n")

    # Move the UR3 back to home position
    move_arm(pub_command, loop_rate, home, vel, accel)
    
    print("\n")
    print("******************************")
    print("Use Ctrl+C to exit program")
    print("******************************")
    print("\n")

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
