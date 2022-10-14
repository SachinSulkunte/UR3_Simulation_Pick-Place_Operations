#!/usr/bin/env python

import rospy
import rospkg
import os
import numpy as np
import random

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

def GetReferenceBlockPositions():
    # positions for four reference white blocks: 4x2 array
    ref_block_xy_pos = np.array([[0.0, 0.9], [0.5, 0.9], [0.5, -0.5], [0.0, -0.5]])
    return ref_block_xy_pos

def GetBlockPositions():
    # ToDo:     Generate 3 random (x, y) positions for spawning blocks
    #           Each x or y is a random number between 0.1 and 0.35
    #           Distance between each 2 of 3 points should be greater than 0.06 meters
    #           
    # Hint:     Use any random number generator in Python (e.g., 'np.random.rand()', etc.)
    #
    # Output:   'block_xy_pos': positions for three blocks as a 3x2 array
    #           1st row: red block (x, y)
    #           2nd row: yellow block (x, y)
    #           3rd row: green block (x, y)

    # Fixed initial positions
    block_xy_pos = np.array([[0.2, 0.05], [0.3, 0.05], [0.4, 0.05]])

    ##### Your Code Starts Here #####
    arr = []
    num = .25 * np.random.random_sample() + .1
    num2 = .25 * np.random.random_sample() + .1
    arr.append((num, num2))
    while len(arr) < 3:
    	num = .25 * np.random.random_sample() + .1
    	num2 = .25 * np.random.random_sample() + .1
    	if all(np.sqrt((other1 - num)**2 + (other2-num2)**2) > .06 for other1, other2 in arr):
		arr.append((num, num2))

    row = 0
    for i,j in arr:
	block_xy_pos[row][0] = i
        block_xy_pos[row][1] = j
	row += 1
	##### Your Code Ends Here #####

    return block_xy_pos


if __name__ == '__main__':

    # Initialize ROS node
    rospy.init_node('lab5_spawn_blocks_node', anonymous=True)
    # Initialize ROS pack
    rospack = rospkg.RosPack()
    # Get path to block
    ur_path = rospack.get_path('lab5')

    block_ref_path = os.path.join(ur_path, 'urdf', 'block_white.urdf')
    block1_path = os.path.join(ur_path, 'urdf', 'block_red.urdf')
    block2_path = os.path.join(ur_path, 'urdf', 'block_yellow.urdf')
    block3_path = os.path.join(ur_path, 'urdf', 'block_green.urdf')
    block_paths = [block1_path, block2_path, block3_path]

    rospy.wait_for_service('gazebo/spawn_urdf_model')
    spawn = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
    delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

    # Spawn four white blocks for position calculation
    ref_block_xy_pos = GetReferenceBlockPositions()
    for i_block_ref in range(4):
        block_name = 'block_ref_' + str(i_block_ref + 1)
        pose = Pose(Point(ref_block_xy_pos[i_block_ref, 0], ref_block_xy_pos[i_block_ref, 1], 0), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(block_ref_path, 'r').read(), 'block', pose, 'world')
      
    # Spawn three blocks
    block_xy_pos = GetBlockPositions()
    for i_block in range(3):
        block_name = 'block_' + str(i_block + 1)
        pose = Pose(Point(block_xy_pos[i_block, 0], block_xy_pos[i_block, 1], 0), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(block_paths[i_block], 'r').read(), 'block', pose, 'world')
    
    print("\n")
    print("********** Initial Block Positions **********")
    print(block_xy_pos)
    print("*********************************************")
    print("\n")
