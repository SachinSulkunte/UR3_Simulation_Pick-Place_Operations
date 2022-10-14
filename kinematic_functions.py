#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm

def Get_MS():
	# Initialize return values of M and S
	M = np.eye(4)
	S = np.zeros((6,6))

	##### Your Code Starts Here #####
	# Fill in scripts from lab3 here

	
	##### Your Code Ends Here #####

	return M, S


def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	##### Your Code Starts Here #####
	# Fill in scripts from lab3 here



	##### Your Code Ends Here #####
	print(str(T) + "\n")

	return_value[0] = theta1 + np.pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*np.pi)
	return_value[4] = theta5
	return_value[5] = theta6

	if T[2, 3] < 0:
		print("Calculated Z coordinate is less than 0. The robot joint angles will be set as 0.")
		return_value = np.array([0, 0, 0, 0, 0, 0])

	return return_value


def inverse_kinematics(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	return_value = np.array([0, 0, 0, 0, 0, 0])

	##### Your Code Starts Here #####
	# Fill in scripts from lab4 here

	


	##### Your Code Ends Here #####

	# print theta values (in degree) calculated from inverse kinematics
	
	print("Joint angles: ")
	print(str(theta_1*180/np.pi) + " " + str(theta_2*180/np.pi) + " " + \
			str(theta_3*180/np.pi) + " " + str(theta_4*180/np.pi) + " " + \
			str(theta_5*180/np.pi) + " " + str(theta_6*180/np.pi))

	# obtain return_value from forward kinematics function
	return_value = lab_fk(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)

	return return_value
