#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm

def Get_MS():
	# Initialize return values of M and S
	M = np.eye(4)
	S = np.zeros((6,6))

	##### Your Code Starts Here #####
	# Fill in scripts from lab3 here
	M = np.eye(4) 
	M = [[0, -1, 0, .39], [0,0,-1,.432], [1,0,0,.2155],[0,0,0,1]]
	S = np.zeros((6,6))
	S = [[0,0,1,-.150,-.150,0],[0,1,0,-.162,0,-.150],[0,1,0,-.162,0,.094],[0,1,0,-.162,0,.307], [1,0,0,0,.162,-.281], [0,1,0,-.162,0,.390]]
	
	
	##### Your Code Ends Here #####

	return M, S


def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	##### Your Code Starts Here #####
	# Fill in scripts from lab3 here
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()
	
	S_mat1 = np.array([[0, -S[0][2], S[0][1], S[0][3]], [S[0][2], 0,-S[0][0], S[0][4]], [-S[0][1], S[0][0], 0, S[0][5]], [0,0,0,0]])
	S_mat2 = np.array([[0, -S[1][2], S[1][1], S[1][3]], [S[1][2], 0,-S[1][0], S[1][4]], [-S[1][1], S[1][0], 0, S[1][5]], [0,0,0,0]])
	S_mat3 = np.array([[0, -S[2][2], S[2][1], S[2][3]], [S[2][2], 0,-S[2][0], S[2][4]], [-S[2][1], S[2][0], 0, S[2][5]], [0,0,0,0]])
	S_mat4 = np.array([[0, -S[3][2], S[3][1], S[3][3]], [S[3][2], 0,-S[3][0], S[3][4]], [-S[3][1], S[3][0], 0, S[3][5]], [0,0,0,0]])
	S_mat5 = np.array([[0, -S[4][2], S[4][1], S[4][3]], [S[4][2], 0,-S[4][0], S[4][4]], [-S[4][1], S[4][0], 0, S[4][5]], [0,0,0,0]])
	S_mat6 = np.array([[0, -S[5][2], S[5][1], S[5][3]], [S[5][2], 0,-S[5][0], S[5][4]], [-S[5][1], S[5][0], 0, S[5][5]], [0,0,0,0]])
	exp_s1 = expm(S_mat1*theta1)
	exp_s2 = expm(S_mat2*theta2)
	exp_s3 = expm(S_mat3*theta3)
	exp_s4 = expm(S_mat4*theta4)
	exp_s5 = expm(S_mat5*theta5)
	exp_s6 = expm(S_mat6*theta6)

	T = np.matmul(exp_s6, M)
	T = np.matmul(exp_s5, T)
	T = np.matmul(exp_s4, T)
	T = np.matmul(exp_s3, T)
	T = np.matmul(exp_s2, T)
	T = np.matmul(exp_s1, T)


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
	L = np.array([0, 152, 120, 244, 93, 213, 83, 83, 82, 53.5, 59])

	x = xWgrip*1000 + 150
	y = yWgrip*1000 - 150
	z = zWgrip*1000 - 10 
	theta_yaw = (yaw_WgripDegree*np.pi)/180

	# Step 1: find gripper position relative to the base of UR3,
	# and set theta_5 equal to -pi/2
	theta_5 = (-np.pi)/2

	# Step 2: find x_cen, y_cen, z_cen
	x_cen = x-L[9]*np.cos(theta_yaw)
	y_cen = y-L[9]*np.sin(theta_yaw)
	z_cen = z

	# Step 3: find theta_1
	theta_1 = np.arctan2(y_cen, x_cen) - np.arcsin((27+L[6])/ np.sqrt(y_cen**2 + x_cen**2))

	# Step 4: find theta_6 
	theta_6 = theta_1 + (np.pi)/2 -theta_yaw

	# Step 5: find x3_end, y3_end, z3_end
	x_3end = x_cen - L[7]*np.cos(theta_1) + (L[6]+27)*np.sin(theta_1)
	y_3end = y_cen - L[7]*np.sin(theta_1) - (L[6]+27)*np.cos(theta_1)
	z_3end = z + L[8] + L[10]

	# Step 6: find theta_2, theta_3, theta_4
	x2 = np.sqrt(x_3end**2 + y_3end**2)
	y2 = z_3end - L[1]
	D = (x2**2 + y2**2 - L[3]**2 - L[5]**2)/((2*L[3])*L[5])
	theta_3 = (np.arctan2(np.sqrt(1-D**2),D))
	theta_2 = -np.arctan2(y2, x2) - np.arctan2((L[5]*np.sin(theta_3)), (L[3] + L[5]*np.cos(theta_3)))
	
	theta_4 = -theta_2 - theta_3

	##### Your Code Ends Here #####

	# print theta values (in degree) calculated from inverse kinematics
	
	print("Joint angles: ")
	print(str(theta_1*180/np.pi) + " " + str(theta_2*180/np.pi) + " " + \
			str(theta_3*180/np.pi) + " " + str(theta_4*180/np.pi) + " " + \
			str(theta_5*180/np.pi) + " " + str(theta_6*180/np.pi))

	# obtain return_value from forward kinematics function
	return_value = lab_fk(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)

	return return_value
