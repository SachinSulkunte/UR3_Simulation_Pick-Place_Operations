#!/usr/bin/env python

# Program takes in image from robot-mounted camera and determines position of specific color block for pick and place operations.

import cv2
import numpy as np

# Determine block presence using specific color masks
def FindBlockByColor(image, color):
    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # lower = (hmin, smin, vmin)
    # upper = (hmax, smax, vmax)
    lower = (0,0,0)
    upper = (179,255,255)

    if color == "GREEN":
        lower = (36, 50, 70)
        upper = (89, 255, 255)
    elif color == "RED":
        lower = (160, 100, 20)
        upper = (180, 255, 255)
    elif color == "YELLOW":
        lower = (25, 50, 70)
        upper = (35, 255, 255)
    elif color == "WHITE":
        lower = (0, 0, 230)
        upper = (10, 10, 255)
    
    # Find mask image
    mask_image = cv2.inRange(hsv_image, lower, upper)
    
    # Uncomment to show the masked image result
    '''
    cv2.namedWindow("Masked Image")
    cv2.imshow("Masked Image", mask_image)
    cv2.waitKey(0)
    '''

    # Output: a masked image for showing the position of the certain color block
    return mask_image

# Determines center point of block using the masked image
# Output: nx2 array where n is # blobs
def FindBlockCenter(mask_image):
    _image = cv2.bitwise_not(mask_image)
    params = cv2.SimpleBlobDetector_Params()
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs
    keypoints = detector.detect(_image)
    _image = cv2.bitwise_not(_image)

    block_centers = [] 
    for i in keypoints: # iterates for number of blobs found
        block_centers.append([i.pt[0], i.pt[1]])
        print(block_centers)

    # 'block_centers' array has a size of (n x 2), where n is the number of blobs found

    # Uncomment to show the image with 'block_centers' result added
    '''
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(_image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    for i in range(len(keypoints)):   
        cv2.circle(im_with_keypoints, (int(keypoints[i].pt[0]), int(keypoints[i].pt[1])), 2, (0, 0, 255), -1)

    # Show keypoints
    cv2.imshow("Keypoints", im_with_keypoints)
    cv2.waitKey(0)
    '''

    return block_centers

# Function that converts the detected center pixel position in image coord 
# to (x, y) coord in world frame
# Use the four white blocks as reference points to calculate coordinates
# Output: 'block_xy': x, y coordinates in world frame of a block, as a list
def PixelToWorld(target_block_center_pixel):

    block_xw = 0.0
    block_yw = 0.0
    block_xy = [block_xw, block_yw]

    xw = 535 # based on distances from white blocks
    yw = 331.5

    distx = xw - target_block_center_pixel[0][0]
    disty = yw - target_block_center_pixel[0][1]

    block_xy[1] = distx / 552.9 # number pixels per world
    block_xy[0] = disty / 552.9
	
    return block_xy

# Integrates all functions for easy usage
def FindColorBlockWorldCoord(image, color):

    block_xy = [0.0, 0.0]

    mask = FindBlockByColor(image, color)
    center = FindBlockCenter(mask)
    block_xy = PixelToWorld(center)
	
    return block_xy
    

# The above functions can be run with the '__main__' function below: 
# Steps:
# 1) Make this script executable
# 2) Run 'python find_block_pos.py' in terminal

if __name__ == '__main__':
    image = cv2.imread("img_test_1.png")
    #newImg = FindBlockByColor(image, "WHITE")
    #white_pos = FindBlockCenter(newImg) # centroids of white blocks
    print(FindColorBlockWorldCoord(image, "RED"))



