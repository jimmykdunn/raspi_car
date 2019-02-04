#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Functions for automatic target following with Raspberry-Pi driven electric car
# via seeking to a target in onboard camera imagery.

import numpy as np
import matplotlib.pyplot as plt
import scipy.ndimage.morphology as spm

# Parameters
DEBUG = False
TARGET_COLOR = "green"  # "red", "blue"
TARGET_COLOR_THREHSOLD = 0.6 # fraction of R+G+B that target pixels must have
MIN_LEADER_SIZE = 0.0001
ANGLE_SCALE = 600.0 # steering degrees conversion. 
AREA_SCALE = 300 # larger = faster transition to full speed when target is not at desired distance
DESIRED_AREA = 0.05 # fraction of the full image that we want the target to take up
AREA_BUFFER = 0.01 # no command will be issued if area is within this much of DESIRED_AREA
ANGLE_BUFFER = 0.1 # no command will be issued if target center is within this much of image center
# NOTE: AREA_BUFFER and ANGLE_BUFFER work together - i.e. BOTH must be satisfied for no action
# to be taken.

# Finds all pixels with the color of the leader ball
def findLeader(image):
    # Threshold pixels on color (needs to be more complicated than this)
    ballColorFraction = -1.0
    if TARGET_COLOR == "red":
    	ballColorFraction = image[:,:,0].astype(float) / np.sum(image.astype(float), axis=2)
    elif TARGET_COLOR == "green":
	ballColorFraction = image[:,:,1].astype(float) / np.sum(image.astype(float), axis=2)
    elif TARGET_COLOR == "blue":
	ballColorFraction = image[:,:,2].astype(float) / np.sum(image.astype(float), axis=2)
    else:
	print("INVALID TARGET COLOR: ", TARGET_COLOR)
    leaderMask = ballColorFraction > TARGET_COLOR_THREHSOLD

    leaderMask = np.reshape(leaderMask, [image.shape[0], image.shape[1]])
    if DEBUG:
        plt.imshow(leaderMask)
        plt.title("leader mask")
        plt.show()
        #input("BREAKPOINT: Press enter to continue")

    # Run imclose operation to remove small noise pixels.
    # Equivalent to erode followed by dilate
    kernelSize = np.round(image.shape[0] * 0.04).astype(int)
    kernel = np.ones((kernelSize,kernelSize), np.bool)
    eroded = spm.binary_erosion(leaderMask, kernel).astype(np.bool)
    dilated = spm.binary_dilation(eroded, kernel).astype(np.bool)
  
    #if DEBUG:
    #    plt.imshow(eroded)
    #    plt.title("Eroded")
    #    plt.show()
    #    plt.imshow(dilated)
    #    plt.title("Dilated")
    #    plt.show()
    
    leaderMask = dilated

    # Pick only the single largest contiguous area, and fit a circle to it.
    # This makes us more robust to spurious junk and on-target lighting.
    # ADD CODE HERE!!!

    return leaderMask


# Calculates area of leader pixels
def calculateLeaderArea(leaderMask):
    return np.sum(leaderMask)

# Finds the center of the masked area
def locateMaskCentroid(mask):
    #print("MASK SHAPE: ", mask.shape)
    #print("WHERE X: ", np.where(mask)[0])
    #print("WHERE Y: ", np.where(mask)[1])
    return np.mean(np.where(mask)[0]), np.mean(np.where(mask)[1])

# Determine steering and throttle command based on image
def calculateCommand(image):
    if DEBUG:
        plt.imshow(image)
        plt.title("original image")
        plt.show()
        print("type(image): ", type(image))
        print("image.shape: ", image.shape)
        #input("BREAKPOINT: Press enter to continue")
        
    # Find all pixels with the ball
    leaderMask = findLeader(image)

    # Area of leader mask determines distance to leader.
    leaderArea = calculateLeaderArea(leaderMask)
    leaderFractionalArea = float(leaderArea)/(image.shape[0]*image.shape[1])

    # If leader is below a certain size threshold, we probably
    # can't see it. Command no motion. May do something 
    # smarter in the future.
    if leaderFractionalArea < MIN_LEADER_SIZE:
	print("    Leader not visible. Car commanded to stop.")
        return 0.0, 0.0

    # Centroid of leader mask is where to point. Find it.
    leaderX, leaderY = locateMaskCentroid(leaderMask)
    leaderX /= leaderMask.shape[0]
    leaderY /= leaderMask.shape[1]
    
    print "    areaPix, area%, X%, Y%: ", "{:4d}".format(leaderArea),\
        "{:5.1f}".format(100*leaderFractionalArea), \
        "{:3d}".format(np.round(leaderX*100).astype(int)), \
        "{:3d}".format(np.round(leaderY*100).astype(int))

    # Angle is a linear function of leader X position
    centerShift = 0.5 # usually middle of the image
    angle = ANGLE_SCALE * (leaderX - centerShift)

    # Force angle to be between -90 and 90
    angle = np.min([90.0, np.max([-90.0, angle])])

    # Throttle duty is a function of the inverse of ball size.
    # Smaller ball area means farther distance to leader,
    # and thus stronger throttle.
    if (np.abs(leaderFractionalArea - DESIRED_AREA) > AREA_BUFFER) \
	and (np.abs(leaderX - centerShift) > ANGLE_BUFFER):
    	throttleDuty = (DESIRED_AREA - leaderFractionalArea) * AREA_SCALE
    else:
	throttleDuty = 0.0

    # Force throttleDuty to be between -1 and 1
    throttleDuty = np.min([1.0, np.max([-1.0, throttleDuty])])

    return angle, throttleDuty