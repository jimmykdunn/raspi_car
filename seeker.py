#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Functions for automatic target following with Raspberry-Pi driven electric car
# via seeking to a target in onboard camera imagery.

import numpy as np
import matplotlib.pyplot as plt
import scipy.ndimage.morphology as spm

DEBUG = False

# Finds all pixels with the color of the leader ball
def findLeader(image):
    # Threshold pixels on color (needs to be more complicated than this)
    #leaderMask = np.logical_and(image[:,:,0] > 230, image[:,:,1] < 200, image[:,:,2] < 200)
    redFraction = image[:,:,0].astype(float) / np.sum(image.astype(float), axis=2)
    leaderMask = redFraction > 0.6

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
    #if DEBUG:
    print("Leader area (number of pixels): ", leaderArea)
    print("Leader area (percent of frame): ", 100*leaderFractionalArea)

    # If leader is below a certain size threshold, we probably
    # can't see it. Command no motion. May do something 
    # smarter in the future.
    minLeaderSize = 0.0001 # fraction of image
    if leaderFractionalArea < minLeaderSize:
	print("Leader not found. Car not commanded to move.")
        return 0.0, 0.0

    # Centroid of leader mask is where to point. Find it.
    leaderX, leaderY = locateMaskCentroid(leaderMask)
    leaderX /= leaderMask.shape[0]
    leaderY /= leaderMask.shape[1]
    
    #if DEBUG:
    print("Leader (X,Y) (percent of frame): (", \
	np.round(leaderX*100), ", ", np.round(leaderY*100), ")")

    # Angle is a linear function of leader X position
    centerShift = 0.5 # usually middle of the image
    angleScale = 200.0 # steering degrees conversion. 
    angle = angleScale * (leaderX - centerShift)

    # Force angle to be between -90 and 90
    angle = np.min([90.0, np.max([-90.0, angle])])

    # Throttle duty is a function of the inverse of ball size.
    # Smaller ball area means farther distance to leader,
    # and thus stronger throttle.
    areaScale = 10000
    desiredArea = 0.05
    throttleDuty = (desiredArea - leaderFractionalArea) * areaScale

    # Force throttleDuty to be between -1 and 1
    throttleDuty = np.min([1.0, np.max([-1.0, throttleDuty])])

    return angle, throttleDuty

