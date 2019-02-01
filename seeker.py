#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Functions for automatic target following with Raspberry-Pi driven electric car
# via seeking to a target in onboard camera imagery.

import numpy as np

# Finds all pixels with the color of the leader ball
def findLeader(image):
    # Threshold pixels on color (needs to be more complicated than this)
    leaderMask = np.where(image > 200)

    # Run imclose operation to remove small noise pixels.
    # Equivalent to erode followed by dilate
    kernelSize = 10
    leaderMask = imclose(leaderMask, kernelSize)

    return leaderMask


# Calculates area of leader pixels
def calculateLeaderArea(leaderMask):
    return np.sum(leaderMask)

# Finds the center of the masked area
def locateMaskCentroid(mask):
    return np.mean(np.find(mask))

# Determine steering and throttle command based on image
def calculateCommand(image):
    # Find all pixels with the ball
    leaderMask = findLeader(image)

    # Area of leader mask determines distance to leader.
    leaderArea = calculateLeaderArea(leaderMask)

    # If leader is below a certain size threshold, we probably
    # can't see it. Command no motion. May do something 
    # smarter in the future.
    minLeaderSize = 30 # pixels
    if leaderArea < minLeaderSize:
        return 0.0, 0.0

    # Centroid of leader mask is where to point. Find it.
    leaderX, leaderY= locateMaskCentroid(leaderMask)

    # Angle is a linear function of leader X position
    centerShift = 500 # usually middle of the image
    angleScale = 0.1 # pixels off center of image to steering degrees conversion. 
    angle = angleScale * leaderX + centerShift

    # Throttle duty is a function of the inverse of ball size.
    # Smaller ball area means farther distance to leader,
    # and thus stronger throttle.
    areaScale = 100
    desiredArea = 100
    throttleDuty = (desiredArea - ballArea) / areaScale

    # Force throttleDuty to be between -1 and 1
    throttleDuty = np.min([1.0, np.max([-1.0, throttleDuty])])

    return angle, throttleDuty

