#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Functions for automatic target following with Raspberry-Pi driven electric car
# via seeking to a target in onboard camera imagery.

import numpy as np
#import matplotlib.pyplot as plt
import scipy.ndimage.morphology as spm
#import scipy.misc
import datetime
import logger
import matplotlib.colors as mpc
from PIL import Image
import scipy.ndimage as spnd

# Parameters
DEBUG = False
TARGET_COLOR = "neonyellow"  # "red" "blue" "green" "neonyellow"
DO_IMOPEN = False # run the imopen operation
TARGET_COLOR_SENSITIVITY = 0.85 #0.5 (good for blue) # fraction of R+G+B that target pixels must have
MIN_LEADER_BRIGHTNESS = 1.0# relative to image mean
MIN_LEADER_SIZE = 0.0001
ANGLE_SCALE = 400.0 #600.0 # steering degrees conversion. 
AREA_SCALE = 30 #300 # larger = faster transition to full speed when target is not at desired distance
DESIRED_AREA = 0.03 #0.10 # fraction of the full image that we want the target to take up
AREA_BUFFER = 0.01 # no command will be issued if area is within this much of DESIRED_AREA
ANGLE_BUFFER = 0.1 # no command will be issued if target center is within this much of image center
# NOTE: AREA_BUFFER and ANGLE_BUFFER work together - i.e. BOTH must be satisfied for no action
# to be taken.
COAST_TIME = 4.0 # Seconds car is allowed to coast last command before stopping when target not visible
MIN_COAST_ANGLE = 60.0 # smallest allowable steer angle for a coast


# HSV Colorspace analysis parameters
# See http://colorizer.org/ for examples
DO_HSV = True
IDEAL_HSV = [60, 0.95, 0.6]  # neon yellow target ball
HSV_SIGMA = [10, 0.1, 0.2]
HUE_RANGE = 15
BALL_THRESH = 0.01 # min probability a pixel can have and be declared ball
MAX_BALL_SIZE = 0.15 # radius around peak ball pixel to search, as a fraction of image size


# Global variables
timeTgtLastVisible = datetime.datetime(2019,01,01,00,00,01)
prevValidAngle = 0.0
prevValidDuty = 0.0

###########################
# Adptded from formulae on www.rapidtables.com/convert/color/rgb-to-hsv.html
def rgb2hsv(rgb):
    r, g, b = rgb[:,:,0]/255.0, rgb[:,:,1]/255.0, rgb[:,:,2]/255.0
    r, g, b = r.flatten(), g.flatten(), b.flatten()
    rgbf = np.reshape(rgb,[rgb.shape[0]*rgb.shape[1],rgb.shape[2]])
    cmax = np.amax(rgbf,axis=1)/255.0
    cmin = np.amin(rgbf,axis=1)/255.0
    delta = cmax-cmin
    h = np.zeros((rgb.shape[0]*rgb.shape[1]))
    s = np.zeros((rgb.shape[0]*rgb.shape[1]))
    v = np.zeros((rgb.shape[0]*rgb.shape[1]))
    #if np.any(delta == 0):
    if np.any(delta <= 1e-4):
        h[delta == 0] = 0
    if np.any(cmax == r):
        h[cmax == r] = (60 * (((g[cmax == r]-b[cmax == r])/delta[cmax == r]) % 6))
    if np.any(cmax == g):
        h[cmax == g] = (60 * (((b[cmax == g]-r[cmax == g])/delta[cmax == g]) + 2))
    if np.any(cmax == b):
        h[cmax == b] = (60 * (((r[cmax == b]-g[cmax == b])/delta[cmax == b]) + 4))
    if np.any(cmax == 0):
        s[cmax == 0] = 0       
    if np.any(cmax != 0):
        s[cmax != 0] = delta[cmax != 0]/cmax[cmax != 0]
    v = cmax
    
    # Eliminate infinities
    h[np.isfinite(h) == False] = 0.0
    s[np.isfinite(s) == False] = 0.0
    v[np.isfinite(v) == False] = 0.0 
    
    h = h.reshape([rgb.shape[0],rgb.shape[1]])
    s = s.reshape([rgb.shape[0],rgb.shape[1]])
    v = v.reshape([rgb.shape[0],rgb.shape[1]]) 
    hsv = np.stack([h,s,v], axis=2)
    
    return hsv


# Find the probability of the ball being at each pixel. Then find the peak
# probabiility pixel and declare it as being in the ball if it is above 
# threshold.  Look for the rest of the ball nearby to determine area.  The 
# idea here is that the "most yellow" thing in the frame should be the ball.
# Thus the best way to find it is finding the most yellow pixel and looking
# around it.
def findTargetProb(hsvFrame):
    nx = hsvFrame.shape[1]
    ny = hsvFrame.shape[0]
    delta = hsvFrame - IDEAL_HSV
    delta[delta > 180] = 360 - delta[delta > 180] # will only change hue. Sat and val are 0 to 1
    probBall = np.exp(-(np.sum((delta/HSV_SIGMA)**2,axis=2)))
    probBall *= (hsvFrame[:,:,0] > (IDEAL_HSV[0] - HUE_RANGE)) * (hsvFrame[:,:,0] < (IDEAL_HSV[0] + HUE_RANGE)) # MUST be correct hue
    
    #probBallSm = spnd.gaussian_filter(probBall, 4) # gaussian smoothing filter
    probBallSm = probBall
    peakXY = np.unravel_index(np.argmax(probBallSm, axis=None), probBall.shape)
    
    X = np.reshape(np.tile(np.arange(nx), ny),[ny,nx])
    Y = np.reshape(np.tile(np.arange(ny), nx),[nx,ny]).T
    
    # Can be slow, maybe do a box instead?
    #distToPeak2 = (peakXY[0] - Y)**2 + (peakXY[1] - X)**2
    #mask = (distToPeak2 < (MAX_BALL_SIZE * (nx+ny)/2)**2) * \
    #       (probBall >= BALL_THRESH)
    distToPeak = np.abs(peakXY[0] - Y) + np.abs(peakXY[1] - X)
    mask = (distToPeak < (MAX_BALL_SIZE * (nx+ny)/2)) * \
           (probBall >= BALL_THRESH)
           
    # If peak pixel is less than threshold, declare ball not present
    if np.amax(probBall) < BALL_THRESH:
        mask[:,:] = False
    
    
    return mask, probBall
    
    
   
############################

# Finds all pixels with the color of the leader ball
def findLeader(image):

    leaderMask = 0
    if DO_HSV:
        hsvFrame = rgb2hsv(image)
        leaderMask, probBall = findTargetProb(hsvFrame)    	    
    else:    
	# Threshold pixels on RGB color (needs to be more complicated than this)
	ballColorFraction = -1.0
	if TARGET_COLOR == "red":
	    ballColorFraction = image[:,:,0].astype(float) / np.sum(image.astype(float), axis=2)
	elif TARGET_COLOR == "green":
	    ballColorFraction = image[:,:,1].astype(float) / np.sum(image.astype(float), axis=2)
	elif TARGET_COLOR == "blue":
	    ballColorFraction = image[:,:,2].astype(float) / np.sum(image.astype(float), axis=2)
	elif TARGET_COLOR == "neonyellow":
	    ballColorFraction = (image[:,:,0].astype(float)+image[:,:,1].astype(float)) / np.sum(image.astype(float), axis=2)
	else:
	    print("INVALID TARGET COLOR: ", TARGET_COLOR)
	    
    	image_brightness = np.sum(image.astype(float), axis=2) / 3 / np.mean(image.astype(float))
    	leaderMask = (ballColorFraction > TARGET_COLOR_SENSITIVITY) * \
    	         (image_brightness > MIN_LEADER_BRIGHTNESS)
    # end else (RGB)

    leaderMask = np.reshape(leaderMask, [image.shape[0], image.shape[1]])

    #scipy.misc.toimage(image).save('fullImage.jpg')
    #scipy.misc.toimage(leaderMask).save('leaderMask.jpg')
    if DEBUG:
    	#plt.imshow(leaderMask)
        #plt.title("leader mask")
        #plt.show()
        input("BREAKPOINT: Press enter to continue")

    # Run imclose operation to remove small noise pixels.
    # Equivalent to erode followed by dilate
    if DO_IMOPEN:
    	kernelSize = np.round(image.shape[0] * 0.04).astype(int)
    	kernel = np.ones((kernelSize,kernelSize), np.bool)
    	eroded = spm.binary_erosion(leaderMask, kernel).astype(np.bool)
    	dilated = spm.binary_dilation(eroded, kernel).astype(np.bool)
    	leaderMask = dilated
  
    #if DEBUG:
    #    plt.imshow(eroded)
    #    plt.title("Eroded")
    #    plt.show()
    #    plt.imshow(dilated)
    #    plt.title("Dilated")
    #    plt.show()
    

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
def calculateCommand(image, loopCount, log):
    global timeTgtLastVisible
    global prevValidAngle
    global prevValidDuty
    
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
    #print("leaderArea: ", leaderArea, ", Image shape: ", image.shape)        
    leaderFractionalArea = float(leaderArea)/(image.shape[0]*image.shape[1])
    
    # If leader is below a certain size threshold, we probably
    # can't see it. Command no motion. May do something 
    # smarter in the future.
    angle = 0.0
    throttleDuty = 0.0
    if leaderFractionalArea < MIN_LEADER_SIZE:
        
        # Target is not (easily) visible. Repeat last command, so long as
        # we have not been without target visibility in more than N sec, and
        # so long as the command is a turn (don't coast forward too much!)
        currentTime = datetime.datetime.now()
        dt = (currentTime - timeTgtLastVisible).total_seconds()
	log.write("^^^," + "{:5d}".format(loopCount) + ", Leader not visible. Coasted for " + str(dt) + " seconds\n")
        if COAST_TIME >= dt and np.abs(prevValidAngle) > MIN_COAST_ANGLE:
	    #print("    Prev angle: ", prevValidAngle)
	    #print("    Prev duty: ", prevValidDuty)
            angle = prevValidAngle
	    throttleDuty = prevValidDuty
	else:
	    log.write("^^^," + "{:5d}".format(loopCount) + ", Leader lost. Car commanded to stop.\n")
            return 0.0, 0.0, leaderMask
    else:
    
        # Centroid of leader mask is where to point. Find it.
        leaderX, leaderY = locateMaskCentroid(leaderMask)
        leaderX /= leaderMask.shape[0]
        leaderY /= leaderMask.shape[1]
    
        log.write(">>>," + "{:5d}".format(loopCount) + ", areaPix, area%, X%, Y%, " + \
            "{:4d}".format(leaderArea) + ", " + \
            "{:5.1f}".format(100*leaderFractionalArea) + ", " + \
            "{:3d}".format(np.round(leaderX*100).astype(int)) + ", " + \
            "{:3d}".format(np.round(leaderY*100).astype(int)) + "\n")

        # Angle is a linear function of leader X position
        centerShift = 0.5 # usually middle of the image
        angle = ANGLE_SCALE * (leaderX - centerShift)

        # Force angle to be between -90 and 90
        angle = np.min([90.0, np.max([-90.0, angle])])

        # Throttle duty is a function of the inverse of ball size.
        # Smaller ball area means farther distance to leader,
        # and thus stronger throttle.
        if (np.abs(leaderFractionalArea - DESIRED_AREA) > AREA_BUFFER) \
	    or (np.abs(leaderX - centerShift) > ANGLE_BUFFER):
    	    throttleDuty = (DESIRED_AREA - leaderFractionalArea) * AREA_SCALE
    	
    	    # Save current time as time the target was last visible
    	    timeTgtLastVisible = datetime.datetime.now()
    	    prevValidAngle = angle
    	    prevValidDuty = throttleDuty
        

    # Force throttleDuty to be between -1 and 1
    throttleDuty = np.min([1.0, np.max([-1.0, throttleDuty])])
    
    # Do not allow negative throttleDuty (i.e. don't go in reverse)
    throttleDuty = np.max([0.0, throttleDuty])

    return angle, throttleDuty, leaderMask
