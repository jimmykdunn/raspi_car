#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Program for offline creation of a video display of the captured camera data. 
# Run offline instead of real-time to reduce latency.

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import matplotlib.pyplot as plt
import struct
import datetime
import platform
import scipy.ndimage as spnd


# Paths to all the data
VIDEO_PATH = "videos/frame_"
VEXT = ".rimg"
MASK_PATH = "videos/mask_"
MEXT = ".rimg"
LOG_PATH = "logs/raspicar.log"
OUT_PATH = "video"

ZOOM = 4 # zoom factor
FONTSIZE = 16

# Class for dealing with raspicar logs
class logReader:
    def __init__(self, path):
        # Initialize member variable arrays
        self.rawlines = []
        self.frames = []
        self.time = []
        self.areapix = []
        self.areapct = []
        self.tgtCtrXpct = []
        self.tgtCtrYpct = []
        self.angle = []
        self.speed = []
        self.leftDuty = []
        self.rightDuty = []
        self.coasting = []
        self.tgtVisible = []
        self.startTime = datetime.datetime.now()
        
        # Read in the log info into a simple text list
        with open(path, "r") as log:
            for line in log:
                #print(line)
                self.rawlines.append(line)
         
    # Append one frame of empty data
    def addFrame(self, num):
        self.frames.append(num)
        self.time.append(0.0)
        self.areapix.append(0.0)
        self.areapct.append(0.0)
        self.tgtCtrXpct.append(0.0)
        self.tgtCtrYpct.append(0.0)
        self.angle.append(0.0)
        self.speed.append(0.0)
        self.leftDuty.append(0.0)
        self.rightDuty.append(0.0)
        self.coasting.append(False)
        self.tgtVisible.append(True)
                
    
    # Take the log and extract useful info into arrays by frame number
    def process(self):
        # Loop over lines
        for line in self.rawlines:
            
            # Split line by commas
            tokens = [token.strip() for token in line.split(',')]
            
            if "Beginning image capture at" in line:
                words = line.split(' ')
                yy = int(words[4][6:10])
                mn = int(words[4][0:2])
                dd = int(words[4][3:5])
                hh = int(words[5][0:2])
                mm = int(words[5][3:5])
                ss = int(words[5][6:8])
                self.startTime = datetime.datetime(yy,mn,dd,hh,mm,ss)
            
            frame = -1
            if tokens[0] == "***" or tokens[0] == "###" or \
             tokens[0] == ">>>" or tokens[0] == "^^^":
                frame = int(tokens[1])
                if not frame in self.frames:
                    self.addFrame(frame)
            
            # Frame time line
            if tokens[0] == "***":
                self.time[frame] = float(tokens[2])
                
            # Target mask info line
            elif tokens[0] == ">>>":
                self.areapix[frame] = float(tokens[6])
                self.areapct[frame] = float(tokens[7])
                self.tgtCtrXpct[frame] = float(tokens[8])
                self.tgtCtrYpct[frame] = float(tokens[9])
                
            # Control command info line
            elif tokens[0] == "###":
                self.angle[frame] = float(tokens[6])
                self.speed[frame] = float(tokens[7])
                self.leftDuty[frame] = float(tokens[8])
                self.rightDuty[frame] = float(tokens[9])
            
            # Target coast/not visible line
            elif tokens[0] == "^^^":
                if "Leader not visible" in tokens[2]:
                    self.tgtVisible[frame] = False
                    self.coasting[frame] = True
                if "Leader lost" in tokens[2]:
                    self.coasting[frame] = False
            
            # Printed info status line (ignore)
            else:
                pass


# Read a frame from file (.rimg format)
def readRIMG(path):
    with open(path, "rb") as vid:
        # Read dimensions
        nx = struct.unpack('i', vid.read(4))[0]
        ny = struct.unpack('i', vid.read(4))[0]
        ncolors = struct.unpack('i', vid.read(4))[0]
    	#print nx, ny, ncolors
        
        # Read image (nx x ny x ncolors)
        b_frame = vid.read(nx * ny * ncolors)
        
        # Convert byte image into something we can display
        frame = np.zeros(nx*ny*ncolors).astype(int)
        for i,byte in enumerate(b_frame):
            if "Windows" in platform.system():
                frame[i] = int(byte)
            else:
                frame[i] = struct.unpack('=B',byte)[0]
    
        frame = frame.reshape(nx,ny,ncolors)
        frame = np.rot90(frame,3)
        
    return frame


# Makes an arrow showing the desired direction of travel based on drive duty.
def drawPowerBars(xy, leftDuty, rightDuty):
    fullbox = np.zeros(xy).astype(np.uint8)
    strip = np.zeros(int(xy[1]/2)).astype(np.uint8)
    
    strip_x = (np.arange(xy[1]/2)-xy[1]/4)/(xy[1]/4)
    strip[(strip_x < 0) * (strip_x > -leftDuty)] = 255
    strip[(strip_x > 0) * (strip_x < rightDuty)] = 255
    
    #if leftDuty > 0 and rightDuty > 0:
       # box_x = np.reshape(np.repeat(np.arange(box.shape[0]), 
       #         box.shape[1],axis=0),[box.shape[0],box.shape[1]]) - box.shape[0]/2
       # box_y = np.reshape(np.repeat(np.arange(box.shape[1]),
       #         box.shape[0],axis=0),[box.shape[1],box.shape[0]]).T - box.shape[1]/2
       # box[np.abs((box_y/box_x) - (rightDuty-leftDuty)) < 0.3] = 255
    x0 = int(1.0*xy[1]/2.0)
    y0 = int(xy[0]*0.28)
    for i in range(xy[2]):
        fullbox[y0,x0:,i] = strip
    
    return fullbox
    



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

############################
# NEW PARAMETERS
#IDEAL_HSV = [55, 0.95, 0.6] # best yet
#HSV_SIGMA = [5, 0.1, 0.2] # best yet
#HUE_RANGE = 15 # best yet
#BALL_THRESH = 0.1 # best yet
IDEAL_HSV = [60, 0.95, 0.6]
HSV_SIGMA = [10, 0.1, 0.2]
HUE_RANGE = 15
BALL_THRESH = 0.01 # min probability a pixel can have and be declared ball
MAX_BALL_SIZE = 0.15 # radius around peak ball pixel to search, as a fraction of image size
    
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
    
    probBallSm = spnd.gaussian_filter(probBall, 4) # gaussian smoothing filter
    #probBallSm = probBall
    peakXY = np.unravel_index(np.argmax(probBallSm, axis=None), probBall.shape)
    
    X = np.reshape(np.tile(np.arange(nx), ny),[ny,nx])
    Y = np.reshape(np.tile(np.arange(ny), nx),[nx,ny]).T
    
    # Can be slow, maybe do a box instead?
    distToPeak2 = (peakXY[0] - Y)**2 + (peakXY[1] - X)**2
    mask = (distToPeak2 < (MAX_BALL_SIZE * (nx+ny)/2)**2) * \
           (probBall >= BALL_THRESH)
           
    # If peak pixel is less than threshold, declare ball not present
    if np.amax(probBall) < BALL_THRESH:
        mask[:,:] = False
    
    
    return mask, probBall

############################



# Main processing function
def process():
    # Read and process the log file
    log = logReader(LOG_PATH)
    log.process()
    
    # Loop over frames
    i = 0
    video = []
    while True:
        rimgvpath = VIDEO_PATH + str(i) + VEXT
        rimgmpath = MASK_PATH  + str(i) + MEXT
        
        # Try to read the frame. If it fails, we are out of images
        try:
            # Read the video frame
            frame = np.fliplr(readRIMG(rimgvpath))
            #plt.imshow(frame)
            
            # HSV probability masking for testing
            #hsvFrame = rgb2hsv(frame)
            #hsvMask, probBall = findTargetProb(hsvFrame)
            #hsvMask3 = 255*np.repeat(np.reshape(hsvMask,[hsvMask.shape[0],hsvMask.shape[1],1]), 3, axis=2)
            #probBall3 = 255*np.repeat(np.reshape(probBall,[probBall.shape[0],probBall.shape[1],1]), 3, axis=2)
            #h3view = np.append(np.append(frame,hsvMask3,axis=1),probBall3,axis=1)
            #plt.imshow(h3view)
            #plt.imshow(hsvMask)
            #plt.imshow(probBall)
            #plt.imshow(frame)
            
            #hs = (frame[:,:,1].astype(float)+frame[:,:,0].astype(float))/(2*255) \
            #    - frame[:,:,2].astype(float) / (255) \
            #    - np.abs(frame[:,:,0].astype(float) - frame[:,:,1].astype(float)) / 255
            #ballColorFraction = hs #-0.5-20*h + 10*v
            
                           
            # Read the mask frame
            mask = np.fliplr(readRIMG(rimgmpath)) * 255
            #mask = np.reshape(((ballColorFraction > 0.1) * 255).astype(np.int32),[hs.shape[0],hs.shape[1],1])
            
            # Use for debugging only!! Post-processing test of algorithm
            #hsvMask = 255*np.reshape(hsvMask, [hsvMask.shape[0], hsvMask.shape[1], 1])#!!!TEMPORARY!!!
            
            mask = np.repeat(mask, 3, axis=2) # make it have 3 colors
            #plt.imshow(mask)
            
            # Put them together with some area for text at the bottom
            display = np.append(frame, mask, axis = 1).astype(np.uint8)
            infoArea = np.zeros((int(200/ZOOM),display.shape[1],3)).astype(np.uint8)
            infoArea += drawPowerBars(infoArea.shape, log.leftDuty[i], log.rightDuty[i])
            display = np.append(display, infoArea, axis = 0)
            #plt.imshow(display) 
            displayPIL = Image.fromarray(display, 'RGB')
            displayPIL = displayPIL.resize(
                    (displayPIL.width*ZOOM, displayPIL.height*ZOOM), 
                    resample=Image.BICUBIC)
            
            lastT = 0.0
            if i > 0:
                lastT = log.time[i-1]
            # Extract info for this frame from the log
            coln1 = "         CAMERA VIEW\n\n" + \
                    "Frame:        " + str(log.frames[i]) + "\n" \
                    "Time (s):     " + "%0.2f"% (log.time[i]) + "\n" \
                    "Frmrate (Hz): " + "%0.2f"% (1.0/(log.time[i]-lastT)) + "\n" \
                    "Speed:        " + str(log.speed[i]) + "\n" \
                    "Left Duty:    " + str(log.leftDuty[i]) + "\n" \
                    "Right Duty:   " + str(log.rightDuty[i]) + "\n" \
                    "Leader Visbl: " + str(log.tgtVisible[i]) + "\n" + \
                    "Coasting:     " + str(log.coasting[i])
            coln2 = "         TARGET MASK\n\n\n" + \
                    "  LEFT MOTOR      RIGHT MOTOR\n" + \
                    "     POWER           POWER\n" + \
                    "Angle (deg):  " + "%0.2f"% (log.angle[i]) + "\n" \
                    "Area (%)      " + "%0.2f"% (log.areapct[i]) + "\n" \
                    "Area (pix):   " + "%0.2f"% (log.areapix[i]) + "\n" \
                    "Target X:     " + "%0.3f"% (log.tgtCtrXpct[i]/100) + "\n" \
                    "Target Y:     " + "%0.3f"% (log.tgtCtrYpct[i]/100)
                                  
            
            # Add text
            font = ImageFont.truetype("fonts/cour.ttf", FONTSIZE)
            p1 = [0 + 10, displayPIL.height - 190]
            p2 = [displayPIL.width/2 + 10, displayPIL.height - 190]
            ImageDraw.Draw(displayPIL).text(p1, coln1, 'white', font=font)
            ImageDraw.Draw(displayPIL).text(p2, coln2, 'white', font=font)
                        
            # Save image to video
            displayPIL.save("videos/test.png")
            video.append(displayPIL)
            
            # Increment to next number
            print("Formed frame ", i)
            i += 1
        except:
            print("Read completed!")
            break

    # Save all images as an animated GIF
    dtframe = np.mean(np.diff(log.time)) * 1000
    startTime = log.startTime.strftime("%Y%m%d_%H%M%S")
    video[0].save(OUT_PATH+"_"+startTime+".gif", 
         save_all=True, append_images=video[1:], duration=dtframe, loop=0)

# Run the process program
process()