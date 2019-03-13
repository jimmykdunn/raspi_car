#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Program for offline creation of a video display of the captured camera data. 
# Run offline instead of real-time to reduce latency.

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import matplotlib.pyplot as plt
import struct

# Paths to all the data
VIDEO_PATH = "videos/frame_"
VEXT = ".rimg"
MASK_PATH = "videos/mask_"
MEXT = ".rimg"
LOG_PATH = "logs/raspicar.log"

ZOOM = 4 # zoom factor

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
        
        # Read image (nx x ny x ncolors)
        b_frame = vid.read(nx * ny * ncolors)
        
        # Convert byte image into something we can display
        frame = np.zeros(nx*ny*ncolors).astype(int)
        for i,byte in enumerate(b_frame):
            frame[i] = int(byte)
    
        frame = frame.reshape(nx,ny,ncolors)
        frame = np.rot90(frame,3)
    
    return frame


# Main processing function
def process():
    # Read and process the log file
    log = logReader(LOG_PATH)
    log.process()
    
    # Loop over frames
    i = 0
    while True:
        rimgvpath = VIDEO_PATH + str(i) + VEXT
        rimgmpath = MASK_PATH  + str(i) + MEXT
        
        # Try to read the frame. If it fails, we are out of images
        try:
            # Read the video frame
            frame = readRIMG(rimgvpath)
            #plt.imshow(frame)
                
            # Read the mask frame
            mask = readRIMG(rimgmpath)
            mask = np.repeat(mask, 3, axis=2) # make it have 3 colors
            #plt.imshow(mask)
            
            display = np.append(frame, mask, axis = 1).astype(np.uint8)
            infoArea = np.zeros((int(200/ZOOM),display.shape[1],3)).astype(np.uint8)
            display = np.append(display, infoArea, axis = 0)
            #plt.imshow(display)
            displayPIL = Image.fromarray(display, 'RGB')
            displayPIL = displayPIL.resize(
                    (displayPIL.width*ZOOM, displayPIL.height*ZOOM), 
                    resample=Image.BICUBIC)
            ll = [0 + 10, displayPIL.height - 40]
            ImageDraw.Draw(displayPIL).text(
                ll,  # xy
                'Hello world!',  # text
                'white',  # color
                font = ImageFont.truetype("fonts/arial.ttf")) # font
            displayPIL.save("videos/test.png")
            
            # Extract info for this frame from the log
            
            
            # Increment to next number
            i += 1
        except:
            # Read failed
            break

# Run the process program
process()