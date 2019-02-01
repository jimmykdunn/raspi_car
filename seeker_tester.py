#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Driver for testing seeker module functions from saved imagery without being
# connected to the raspberry pi.

import seeker
from PIL import Image
import numpy as np

def testseeker():
    # Read test image from file
    frame = Image.open("test_ball_2ft.jpg")
    print("Read test file successfully")
    print("Bit depth: ", frame.bits)
    print("(nx, ny): ", frame.size)
    print("Format: ", frame.format)
    frame = frame.rotate(180)  # Rotate 180 degress: camera mounted upside down
    frame = np.array(frame)  # convert from PIL image to 3-D array
    
    # Calculate commands for the image
    angle, throttleDuty = seeker.calculateCommand(frame)
    print("commanded (angle, throttleDuty): (", angle, ", ", throttleDuty, ")")
    
testseeker()
