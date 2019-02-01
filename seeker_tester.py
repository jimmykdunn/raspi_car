#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Driver for testing seeker module functions from saved imagery without being
# connected to the raspberry pi.

import seeker
from PIL import Image

def testseeker():
    frame = Image.open("test_ball_2ft.jpg")
    print("Read test file successfully")
    print(frame.bits, frame.size, frame.format)
    angle, throttleDuty = seeker.calculateCommand(frame)
    print("commanded (angle, throttleDuty): (", angle, ", ", throttleDuty, ")")
    
