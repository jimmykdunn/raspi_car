#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Functions for writing an image to file

import numpy as np
from PIL import Image

def saveImage(image, filebase, number):
    #pilimg = Image.fromarray(image)
    #pilimg.save(filebase + "_" + str(int(number)) + ".png", optimize = True)
    
    file = open(filebase + "_" + str(int(number)) + ".rimg", "wb")
    file.write(bytearray(image))
    file.close()
    
