#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Functions for writing an image to file

import numpy as np
from PIL import Image
import struct

def saveImage(image, filebase, number):
    #pilimg = Image.fromarray(image)
    #pilimg.save(filebase + "_" + str(int(number)) + ".png", optimize = True)
    
    file = open(filebase + "_" + str(int(number)) + ".rimg", "wb")
    
    # Write header (just 3 ints with the dimensions)
    file.write(struct.pack('i',image.shape[0]))
    file.write(struct.pack('i',image.shape[1]))
    if image.ndim > 2:
        file.write(struct.pack('i',image.shape[2]))
    else:
        file.write(struct.pack('i',1))
    
    # Write the actual image (just an array of bytes, RGB)
    file.write(bytearray(image))
    file.close()
    
