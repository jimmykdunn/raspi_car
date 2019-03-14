#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Functions for manual control of Raspberry-Pi driven electric car.

# Get user input from keyboard (if any) and translate to angle and speed.
def getUserCmd():
    override = False
    userSteer = 0.0
    userDuty = 0.0

    # User override commands are held in a one-character text file.
    # The file is updated at a high rate by an external program. 
    # Read that file here.  This allows input over an ssh terminal.
    try:
        fd = open("overrideCmd.txt", "r")
        keypress = fd.read()
        fd.close()
    except:
        keypress = ' '
        print("Unable to read user override command file. User override " + \
            "will be unavailable")
    
    # Transalte key press
    if keypress == 'a':  # Left
        userSteer = -90.0
        override = True
    if keypress == 'd':  # Right
        userSteer = 90.0
        override = True
    if keypress == 's':  # Backward
        userDuty = -1.0
        override = True
    if keypress == 'w':  # Forward
        userDuty = 1.0
        override = True
    if keypress == 'c':  # Stop moving
        userDuty = 0.0
        override = True
    
    return override, userSteer, userDuty
