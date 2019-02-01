#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Control program for raspberry-pi driven RC car.  Program is intended to be
# run constantly on the raspberry pi that sits aboard the car.
# import matplotlib.pyplot as plt # plotting
import numpy as np # basic math
from time import sleep # sleep statements
import RPi.GPIO as GPIO # general purpose output pin control library
import atexit
import pigpio
import sys
import select

STEER_SCALE = 1.0 # Calibrates how much to slow motors when turning
PWM_FREQ = 1000 # Frequency of motor control PWM signal
PWM_VAL_MAX = 1e4 # Maximum value of PWM duty command

# Pin mappings
PIN_INFO_LED = 4 #7 # Pin connecting to LED that confirms main program is running
PIN_LT_PWM = 12 #32 # Pin that controls the PWM for the left wheels (GPIO12, PWM0)
PIN_RT_PWM = 13 #33 # Pin that controlls the PWM for the right wheels (GPIO13, PWM1)
PIN_ERROR_LED = 17 #11 # Pin connecting to LED that indicates an error
PIN_LT_POL_FWD = 27 #13 # Pin for commanding drive direction (left wheels)
PIN_LT_POL_BWD = 22 #15 # Pin for commaiding drive direction (left wheels)
PIN_RT_POL_FWD = 23 #16 # Pin for commanding drive direction (left wheels)
PIN_RT_POL_BWD = 24 #18 # Pin for commaiding drive direction (left wheels)
PIN_LT_LED = 10 #19 # Pin for LED indicating left turn commanded
PIN_RT_LED = 9 #21 # Pin for LED indicating right turn commanded

# Inital setup script
def setup():
    
    # GPIO Setup
    GPIO.setwarnings(False)
    #GPIO.setmode(GPIO.BOARD) # utilize physical pin mappings
    GPIO.setmode(GPIO.BCM) # utilize physical pin mappings
    
    # Initialize pins
    GPIO.setup(PIN_INFO_LED, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_LT_PWM, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_RT_PWM, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_ERROR_LED, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_LT_POL_FWD, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_RT_POL_FWD, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_LT_POL_BWD, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_RT_POL_BWD, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_LT_LED, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_RT_LED, GPIO.OUT, initial=GPIO.LOW)

    # Set up pigpio daemon for PWM
    pig = pigpio.pi()
    for pin in [PIN_LT_PWM, PIN_RT_PWM]:
        pig.set_mode(pin, pigpio.OUTPUT)
        pig.hardware_PWM(pin, PWM_FREQ, 0)
        pig.set_PWM_range(pin, PWM_VAL_MAX)

    # Blink all LEDs 5x to confirm that the program is starting, 
    # leave green one on to confirm main program is running.
    for i in range(8):
        GPIO.output(PIN_INFO_LED, GPIO.HIGH)
        GPIO.output(PIN_ERROR_LED, GPIO.HIGH)
        GPIO.output(PIN_LT_LED, GPIO.HIGH)
        GPIO.output(PIN_RT_LED, GPIO.HIGH)
        sleep(0.2)
        GPIO.output(PIN_INFO_LED, GPIO.LOW)
        GPIO.output(PIN_ERROR_LED, GPIO.LOW)
        GPIO.output(PIN_LT_LED, GPIO.LOW)
        GPIO.output(PIN_RT_LED, GPIO.LOW)
        sleep(0.2)
    GPIO.output(PIN_INFO_LED, GPIO.HIGH)
        
    # Maybe also connect passive buzzer and play a tone to indicate poweron?
    # Nice to have, but not necessary.

    print("Initialization complete")

    return pig
    
# end setup
    
    
# Function to run when error is indicated    
def error(message):
    # Light the error LED
    GPIO.output(PIN_ERROR_LED, GPIO.HIGH)
    
    # Play a tone on the passive buzzer?
    
    # Write error message to log file?
    
# end error()
    
        
# Converts the desired angle and speed into PWM duty factors for the left and
# right wheel motors. Angle is in degrees, negative for left, positive for 
# right. Speed is -1 (full throttle backward), to +1 (full throttle forward).
# Output duty is -1 (full throttle backward), to +1 (full throttle forward).
def angleSpeedToDuty(angleDeg, speed):
    if angleDeg <= 0.0:
        # Turn left by slowing the left motors
        leftDuty = speed * np.cos(np.deg2rad(angleDeg)) * STEER_SCALE
        rightDuty = speed
    else:
        # Turn right by slowing the right motors
        leftDuty = speed
        rightDuty = speed * np.cos(np.deg2rad(angleDeg)) * STEER_SCALE
        
    # Change to a "rotate vehicle" regime instead of a "move forward" regime
    # if the commanded angle is large.
    if angleDeg < -80.0:
        leftDuty = 0.0
    if angleDeg > 80.0:
        rightDuty = 0.0

    return leftDuty, rightDuty
# end angleSpeedToDuty    


# Commands the desired pin to execute a PWM at the specified duty factor
def commandPWM(pig, duty, pin):

    # Frequency in Hz, duty 0 (off) to PWM_VAL_MAX (full on)
    pig.set_PWM_dutycycle(pin, PWM_VAL_MAX*abs(duty))
    #GPIO.output(pin, GPIO.HIGH)
        
# end commandPWM
    

# Command the polarity (forward/backward). Polarity = True indicates forward,
# polarity = False indicates backward.
def commandMotorPolarity(polarity, pin):
    if polarity:
        GPIO.output(pin, GPIO.HIGH)
    else:
        GPIO.output(pin, GPIO.LOW)
# end commandMotorPolarity

        
# Illuminate LED(s) indicating desired turn direction
def illumnateDirectionPins(leftDuty, rightDuty, pin_left, pin_right):
    if leftDuty > rightDuty:
        # Car has been commanded to turn right
        GPIO.output(pin_left, GPIO.LOW)
        GPIO.output(pin_right, GPIO.HIGH)
        
    if leftDuty < rightDuty:
        # Car has been commanded to turn left
        GPIO.output(pin_left, GPIO.HIGH)
        GPIO.output(pin_right, GPIO.LOW)
        
    if leftDuty == rightDuty:
        # Car has been commanded to move perfectly straight
        GPIO.output(pin_left, GPIO.HIGH)
        GPIO.output(pin_right, GPIO.HIGH)
        
# end illuminateDirectionPins


# Turn off pins
def turnOff():
    print("Exiting gracefully")
    GPIO.output(PIN_INFO_LED, GPIO.LOW)
    GPIO.output(PIN_LT_PWM, GPIO.LOW)
    GPIO.output(PIN_RT_PWM, GPIO.LOW)
    GPIO.output(PIN_ERROR_LED, GPIO.LOW)
    GPIO.output(PIN_LT_POL_FWD, GPIO.LOW)
    GPIO.output(PIN_RT_POL_FWD, GPIO.LOW)
    GPIO.output(PIN_LT_POL_BWD, GPIO.LOW)
    GPIO.output(PIN_RT_POL_BWD, GPIO.LOW)
    GPIO.output(PIN_LT_LED, GPIO.LOW)
    GPIO.output(PIN_RT_LED, GPIO.LOW)
    print("Reset all control pins to LOW")


# Pull image from the camera. Only update commands when a new image comes  in
def getCameraFrame():
    return image

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


# Main loop.  This is looped over external to the function
def mainLoop(loopCount, pig):
   
    # Determine the desired steering angle in degrees and speed from 0 
    # (stopped), to 1 (full speed).
    # 0.0 is straight, - is left, + is right

    # Test: S-turns
    cycleLoops = 2000
    lmc = loopCount % cycleLoops
    if lmc < 0.00*cycleLoops:
        desiredSteerAngle_deg = 80.0
    elif (lmc > 0.0*cycleLoops) and (lmc < 1.0*cycleLoops):
        desiredSteerAngle_deg = -79.0
    else:
        desiredSteerAngle_deg = 0.0
    desiredSpeed = 1.0 # hardcode for now. Done by algorithm later.

    # Calculate desired steering and angle command from the image content
    #image = getCameraFrame()
    #desiredSteerAngle_deg, desiredSpeed = calculateCommand(image)

    # Get user input from keyboard (if any) and translate to angle and speed.
    # User commands (if any) will override the automatic commands for safety.
    #override, userSteer, userDuty = getUserCmd()    
    #if override:
    #    print 'USER OVERRIDE'
    #    desiredSteerAngle_deg = userSteer
    #    desiredSpeed = userDuty

    
    # Error check the steering angle and speed for validity
    if np.abs(desiredSteerAngle_deg) > 90.0:
        error("ERROR: Steering angle (" + desiredSteerAngle_deg + ") too large!")
    if np.abs(desiredSpeed) > 1.0:
        error("ERROR: Commanded speed (" + desiredSpeed + ") too large!")
    
    # Convert the desired angle and speed into PWM duty factors for the left
    # and right wheel motors.
    leftDuty, rightDuty = angleSpeedToDuty(desiredSteerAngle_deg, desiredSpeed)
    

    # Print status
    if loopCount % 1000 == 0:
        print '#', loopCount, ' (angle, speed, lduty, rduty) = (', desiredSteerAngle_deg, \
            ', ', desiredSpeed, ', ', leftDuty, ', ', rightDuty, ')'

    # Command the PWM input to the motor drive transistors
    commandPWM(pig, leftDuty, PIN_LT_PWM)
    commandPWM(pig, rightDuty, PIN_RT_PWM)
    
    # Command motor direction (polarity of drive)
    commandMotorPolarity(leftDuty > 0.0, PIN_LT_POL_FWD)
    commandMotorPolarity(leftDuty <= 0.0, PIN_LT_POL_BWD)
    commandMotorPolarity(rightDuty > 0.0, PIN_RT_POL_FWD)
    commandMotorPolarity(rightDuty <= 0.0, PIN_RT_POL_BWD)
    
    # Illuminate LED(s) indicating desired drive direction (informational)
    illumnateDirectionPins(leftDuty, rightDuty, PIN_LT_LED, PIN_RT_LED)
    
    return loopCount
# end mainLoop


# Main execution program.  The intent is for this to be running in
# the background at all times that the pi is on.  There is a separate
# physical switch for the drive motors that can be used to prevent the car
# from actually moving while this program is still running in the background.
def main():

    # Action to take at program exit
    atexit.register(turnOff)
    
    # Execute any necessary instantiation routines
    pig = setup()

    
    # Main execution loop
    loopCount = 0
    while True:
        loopCount = mainLoop(loopCount, pig)

        loopCount += 1
        #sleep(0.0) # optional pause statement


# Actually run the program
main()
