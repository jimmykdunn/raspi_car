#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Control program for raspberry-pi driven RC car.  Program is intended to be
# run constantly on the raspberry pi that sits aboard the car. This file contains
# the functions for direct interaction with the autonomous vehicle via GPIO pins,
# plus the main execution loop.  Specifics of control command calculation and
# manual driving are relegated to imported files.

# import matplotlib.pyplot as plt # plotting
print("Importing libraries")
import numpy as np # basic math
from time import sleep # sleep statements
import RPi.GPIO as GPIO # general purpose output pin control library
import atexit
import pigpio
import picamera
import picamera.array
import user_control
import seeker
import io
from PIL import Image

# Parameters
STEER_SCALE = 1.0 # Calibrates how much to slow motors when turning
PWM_FREQ = 100 # Frequency of motor control PWM signal
PWM_VAL_MAX = 1e4 # Maximum value of PWM duty command
IMAGE_RES = (128, 96)  # (x,y) num pixels of captured imagery
IMAGE_ROTATE_ANGLE = 180  # Adjust for orientation of camera on car

# Pin mappings
print("Setting pin mappings")
PIN_INFO_LED = 4 #7 # Pin connecting to LED that confirms main program is running
PIN_LT_PWM = 12 #32 # Pin that controls the PWM for the left wheels (GPIO12, PWM0)
PIN_RT_PWM = 13 #33 # Pin that controlls the PWM for the right wheels (GPIO13, PWM1)
PIN_ERROR_LED = 17 #11 # Pin connecting to LED that indicates an error
PIN_LT_POL_FWD = 27 #13 # Pin for commanding drive direction (left wheels)
PIN_LT_POL_BWD = 22 #15 # Pin for commaiding drive direction (left wheels)
PIN_RT_POL_FWD = 23 #16 # Pin for commanding drive direction (right wheels)
PIN_RT_POL_BWD = 24 #18 # Pin for commaiding drive direction (right wheels)
PIN_LT_LED = 10 #19 # Pin for LED indicating left turn commanded
PIN_RT_LED = 9 #21 # Pin for LED indicating right turn commanded

# Inital setup script
def setup():
    print("Running initialization procedure") 
    
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
    print("Setting up pigpio library for PWM commands")
    pig = pigpio.pi()
    for pin in [PIN_LT_PWM, PIN_RT_PWM]:
        pig.set_mode(pin, pigpio.OUTPUT)
        pig.hardware_PWM(pin, PWM_FREQ, 0)
        pig.set_PWM_range(pin, PWM_VAL_MAX)

    # Blink all LEDs 5x to confirm that the program is starting, 
    # leave green one on to confirm main program is running.
    print("Blinking lights for visual confirmation of code running")
    for i in range(4):
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
def illumnateDirectionPins(angle, leftDuty, rightDuty, pin_left, pin_right):
    if leftDuty == 0.0 and rightDuty == 0.0:
	# No command. Do not illuminate lights.
        GPIO.output(pin_left, GPIO.LOW)
        GPIO.output(pin_right, GPIO.LOW)
        
    else:
        if angle > 0:
            # Car has been commanded to turn right
            GPIO.output(pin_left, GPIO.LOW)
            GPIO.output(pin_right, GPIO.HIGH)
            
        if angle < 0:
            # Car has been commanded to turn left
            GPIO.output(pin_left, GPIO.HIGH)
            GPIO.output(pin_right, GPIO.LOW)

        if leftDuty < 0.0 and rightDuty < 0.0:
            # Car has been commanded to reverse
            GPIO.output(PIN_ERROR_LED, GPIO.HIGH)
	else:
	    GPIO.output(PIN_ERROR_LED, GPIO.LOW)
            
        if angle == 0:
            # Car has been commanded to move perfectly straight
            GPIO.output(pin_left, GPIO.HIGH)
            GPIO.output(pin_right, GPIO.HIGH)
        
# end illuminateDirectionPins


# Turn off pins
def turnOff(camera):
    print("Exiting gracefully")
    camera.close()
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


# Defines a manually-generated script for how to drive the car.
def scripted_commands(loopCount):

    # Test: S-turns
    cycleLoops = 6000
    lmc = loopCount % cycleLoops
    if lmc < 0.5*cycleLoops:
        desiredSteerAngle_deg = 80.0
    elif (lmc > 0.5*cycleLoops) and (lmc < 1.0*cycleLoops):

        desiredSteerAngle_deg = -80.0
    else:
        desiredSteerAngle_deg = 0.0
    desiredSpeed = 1.0 # hardcode for now. Done by algorithm later.
    return desiredSteerAngle_deg, desiredSpeed


# Main loop.  This is looped over external to the function
def mainLoop(image, loopCount, pig):
   
    # Determine the desired steering angle in degrees and speed from 0 
    # (stopped), to 1 (full speed).
    # 0.0 is straight, - is left, + is right
    # Calculate desired steering and angle command from the image content
    desiredSteerAngle_deg, desiredSpeed = seeker.calculateCommand(image)

    # Run a scripted command set
    # desiredSteerAngle_deg, desiredSpeed = scripted_commands(loopCount)

    # Get user input from keyboard (if any) and translate to angle and speed.
    # User commands (if any) will override the automatic commands for safety.
    #override, userSteer, userDuty = user_control.getUserCmd()    
    #if override:
    #    print('USER OVERRIDE')
    #    desiredSteerAngle_deg = userSteer
    #    desiredSpeed = userDuty

    
    # Error check the steering angle and speed for validity
    if np.abs(desiredSteerAngle_deg) > 90.0:
        error("ERROR: Steering angle (", desiredSteerAngle_deg, ") too large!")
    if np.abs(desiredSpeed) > 1.0:
        error("ERROR: Commanded speed (", desiredSpeed, ") too large!")
    
    # Convert the desired angle and speed into PWM duty factors for the left
    # and right wheel motors.
    leftDuty, rightDuty = angleSpeedToDuty(desiredSteerAngle_deg, desiredSpeed)
    

    # Print status
    if loopCount % 1 == 0:
        print('#', loopCount, ' (angle, speed, lduty, rduty) = (', desiredSteerAngle_deg, \
            ', ', desiredSpeed, ', ', leftDuty, ', ', rightDuty, ')')

    # Command the PWM input to the motor drive transistors
    commandPWM(pig, leftDuty, PIN_LT_PWM)
    commandPWM(pig, rightDuty, PIN_RT_PWM)
    
    # Command motor direction (polarity of drive)
    commandMotorPolarity(leftDuty > 0.0, PIN_LT_POL_FWD)
    commandMotorPolarity(leftDuty <= 0.0, PIN_LT_POL_BWD)
    commandMotorPolarity(rightDuty > 0.0, PIN_RT_POL_FWD)
    commandMotorPolarity(rightDuty <= 0.0, PIN_RT_POL_BWD)
    
    # Illuminate LED(s) indicating desired drive direction (informational)
    illumnateDirectionPins(desiredSteerAngle_deg, leftDuty, rightDuty, PIN_LT_LED, PIN_RT_LED)
    
    return loopCount
# end mainLoop


# Main execution program.  The intent is for this to be running in
# the background at all times that the pi is on.  There is a separate
# physical switch for the drive motors that can be used to prevent the car
# from actually moving while this program is still running in the background.
def main():

    
    # Execute any necessary instantiation routines
    pig = setup()

    
    # Main execution loop
    loopCount = 0
    with picamera.PiCamera() as camera:

	# Action to take at program exit
        atexit.register(turnOff, camera)

	print("Booting camera")
	# Camera properties
        camera.resolution = IMAGE_RES # set resolution
        camera.rotation = IMAGE_ROTATE_ANGLE # camera is mounted upside down
        stream = io.BytesIO() # stream to store imagery
        sleep(2) # let the camera warm up for 2 sec

	print("Beginning image capture")
	# Capture images continuously at the natural framerate of the camera
        for foo in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
	    stream.seek(0) # reset to the start of the stream so we can read from it
            image = Image.open(stream)  # read image from stream as PIL image
	    frame = np.array(image)  # Trnasform PIL image into a numpy ndarray
            frame = np.transpose(frame, (1,0,2)) # transpose to (x,y) from (y,x)

	    # Process image frame and command car
            loopCount = mainLoop(frame, loopCount, pig)
	    stream.seek(0)
            loopCount += 1
            #sleep(0.0) # optional pause statement
	camera.close()

# Actually run the program
main()
