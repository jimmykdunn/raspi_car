#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Control program for raspberry-pi driven RC car.  Program is intended to be
# run constantly on the raspberry pi that sits aboard the car. This file contains
# the functions for direct interaction with the autonomous vehicle via GPIO pins,
# plus the main execution loop.  Specifics of control command calculation and
# manual driving are relegated to imported files.

# import matplotlib.pyplot as plt # plotting
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
PIN_RT_POL_FWD = 23 #16 # Pin for commanding drive direction (right wheels)
PIN_RT_POL_BWD = 24 #18 # Pin for commaiding drive direction (right wheels)
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


# Pull image from the camera immediately.
def getCameraFrame():
    with picamera.PiCamera() as camera:
        camera.resolution = (128, 96)
	#camera.start_preview()
        with picamera.array.PiRGBArray(camera) as image:
	    camera.capture(image, format='rgb')
    	    camera.capture('test.jpg')
            frame = image.array
    print(frame.shape)
    return frame


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
def mainLoop(loopCount, pig):
   
    # Determine the desired steering angle in degrees and speed from 0 
    # (stopped), to 1 (full speed).
    # 0.0 is straight, - is left, + is right
    # Calculate desired steering and angle command from the image content
    image = getCameraFrame()
    desiredSteerAngle_deg, desiredSpeed = seeker.calculateCommand(image)

    # Run a scripted command set
    #desiredSteerAngle_deg, desiredSpeed = scripted_commands(loopCount)

    # Get user input from keyboard (if any) and translate to angle and speed.