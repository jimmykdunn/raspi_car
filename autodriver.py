#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Code that is intended to run immediately at startup and continue
# running until shutdown. 
# Blinks lights/plays a simple tone to indicate beginning of execution.
# Pressing a button on the car chassis will start/restart the code ad
# infinitum.
# Pressing another button on the car chassis will execute a soft reboot of
# the raspberry pi.

import atexit
from time import sleep # sleep statements
import RPi.GPIO as GPIO # general purpose output pin control library
import subprocess
import signal
import os

PIN_STATUS_LED = 16 # Pin for software active LED
PIN_RESET_RPI = 14 # Pin connected to button for resetting the raspberry pi
PIN_RESET_CONTROL = 21 # Pin connected to button for resetting control SW

CONTROLLER_STR = "main.py" # what the controller software shows as in ps -ef


# Function to run when this program ends (intentionally or otherwise)
def scriptExit():
    # Turn off software-active indication LED
    GPIO.output(PIN_STATUS_LED, GPIO.LOW)
    
    
# Executes a soft shutdown of the raspberry pi
def softShutdown():
    # Blink the status pin to indicate shutdown initiated
    GPIO.output(PIN_STATUS_LED, GPIO.LOW)
    sleep(0.4)
    GPIO.output(PIN_STATUS_LED, GPIO.HIGH)
    sleep(0.4)
    GPIO.output(PIN_STATUS_LED, GPIO.LOW)
    sleep(0.4)
    GPIO.output(PIN_STATUS_LED, GPIO.HIGH)
    sleep(0.4)
    GPIO.output(PIN_STATUS_LED, GPIO.LOW)
    
    # Execute a soft shutdown
    print("Executing soft shutdown")
    killstr = "sudo shutdown now"
    killprocess = subprocess.Popen(killstr.split(), stdout=subprocess.PIPE)
    killprocess.communicate()[0]
    
    # Do not automatically kill this program, let the shutdown do that
    
    
# Turns off the controller process via linux calls
def killController():
    # Find all active processes on the raspberry pi
    listprocess = subprocess.Popen(['ps', '-ef'], stdout=subprocess.PIPE)
    plist, err = listprocess.communicate()
    
    # Find the controller's line and pid and kill it
    for line in plist.splitlines():
        line = line.decode("utf-8")
        if "main.py" in line:
            #print(line)
            pid = int(line.split()[1]) # [1] is the pid of main.py
            print("Killing controller process: ", pid)
            os.kill(pid, signal.SIGKILL)    
    
    print("Kill controller exiting")
    
# Turns on the controller process via linux call
def startController():
    # Start a single instance of the controller process
    print("execute main.py")
    sleep(0.1)
    stuff = os.spawnl(os.P_NOWAIT, '/usr/bin/python', 'python', '/home/pi/car/raspi_car/main.py')
    #print(stuff)
    print("started main.py")
    
    
# Executes a restart of the controller program
def resetController(): 
    sleep(1.0) # sleep to avoid race condition      
    killController() # should happen already in the controller, redo to make sure
    sleep(2.0) # sleep to give time for the kill action to take
    startController()


def main():
    GPIO.setmode(GPIO.BCM)
    atexit.register(scriptExit)
    
    # Initialize pins
    GPIO.setup(PIN_STATUS_LED, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_RESET_RPI, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PIN_RESET_CONTROL, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    
    # Blink software-active indication LED quickly to confirm running
    for i in range(12):
	GPIO.output(PIN_STATUS_LED, GPIO.LOW)
	sleep(0.1)
	GPIO.output(PIN_STATUS_LED, GPIO.HIGH)
    	sleep(0.1)
    
    controlSWOn = False
    
    # Loop and don't ever exit
    while True:
        # True (High) indicates button has been pressed. Execute shutdown
        if GPIO.input(PIN_RESET_RPI) == False:
            softShutdown()
            
        # True (High) indicates button has been pressed. Kill/start control.
        if GPIO.input(PIN_RESET_CONTROL) == False:
            if not controlSWOn:
                startController() 
                
                # Blink once for confirmation
	        GPIO.output(PIN_STATUS_LED, GPIO.LOW)
	        sleep(0.2)
	        GPIO.output(PIN_STATUS_LED, GPIO.HIGH)
	        
	        sleep(1.0) # in case button is pressed for too long
	        
	        controlSWOn = True
	    else:
	        sleep(1.0) # avoid race condition with controller itself
	        killController() # likely duplicate with controller kill
	        
	        # Blink twice for confirmation
	        GPIO.output(PIN_STATUS_LED, GPIO.LOW)
	        sleep(0.2)
	        GPIO.output(PIN_STATUS_LED, GPIO.HIGH)
	        sleep(0.2)
	        GPIO.output(PIN_STATUS_LED, GPIO.LOW)
	        sleep(0.2)
	        GPIO.output(PIN_STATUS_LED, GPIO.HIGH)
	        sleep(1.0)
	        
	        controlSWOn = False
	        
            
        sleep(0.1) # sleep to prevent overexecution
    # end while True
    
    # Should never get here. Turn off software-active indication LED
    scriptExit()

main()