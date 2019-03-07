#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Test code for the autodriver

from time import sleep # sleep statements
import subprocess
import signal
import os
CONTROLLER_STR = "python counter.py" # what the controller software shows as in ps -ef
 
    
# Turns off the controller process via linux calls
def killController():
    # Find all active processes on the raspberry pi
    listprocess = subprocess.Popen(['ps', '-ef'], stdout=subprocess.PIPE)
    plist, err = listprocess.communicate()
    
    # Find the controller's line and pid and kill it
    for line in plist.splitlines():
        if CONTROLLER_STR in line:
            pid = int(line.split()[1]) # [1] is the pid of main.py
            os.kill(pid, signal.SIGKILL)  
    
    
# Turns on the controller process via linux call
def startController():
    # Start a single instance of the controller process
    exestr = CONTROLLER_STR
    exeprocess = subprocess.Popen(exestr.split(), stdout=subprocess.PIPE)
    exeprocess.communicate()
    
    
# Executes a restart of the controller program
def resetController():         
    killController()
    sleep(2.0) # sleep to give time for the kill action to take
    startController()


def main():
    
    # Loop and don't ever exit
    while True:
        resetController()            
            
        sleep(10.0) # sleep to prevent overexecution
    # end while True

main()