#!/usr/bin/python
# Copyright James Dunn (jkdunn@bu.edu), 2019. Boston University, MS ECE.
# Code written for ME/SE 740 (Intellegent Machines) term project.

# Test code for the autodriver

from time import sleep # sleep statements
import subprocess
import signal
import os


# Turns off the controller process via linux calls
def killController():
    # Find all active processes on the raspberry pi
    listprocess = subprocess.Popen(['ps', '-ef'], stdout=subprocess.PIPE)
    plist, err = listprocess.communicate()
    
    # Find the controller's line and pid and kill it
    for line in plist.splitlines():
        print(line)
        for word in line.split():
            if 'counter.py' == word:
                pid = int(line.split()[1]) # [1] is the pid of main.py
                print("Killing counter.py: ", pid)
                os.kill(pid, signal.SIGKILL)    
    
    print("Kill controller exiting")
    
# Turns on the controller process via linux call
def startController():
    #print("Start controller function")
    # Start a single instance of the controller process
    #exestr = "python counter.py"
    #DETACHED_PROCESS = 0x00000008
    #exeprocess = subprocess.Popen(exestr.split(), shell=False, stdin=None, stdout=None, stderr=None, creationflags=DETACHED_PROCESS)
    #print("execute counter.py")
    #exeprocess.spawnl()
    #print("started counter.py")
    print("execute counter.py")
    stuff = os.spawnl(os.P_NOWAIT, 'python', ['counter.py'])
    print(stuff)
    print("started counter.py")

# Executes a restart of the controller program
def resetController():         
    killController()
    sleep(2.0) # sleep to give time for the kill action to take
    startController()


def main():
    
    # Loop and don't ever exit
    while True:
        print("Executing loop")
        resetController()            
            
        sleep(10.0) # sleep to prevent overexecution
    # end while True

main()
