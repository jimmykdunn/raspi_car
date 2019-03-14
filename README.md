# raspi_car
Python code for autonomous control of an RC car with a raspberry pi

Run code by calling "python main.py" from a terminal. 
BE SURE TO RUN main.py FROM ITS NATIVE DIRECTORY! IT WILL NOT RUN CORRECTLY OTHERWISE!

# rc.local
Enable the camera on the raspberry pi configuration, the GPIO pins, and the autodriver script in your rc.local file by copying the contents of the rc.local file into your rc.local file.  This will ensure that your environment is correct.

# DEPENDENCIES (not included with default Raspbian install)
scipy: sudo apt-get install python-scipy

# OTHER DEPENDENCIES (usually included with default Raspbian install)
numpy
PIL
time
RPi.GPIO
atexit
pigpio
picamera
os
glob
datetime
subprocess
signal
matplotlib
struct
platform
