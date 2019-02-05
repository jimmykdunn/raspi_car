# raspi_car
Python code for autonomous control of an RC car with a raspberry pi

# Be sure to enable the camera on the raspberry pi configuration
# You must also run this at bootup (add to /etc/rc.local):
sudo systemctl enable pigpiod.service
sudo pigpiod

# DEPENDENCIES (not included with default Raspbian install)
scipy: sudo apt-get install python-scipy