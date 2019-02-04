# Test piping raspivid video stream into python.
# Run raspivid -vf -w 640 -h 480 -o - -t 0 -b 2000000 > pipetest.py


import picamera
import picamera.array
import numpy as np
import io
import time
from PIL import Image


i = 0
with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)
    camera.rotation = 180
    stream = io.BytesIO()
    time.sleep(2) # let the camera warm up for 2 sec
    for foo in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
	print("TRYING FRAME NUMBER: ", i)
	stream.seek(0)
        image = Image.open(stream)  # read image from stream as PIL image
	frame = np.array(image)

        frame = np.transpose(frame, (1,0,2))
	#print("FRAME TYPE: ", type(frame))
	print("FRAME SHAPE: ", frame.shape)
	print("CAPTURED FRAME NUMBER: ", i)
	i+=1
    camera.close()