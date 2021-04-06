#Author: Kartik Bharadwaj

from keyclipwriter import KeyClipWriter
from imutils.video import VideoStream
import argparse
import datetime
import imutils
import cv2
import numpy as np

import serial
import RPi.GPIO as GPIO      
import os, time

ap = argparse.ArgumentParser()
ap.add_argument("-a", "--min-area", type=int, default=1000, help="minimum area size")
ap.add_argument("-o", "--output", type=str,default="E:/Surveillance-Living Room",help="path to output directory")
ap.add_argument("-f", "--fps", type=int, default=20,help="FPS of output video")
ap.add_argument("-c", "--codec", type=str, default="MJPG",help="codec of output video")
ap.add_argument("-b", "--buffer-size", type=int, default=64,help="buffer size of video clip writer")
args = vars(ap.parse_args())

camera = VideoStream(0).start()
time.sleep(0.25)

fgbg = cv2.createBackgroundSubtractorMOG2(detectShadows = True)
kcw = KeyClipWriter(bufSize=args["buffer_size"])
consecFrames = 0
x = 0

######Raspberry Pi initialisation##########
GPIO.setmode(GPIO.BOARD)    

#Enabling Serial Communication
X = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

#Transmitting AT Commands to the Modem

X.write('AT'+'\r\n')
result = X.read(10)
print result
time.sleep(0.5)

X.write('ATE0'+'\r\n') 
result = X.read(10)
print result
time.sleep(0.5)

#Message as Text mode
X.write('AT+CMGF=1'+'\r\n')
result = X.read(10)
print result
time.sleep(0.5)

#SMS indications
X.write('AT+CNMI=2,1,0,0,0'+'\r\n') 
result = X.read(10)
print result
time.sleep(0.5)

# Sending message

X.write('AT+CMGS="9819745214"'+'\r\n')
result = X.read(10)
print result
time.sleep(0.5)

X.write('Emergency'+'\r\n')  # Message
result = X.read(10)
print result

############Raspberry Pi initialisation END#######

while True:
    frame = camera.read()
    text = "Living Room"
    
    frame = imutils.resize(frame, width=500)
    updateConsecFrames = True
    gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
    
    fgmask = fgbg.apply(gray)
    thresh = cv2.threshold(fgmask, 35, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=4)
    (_,cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    if x >= 3:
        for c in cnts:
            updateConsecFrames = cv2.contourArea(c) < args["min_area"]
                              
            if cv2.contourArea(c) > args["min_area"]:
                consecFrames = 0
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                text = "Living Room:Intruder"

		X.write("\x1A") # Enabling SMS
		for i in range(10):
			result = X.read(10)
			print result

                if not kcw.recording:
                    timestamp = datetime.datetime.now()
                    p = "{}/{}.avi".format(args["output"],timestamp.strftime("%Y%m%d-%H%M%S"))
                    kcw.start(p, cv2.VideoWriter_fourcc(*args["codec"]),args["fps"])
    else:
        x = x + 1

    if updateConsecFrames:
         consecFrames += 1

    kcw.update(frame)
    if kcw.recording and consecFrames == args["buffer_size"]:
         kcw.finish()

    cv2.putText(frame, "{}".format(text), (10, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(frame, datetime.datetime.now().strftime("%A %d %B %Y %I:%M:%S%p"),(10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
        
    cv2.imshow("Security Feed", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break
if kcw.recording:
    kcw.finish()
cv2.destroyAllWindows()
VideoStream(0).stop()