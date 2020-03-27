import sys
import time
import RPi.GPIO as GPIO

from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

motor1 = 32
motor2 = 33
dc = 20

GPIO.setmode(GPIO.BOARD)
GPIO.setup(motor1, GPIO.OUT)
GPIO.setup(motor2, GPIO.OUT)

def normalize(min, max, new_min, new_max, value):
    new_value = ((value-min)/(max-min))*(new_max-new_min)+new_min

def PIDcontroller(x_input):
    #from motor_test (for running DC brushed motors with RPi4)
    #set target value for the car to center at
    target = 0
    Kp,Kd,Ki=.02,.01,.005

    x_error_sum = 0
    x_error = target - x_input
    x_prev_error = 0

    normx_error = normalize(-300,300,-1,1, x_error)
    normx_prev_error = normalize(-300,300,-1,1, x_prev_error)

    new_PWM += Kp*normx_error + Kd*normx_prev_error + Ki*x_error_sum
    if new_PWM >= 300:
        new_PWM = 300
    elif new_PWM <= -300:
        new_PWM = -300

    #new_PWM = normalize(0,600,0,100, new_PWM)

    #sleep(SAMPLETIME)
    x_prev_error=x_error
    x_error_sum+=x_error_sum
    print('new_PWM')
    return new_PWM

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
redLower = (170, 157, 64)
redUpper = (186, 255, 255)
# greenLower = (29, 86, 6)
# greenUpper = (64, 255, 255)

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
	vs = VideoStream(src=0).start()

# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(2.0)

# keep looping
while True:
	# grab the current frame
	frame = vs.read()

	# handle the frame from VideoCapture or VideoStream
	frame = frame[1] if args.get("video", False) else frame

	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
	if frame is None:
		break

	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = imutils.resize(frame, width=600)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, redLower, redUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None

	# only proceed if at least one contour was found
        if len(cnts) > 0:
	    # find the largest contour in the mask, then use
	    # it to compute the minimum enclosing circle and
	    # centroid
	    c = max(cnts, key=cv2.contourArea)
	    ((x, y), radius) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


	    # display position
	    xpos=center[0]
	    ypos=center[1]
            cv2.putText(frame, "X: {}, Y: {}".format(xpos,ypos), (10, 30), cv2.FONT_HERSHEY_SIMPLEX,0.65, (0, 0, 255), 2)

	    # draw line from point to center
	    cv2.rectangle(frame, (300, ypos+3), (xpos, ypos-3), (0, 0, 255), -1)
	    center_dis=xpos-300
	    #PIDcontroller(center_dis)
	    PWM = PIDcontroller(center_dis)
	    p1 = GPIO.PWM(motor1, PWM)
	    p1.start(0)
	    p2 = GPIO.PWM(motor2, PWM)
	    p2.start(0)

	    while True:
                p1.ChangeDutyCycle(dc)
		p2.ChangeDutyCycle(dc)
            GPIO.cleanup()
            sys.exit(0)
	    # only proceed if the radius meets a minimum size
	    if radius > 10:
		# draw the circle and centroid on the frame,
		# then update the list of tracked points
		cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
		cv2.circle(frame, center, 5, (0, 0, 255), -1)


	# draw center line of frame
	cv2.rectangle(frame, (297, 0), (303, 360), (0, 0, 255), -1)


	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break


# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
	vs.stop()

# otherwise, release the camera
else:
	vs.release()

# close all windows
cv2.destroyAllWindows()
