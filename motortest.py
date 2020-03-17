#!/usr/bin/python2

import sys
import time
import RPi.GPIO as GPIO

motor1 = 32  
motor2 = 33  
dc = 20

GPIO.setmode(GPIO.BOARD)
GPIO.setup(motor1, GPIO.OUT)
GPIO.setup(motor2, GPIO.OUT)

p1 = GPIO.PWM(motor1, 500)
p1.start(0)

p2 = GPIO.PWM(motor2, 500)
p2.start(0)

while True:
    p1.ChangeDutyCycle(dc)
    p2.ChangeDutyCycle(dc)

GPIO.cleanup()
sys.exit(0)
