import RPi.GPIO as GPIO
import time
from picamera2 import Picamera2
import birdeye
import cv2
import numpy as np
import datetime

IN1 = 22
IN2 = 27
IN3 = 17
IN4 = 23
camera = Picamera2()
camera_config = camera.create_still_configuration(main={"size": (1640, 1232)}, lores={"size": (640, 480)}, display="lores")
camera.configure(camera_config)
servoPin = 12
camera.start()
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

def rotate(num, interval):
	if num > 0:
		for i in range(num):
			GPIO.output(IN1, GPIO.LOW)	
			GPIO.output(IN2, GPIO.LOW)
			GPIO.output(IN3, GPIO.LOW)
			GPIO.output(IN2, GPIO.HIGH)
			time.sleep(interval / 1000)
			GPIO.output(IN1, GPIO.LOW)
			GPIO.output(IN2, GPIO.LOW)
			GPIO.output(IN3, GPIO.HIGH)
			GPIO.output(IN4, GPIO.HIGH)
			time.sleep(interval / 1000)
			GPIO.output(IN1, GPIO.LOW)
			GPIO.output(IN2, GPIO.LOW)
			GPIO.output(IN3, GPIO.HIGH)
			GPIO.output(IN4, GPIO.LOW)
			time.sleep(interval / 1000)
			GPIO.output(IN1, GPIO.LOW)
			GPIO.output(IN2, GPIO.HIGH)
			GPIO.output(IN3, GPIO.HIGH)
			GPIO.output(IN4, GPIO.LOW)
			time.sleep(interval / 1000)
			GPIO.output(IN1, GPIO.LOW)
			GPIO.output(IN2, GPIO.HIGH)
			GPIO.output(IN3, GPIO.LOW)
			GPIO.output(IN4, GPIO.LOW)
			time.sleep(interval / 1000)
			GPIO.output(IN1, GPIO.HIGH)
			GPIO.output(IN2, GPIO.HIGH)
			GPIO.output(IN3, GPIO.LOW)
			GPIO.output(IN4, GPIO.LOW)
			time.sleep(interval / 1000)
			GPIO.output(IN1, GPIO.HIGH)
			GPIO.output(IN2, GPIO.LOW)
			GPIO.output(IN3, GPIO.LOW)
			GPIO.output(IN4, GPIO.LOW)
			time.sleep(interval / 1000)
			GPIO.output(IN1, GPIO.HIGH)
			GPIO.output(IN2, GPIO.LOW)
			GPIO.output(IN3, GPIO.LOW)
			GPIO.output(IN4, GPIO.HIGH)
			time.sleep(interval / 1000)
	else:
		num = abs(num)
		for i in range(num):
			GPIO.output(IN1, GPIO.LOW)	
			GPIO.output(IN2, GPIO.LOW)
			GPIO.output(IN3, GPIO.LOW)
			GPIO.output(IN2, GPIO.HIGH)
			time.sleep(interval / 1000)
			GPIO.output(IN1, GPIO.HIGH)
			GPIO.output(IN2, GPIO.LOW)
			GPIO.output(IN3, GPIO.LOW)
			GPIO.output(IN4, GPIO.HIGH)
			time.sleep(interval / 1000)
			GPIO.output(IN1, GPIO.HIGH)
			GPIO.output(IN2, GPIO.LOW)
			GPIO.output(IN3, GPIO.LOW)
			GPIO.output(IN4, GPIO.LOW)
			time.sleep(interval / 1000)
			GPIO.output(IN1, GPIO.HIGH)
			GPIO.output(IN2, GPIO.HIGH)
			GPIO.output(IN3, GPIO.LOW)
			GPIO.output(IN4, GPIO.LOW)
			time.sleep(interval / 1000)
			GPIO.output(IN1, GPIO.LOW)
			GPIO.output(IN2, GPIO.HIGH)
			GPIO.output(IN3, GPIO.LOW)
			GPIO.output(IN4, GPIO.LOW)
			time.sleep(interval / 1000)
			GPIO.output(IN1, GPIO.LOW)
			GPIO.output(IN2, GPIO.HIGH)
			GPIO.output(IN3, GPIO.HIGH)
			GPIO.output(IN4, GPIO.LOW)
			time.sleep(interval / 1000)
			GPIO.output(IN1, GPIO.LOW)
			GPIO.output(IN2, GPIO.LOW)
			GPIO.output(IN3, GPIO.HIGH)
			GPIO.output(IN4, GPIO.LOW)
			time.sleep(interval / 1000)
			GPIO.output(IN1, GPIO.LOW)
			GPIO.output(IN2, GPIO.LOW)
			GPIO.output(IN3, GPIO.HIGH)
			GPIO.output(IN4, GPIO.HIGH)
			time.sleep(interval / 1000)

def takecamera():
	rotate(64, 2)
	time.sleep(0.3)
	a2 = camera.capture_array("main")
	time.sleep(0.3)
	rotate(64, 2)
	time.sleep(0.3)
	a1 = camera.capture_array("main")
	time.sleep(0.3)
	rotate(-64, 2)
	rotate(-63, 2)
	time.sleep(0.3)
	a3 = camera.capture_array("main")
	time.sleep(0.3)
	rotate(-64, 2)
	time.sleep(0.3)
	a4 = camera.capture_array("main")
	time.sleep(0.3)
	rotate(-64, 2)
	time.sleep(0.3)
	a5 = camera.capture_array("main")
	time.sleep(0.3)
	rotate(64, 2)
	rotate(64, 2)
	return [a1, a2, a3, a4, a5]