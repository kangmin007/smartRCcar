import time
from picamera2 import Picamera2, Preview
import pigpio
import serial

ser = serial.Serial('/dev/ttyAMA3', 9600)
camera = Picamera2()
camera_config = camera.create_still_configuration(main={"size": (1640, 1232)}, lores={"size": (640, 480)}, display="lores")
camera.configure(camera_config)
servoPin = 12
camera.start()
pi = pigpio.pi()
def cap(n):
    camera.capture_file(f"{n}.png")

def set_speed(speed):
	pulse_width = int(1450 + (speed *5))
	pi.set_servo_pulsewidth(servoPin, pulse_width)

def rotate(target_angle):
	speed = 30
	angle_time_map = {
	1: 0.155,
    2: 0.155,
	3: 0.155,
	4: -0.28,
	5: -0.28,
	6: -0.28,
	7: -0.28,
	8: -0.28,
	9: -0.28,
	10: 0.155,
	11: 0.155,
	12: 0.155
	}
	move_time = abs(angle_time_map[target_angle])
	direction = 1 if angle_time_map[target_angle] > 0 else -1
	set_speed(speed * direction)
	time.sleep(move_time)
	set_speed(0)

def shot():
    for i in range(1, 13):
        rotate(i)
        if i in [1, 2, 3, 6, 7, 8, 9]:
            time.sleep(0.25)
            cap(i)

while True:
     if ser.in_waiting > 0:
         code = int.from_bytes(ser.read(), "big")
         print(code)
         shot()
         ser.write(bytes([1, ]))

	
