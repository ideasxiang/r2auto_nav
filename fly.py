import time
import RPi.GPIO as GPIO

wheel_pin = 23
#p=GPIO.PWM(wheel_pin,50)
GPIO.setmode(GPIO.BCM)
GPIO.setup(wheel_pin, GPIO.OUT)
choice = 1
p=GPIO.PWM(wheel_pin,50)

def main():
	try:
	   while (True):
	       GPIO.output(wheel_pin,1)
	       time.sleep(15)
	       GPIO.output(wheel_pin,0)
	       time.sleep(3)

	except KeyboardInterrupt:
	     p.stop() 
	     GPIO.cleanup()
