import RPi.GPIO as GPIO
import time

#Set GPIO pin numbers
echo_pin = 23
trig_pin = 14

#Set GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Set up GPIO pins
GPIO.setup(trig_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)

#Functiom to measure distance
def measure_distance():
	GPIO.output(trig_pin, True)
	time.sleep(0.00001)
	GPIO.output(trig_pin, False)
	while GPIO.input(echo_pin) == 0:
		pulse_start = time.time()
	while GPIO.input(echo_pin) == 1:
		pulse_end = time.time()
	pulse = pulse_end - pulse_start
	#Calculate distance
	distance = (pulse * 34300) / 2
	return distance

print(measure_distance())	
