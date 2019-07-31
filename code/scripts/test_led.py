#!/usr/bin/python
# LED TEST

import RPi.GPIO as gpio
import time

LED_R = 21
LED_L = 13

gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
gpio.setup(LED_R, gpio.OUT)
gpio.output(LED_R, False)
gpio.setup(LED_L, gpio.OUT)
gpio.output(LED_L, False)

# LED ON:
print("ON")
gpio.output(LED_R, True)
gpio.output(LED_L, True)
time.sleep(5)

# LED OFF
print("OFF")
gpio.output(LED_R, False)
gpio.output(LED_L, False)

print ("Finished test")