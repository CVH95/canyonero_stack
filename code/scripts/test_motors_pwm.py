#!/usr/bin/python
# SIMPLE PWM TEST
# 6-pin Connection

import RPi.GPIO as gpio
import time

print("CANYONERO \n\n")
print("Simple PWM Test")
print("...")
time.sleep(3)

gpio.setmode(gpio.BCM)
    
# Motor pins 
gpio.setup(17, gpio.OUT)
gpio.setup(22, gpio.OUT)
gpio.setup(23, gpio.OUT)
gpio.setup(24, gpio.OUT)

#Enable pins
gpio.setup(16, gpio.OUT)
gpio.setup(20, gpio.OUT)

# Set PWM duty cycles
pwm_left = gpio.PWM(16, 100)
pwm_right = gpio.PWM(20, 100)
pwm_left.start(50)
pwm_right.start(50)

# Move
print("Canyonero moving forward")
print("...")
gpio.output(17, gpio.HIGH)
gpio.output(22, gpio.LOW)
gpio.output(23, gpio.HIGH)
gpio.output(24, gpio.LOW)
time.sleep(3)

gpio.cleanup()
print("\n\nEnd of the test")