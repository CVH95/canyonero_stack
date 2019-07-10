#!/usr/bin/python
# SIMPLE PWM TEST

import RPi.GPIO as gpio
import time

def init():
    gpio.setmode(gpio.BCM)
    
    #Enable pins
    pwm_left = gpio.PWM(16, gpio.OUT)
    pwm_right = gpio.PWM(20, gpio.OUT)
    
    # Motor pins 
    gpio.setup(17, gpio.OUT)
    gpio.setup(22, gpio.OUT)
    gpio.setup(23, gpio.OUT)
    gpio.setup(24, gpio.OUT)
    pwm_left = gpio.PWM(16, 100)
    pwm_right = gpio.PWM(20, 100)

def move_forward(t_s):
    gpio.output(17, True)
    gpio.output(22, False)
    gpio.output(23, True)
    gpio.output(24, False)
    time.sleep(t_s)
    
# Set PWM duty cycles
init()
pwm_left.start(50)
pwm_right.start(20)

# Move
print("Canyonero moving forward")
print("...")
move_forward(10)

gpio.cleanup()
print("\n\n End of the test")



