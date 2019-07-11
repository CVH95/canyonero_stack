#!/usr/bin/python
# SIMPLE MOTOR TEST
# 4-pin Connection

import RPi.GPIO as gpio
import time

def init():
    gpio.setmode(gpio.BCM)
    gpio.setup(17, gpio.OUT)
    gpio.setup(22, gpio.OUT)
    gpio.setup(23, gpio.OUT)
    gpio.setup(24, gpio.OUT)
    
def move_forward(t_s):
    init()
    gpio.output(17, True)
    gpio.output(22, False)
    gpio.output(23, True)
    gpio.output(24, False)
    time.sleep(t_s)
    gpio.cleanup()
    
def move_backward(t_s):
    init()
    gpio.output(17, False)
    gpio.output(22, True)
    gpio.output(23, False)
    gpio.output(24, True)
    time.sleep(t_s)
    gpio.cleanup()

def turn_rigt_onSpot(t_s):
    init()
    gpio.output(17, True)
    gpio.output(22, False)
    gpio.output(23, False)
    gpio.output(24, True)
    time.sleep(t_s)
    gpio.cleanup()

def turn_left_onSpot(t_s):
    init()
    gpio.output(17, False)
    gpio.output(22, True)
    gpio.output(23, True)
    gpio.output(24, False)
    time.sleep(t_s)
    gpio.cleanup()
    
    
print("CANYONERO \n\n")
print("Simple Motor Test")
print("...")
time.sleep(3)

ti = 2

print("Canyonero moving forward")
print("...")
move_forward(ti)
time.sleep(0.5)
print("Canyonero moving backward")
print("...")
move_backward(ti)
time.sleep(0.5)

print("Canyonero turning right on spot")
print("...")
turn_rigt_onSpot(0.3)
time.sleep(0.5)

print("Canyonero moving forward")
print("...")
move_forward(ti)
time.sleep(0.5)
print("Canyonero moving backward")
print("...")
move_backward(ti)
time.sleep(0.5)

print("Canyonero turning left on spot")
print("...")
turn_left_onSpot(0.3)

print("\n\nEnd of the test")
