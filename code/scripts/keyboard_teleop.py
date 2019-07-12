#!/usr/bin/python
# SIMPLE PWM TEST
# 6-pin Connection
# Keyboard Teleoperation

import RPi.GPIO as gpio
import time
import getch

def_speed_1 = 50
def_speed_2 = 50

print("CANYONERO \n\n")
print("Keyboard Teleoperation \n")
print("  >> Forward: w")
print("  >> Backward: s")
print("  >> Turn right (on spot): d")
print("  >> Turn left (on spot): a")
print("  >> Stop: b")
print("  >> Increase speed: k")
print("  >> Reduce speed: j")
print("  >> Exit program: p \n")
print("Default speed = 50%")

# Configuration
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
    
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
pwm_left.start(def_speed_1)
pwm_right.start(def_speed_2)

# Default start
gpio.output(17, gpio.LOW)
gpio.output(22, gpio.LOW)
gpio.output(23, gpio.LOW)
gpio.output(24, gpio.LOW)

while(1):
    
    #keyb = input() # Requires pressing enter after input key
    keyb = getch.getch()#.decode("utf-8").lower()
    
    if keyb == 'w':
        # FW
        gpio.output(17, gpio.HIGH)
        gpio.output(22, gpio.LOW)
        gpio.output(23, gpio.HIGH)
        gpio.output(24, gpio.LOW)
        keyb == 'x'
    elif keyb == 's':
        #BW
        gpio.output(17, gpio.LOW)
        gpio.output(22, gpio.HIGH)
        gpio.output(23, gpio.LOW)
        gpio.output(24, gpio.HIGH)
        keyb == 'x'
    elif keyb == 'a':
        #TL
        gpio.output(17, gpio.LOW)
        gpio.output(22, gpio.HIGH)
        gpio.output(23, gpio.HIGH)
        gpio.output(24, gpio.LOW)
        keyb == 'x'
    elif keyb == 'd':
        #TR
        gpio.output(17, gpio.HIGH)
        gpio.output(22, gpio.LOW)
        gpio.output(23, gpio.LOW)
        gpio.output(24, gpio.HIGH)
        keyb == 'x'
    elif keyb == 'k':
        #MoreVel
        def_speed_1 = def_speed_1 + 10
        if def_speed_1 > 100:
            def_speed_1 = 100
        def_speed_2 = def_speed_1
        pwm_left.start(def_speed_1)
        pwm_right.start(def_speed_2)
        print("Current speed: " + str(def_speed_1) + "%")
        keyb == 'x'
    elif keyb == 'j':
        #LessVel
        def_speed_1 = def_speed_1 - 10
        if def_speed_1 < 10:
            def_speed_1 = 10
        def_speed_2 = def_speed_1
        pwm_left.start(def_speed_1)
        pwm_right.start(def_speed_2)
        print("Current speed: " + str(def_speed_1) + "%")
        keyb == 'x'
    elif keyb == 'b':
        # Stop
        gpio.output(17, gpio.LOW)
        gpio.output(22, gpio.LOW)
        gpio.output(23, gpio.LOW)
        gpio.output(24, gpio.LOW)
        keyb == 'x'
    elif keyb == 'p':
        print("\n\nEnd of keyboard teleoperation.")
        gpio.cleanup()
        break
    
    else:
        print("This key has no function...")