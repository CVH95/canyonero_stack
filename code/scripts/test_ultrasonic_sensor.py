#!/usr/bin/python
# ULTRASONIC SENSOR HC-SR40 TEST

import RPi.GPIO as gpio
import time

# Trig pin: GPIO 19
TRIGGER = 12
# Echo pin: GPIO 26
ECHO = 6
## Init ##
gpio.setmode(gpio.BCM)
gpio.setup(TRIGGER, gpio.OUT)
gpio.setup(ECHO, gpio.IN)
gpio.output(TRIGGER, False)
time.sleep(2)
print("Now HC-SR04 is up and running")

try:
    while True:
        
        print("Inside while loop")
        
        # Turn on TRIGGER
        gpio.output(TRIGGER, True)
        time.sleep(0.00001) # 10 us
        
        # Turn off TRIGGER
        gpio.output(TRIGGER, False)
        start = time.time()
        print("Trigger Out again")

        # Count
        while gpio.input(ECHO)==0:
            start = time.time()
          #  print("start = " + str(start))
        while gpio.input(ECHO)==1:
            end = time.time()
          #  print("end = " + str(end))
        past = end - start

        distance = past*34300/2
        #distance = past/58
        
        print("Closest object: " + str(distance*10) + " mm.")
        
        time.sleep(1)
except KeyboardInterrupt:
    gpio.cleanup()
    print("Finished test")

#def obstacle_detection():
#    
#    # Wait until HC-SR04 settles
#    #gpio.output(TRIGGER, gpio.LOW)
#    #time.sleep(0.000002) # 2 us
#    
#    # Turn on TRIGGER
#    gpio.output(TRIGGER, True)
#    time.sleep(0.00001) # 10 us
#    
#    # Turn off TRIGGER
#    gpio.output(TRIGGER, False)
#
#    # Count
#    while gpio.input(ECHO) == 0:
#        start = time.time()
#    while gpio.input(ECHO) == 1:
#        end = time.time()
#    past = end - start
#
#    past = past*34300/2
#    distance = past/58
#    
#    print("Closest object: " + str(distance*10) + " mm.")
#    
#    time.sleep(1)
#
##try:
##    while True:
##        obstacle_detection()

#except KeyboardInterrupt:
#    gpio.cleanup()
#    print("Finished test")
    
    