#!/usr/bin/python
# SERVO MOTOR TEST (CAMERA PLATFORM)

import RPi.GPIO as gpio
import time

def pan_ToDutyCycle(theeta):
    # Min DTC = 2; Max DTC = 14
    m = 1/15
    dtcy = m*theeta + 2
    return dtcy

def tilt_toDutyCycle(theeta):
    # Min DTC = 0.5; Max DTC = 12
    m = 11.5/180
    dtcy = m*theeta + 0.5
    
gpio.setmode(gpio.BCM)
gpio.setup(18, gpio.OUT)
gpio.setup(25, gpio.OUT)
pan = gpio.PWM(18, 50)
tilt = gpio.PWM(25, 50)

print("Checking some values on the conversion function:")
m_0 = pan_ToDutyCycle(0)
m_45 = pan_ToDutyCycle(45)
m_90 = pan_ToDutyCycle(80)
m_120 = pan_ToDutyCycle(120)
m_180 = pan_ToDutyCycle(180)
print("Duty Cycle for 0 deg is: " + str(m_0))
print("Duty Cycle for 0 deg is: " + str(m_45))
print("Duty Cycle for 90 deg is: " + str(m_90))
print("Duty Cycle for 120 deg is: " + str(m_120))
print("Duty Cycle for 180 deg is: " + str(m_180))
time.sleep(3)

print("Starting the (base) servo. Going to 0 deg")
pan.start(m_90)
tilt.start(7.5)

try:
    while True:
        
        pan.ChangeDutyCycle(m_45)
        tilt.ChangeDutyCycle(11)
        #print("moved to 45 deg")
        time.sleep(3)
        pan.ChangeDutyCycle(m_90)
        tilt.ChangeDutyCycle(11)
        #print("moved to 90 deg")
        time.sleep(3)
        pan.ChangeDutyCycle(m_90)
        tilt.ChangeDutyCycle(9.5)
        #print("moved to 120 deg")
        time.sleep(3)
        pan.ChangeDutyCycle(m_120)
        tilt.ChangeDutyCycle(9.5)
        #print("moved to 180 deg")
        time.sleep(3)
        pan.ChangeDutyCycle(m_90)
        tilt.ChangeDutyCycle(11)
        #print("moved to 0 deg")
        #time.sleep(2)
        
except KeyboardInterrupt:
    pan.stop()
    tilt.stop()
    gpio.cleanup()
