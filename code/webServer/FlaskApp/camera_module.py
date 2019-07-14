#!/usr/bin/python

#############
# CANYONERO #
#############

#### FLASK WEB SERVER CAMERA MODULE ####

import cv2

cap = cv2.VideoCapture(0)

def getStream():
    
    ret, frame = cap.read()
    #frame = cv2.resize(frame(600,480))
    return cv2.imencode('.jpg', frame)[1].tobytes()
    