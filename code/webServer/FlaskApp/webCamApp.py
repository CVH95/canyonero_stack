#!/usr/bin/python

#############
# CANYONERO #
#############

#### FLASK WEB SERVER FOR VIDEO STREAMING ####

from flask import Flask, render_template, Response
from camera_module import getStream
import time
import threading
import sys

# Define globals
_arg = sys.argv[1]
_ip_addr =  str(_arg)
_port = 5000
print("Canyonero's WebApp running on address: " + _ip_addr + ":" + str(_port))

# Init Web App
app = Flask(__name__)

@app.route("/")
def main():
    return render_template('index.html')

def gen():
    try:
        while(True):
            frame = getStream()
            yield(b'--frame\r\n'
                  b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    except KeyboardInterrupt:
        print("\n\n\n\n")
        print("Onboard camera down")

@app.route("/video_feed")
def video_feed():
    return Response(gen(),mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/info")
def online_info():
    return render_template('info_robot.html')
    

if __name__ == "__main__":
    try:
        app.run(host = _ip_addr, port = _port, threaded=True)
    except KeyboardInterrupt:
        print("\n\n\n\n")
        print("Canyonero's server shut down")