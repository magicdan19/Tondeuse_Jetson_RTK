#!/usr/bin/env python

"""
GpsPart.py 
it is donkeycar part for GPSRTK module.
This park will polls GPS data and sends to Vehicle.py
@authors: Chanyang Yim
"""

import numpy as np
import serial
import pynmea2
import time

class GpsRTK():
    def __init__(self,port,baudRate,timeOut):
        #will get gps data 
        self.gpsData = []
        self.running = True
        self.gpsSer = serial.Serial(port=port,baudrate = baudRate,timeout = timeOut)
        self.current_longitude = 0
        self.current_latitude = 0
    
    # return current location to Vehicel 
    def run_threaded(self):
        #return 32.865352333333334, -117.209856
        return self.current_latitude, self.current_longitude

    # will get gps data from GPS-RTK
    def poll(self):
        gpsMsg = self.gpsSer.readline().decode('utf-8')
        if "$GNRMC" not in gpsMsg:
            return
        gpsCoord = pynmea2.parse(gpsMsg)
        self.current_longitude = gpsCoord.longitude
        self.current_latitude = gpsCoord.latitude
        #self.current_longitude = self.convertDegToRad(gpsCoord.longitude)
        #self.current_latitude = self.convertDegToRad(gpsCoord.latitude)
   
    def update(self):
        while self.running:
            self.poll()

    def shutdown(self):
        self.running = False
    
    def convertDegToRad(self,val):
        return float(np.radians(val))
