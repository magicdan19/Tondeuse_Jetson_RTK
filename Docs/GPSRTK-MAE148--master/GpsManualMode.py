#!/usr/bin/env python

"""
GpsManualMode.py
it is donkeycar part for GPS simulation module.
This park will polls GPS data by user input as current robot position
@authors: Chanyang Yim
"""

import numpy as np
import serial
import pynmea2

class GpsManualMode():
    def __init__(self):
        pass
    # return current location to Vehicel 
    def run(self):
        lat, lon = input("Enter curr robot lat, lon: ").split(',')
        #lat,lon = 1,1
        print("[GPSManualMode]",lat,lon)
        return float(lat),float(lon)
    def shutdown(self):
        pass
