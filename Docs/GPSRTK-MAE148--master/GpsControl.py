#!/usr/bin/env python

"""
GpsControl
it is donkeycar part for controling gps data.
This park will polls x,y coordinate from input values: lat and longs from GpsRTK part
@authors: Chanyang Yim
"""

import numpy as np

class GpsControl():
    def __init__(self,longitude,latitude):
        
        self.running = False
        
        self.base_longitude = None
        self.base_latitude = None
        
        self.curr_longitude = longitude
        self.curr_latitude = latitude

        self.x = 0
        self.y = 0
        #self.distance = 0

    # will get gps data from GPS-RTK
    """
    def poll(self):
   
    def update(self):

        while self.running:
            self.poll()

        if self.base_longitude is None or self.base_latitude is None:
            self.base_longitude = self.curr_longitude
            self.base_latitude = self.curr_longitude
            self.x = 0
            self.y = 0
            self.running = True
    """
    # poll and update need?? 
    def run(self):

        if self.base_longitude is None or self.base_latitude is None:
            self.base_longitude = self.curr_longitude
            self.base_latitude = self.curr_latitude
            self.x = 0
            self.y = 0
            return 0,0

        self.x,self.y = getXYFromGPS(self.curr_longitude,self.curr_latitude)
        return self.x, self.y

    def shutdown(self):
        return

    def getXYFromGPS(self,cur_lon,cur_lat):


