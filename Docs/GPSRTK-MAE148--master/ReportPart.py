#!/usr/bin/env python

"""
ReportPart.py

it is donkeycar part which print out current status like gpsData, Steering,Throttling
need to change 
@authors: Chanyang Yim
"""
import time
class ReportPart():
    def __init__(self):
        print("Success to add ReportPart")
        self.last1= 0
        self.last2 = 0
    
    
    def run(self,lat,lot,steer,throt):
        if self.last1 is 0:
            self.last1 = lat
            self.last2 = lot
            return
        difflat = self.last1 - lat
        difflot = self.last2 - lot
        print("[GPS Data]: Lat, Lot",lat,lot)
        #print("[GPS Data]: Lat, Lot",difflat,difflot)
        #print("[CMD Data]: steer, throt",steer,throt)

        self.last1 = lat
        self.last2 = lot
        #time.sleep(1) 
    def shutdown(self):
        pass 
