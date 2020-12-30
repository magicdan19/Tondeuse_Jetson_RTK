#!/usr/bin/env
import random
import utm
import math
import time
from numpy import pi, cos, sin, arctan2, sqrt, square, radians
"""
Controller.py

It is donkeycar part for controlling the rc car 
it would get GPS data and return throller and stearing to control rc car
@authors:Chanyang Yim
"""

class Controller():

    def __init__(self,path):
        print("Controller part start...") 
        
        self.angle_L = 278.04 + 0.1
        self.angle_R = 278.47 - 0.1
        # will be middle point
        self.angle_cmd = (self.angle_R + self.angle_L)/2 
        
        self.angle_cmd_save = self.angle_cmd

        self.throttle_Max = 0.20
        self.throttle_cmd = 0.050
        #self.throttle_cmd = 0.2
        self.throttle_cmd_save = 0.20

        self.prev_loc = [None,None]
        self.cur_loc = self.prev_loc
        
        self.path_locs = path
        self.path_index = 0 
        self.goal_loc = path[0]
        
        self.init_pos = [None,None]
        self.prev_pos = self.init_pos
        self.count = 0 
        self.dir = 0.03 
    
        self.steer_gain =0.2
        self.steer_I_gain = 0.000
        self.dt = 0.2   #200ms
        self.sumI = 0

        self.isFirst = True
    def run2(self,lat,lon):
        angle = self.angle_cmd
        self.throttle_cmd = 0.08
        tt = input("throttle_cmd:")
        self.throttle_cmd = float(tt)
        print("[RUN] angle, thrott",angle,self.throttle_cmd)
        return angle,self.throttle_cmd

    def run(self,lat,lon):
        
        self.cur_loc = [lat,lon]
        if None in self.init_pos or 0 in self.init_pos:
            print("cur_loc1",self.cur_loc)
            self.init_pos = self.convertXYFromlatlon(self.cur_loc[0],self.cur_loc[1])
            self.prev_loc = self.cur_loc
            print("Controll:: init prev locationi (",self.prev_pos,")")
            return self.angle_cmd, self.throttle_cmd 
        
        d_prev =  self.dist_between_gps_points(self.cur_loc, self.prev_loc)
        if d_prev < 10 and self.isFirst:
            
            print('d_prev :',d_prev)
            
            if d_prev > 9:
                self.isFirst = False
                print("Controll: Prev location",self.prev_pos)
            
            return self.angle_cmd, self.throttle_cmd_save       
        
        
        #temp_cmd = self.steering_controller(self.cur_loc, self.prev_loc)
        temp_cmd = self.get_steer_cmd(self.cur_loc,self.prev_loc)  
        
        p_value = temp_cmd * self.steer_gain 
        i_value = self.steer_I_gain * temp_cmd * self.dt 
        self.sumI = self.sumI + i_value 
        temp_angle = self.angle_cmd + p_value + self.sumI
        
        print("----------------")
        print("p,v:",p_value,self.sumI)
        print("to index:",self.path_index)
        print("temp_cmd(p)",p_value, "deg:",math.degrees(p_value))
        if temp_angle < self.angle_L:
            #self.angle_cmd = self.angle_L
            print('max L')
            temp_angle = self.angle_L
        elif temp_angle > self.angle_R:
            #self.angle_cmd = self.angle_R
            print('max R')
            temp_angle = self.angle_R
        else:
            print("else case",temp_angle)
            #self.angle_cmd = temp_angle
        d_goal = self.update_distance()
        #temp_throttle = self.throttle_cmd
        if d_goal < 10:
            if self.path_index+1 == len(self.path_locs):
                print("DONONONONONONONNONONONONONONONONONONONOE")
                #self.path_index = 0
                return 111111
                #return self.angle_cmd, self.throttle_cmd
            else:
                print("Pass position:  ",self.path_index)
                self.path_index = self.path_index + 1 

            #input("enter to go next")
            #self.isFirst = True
        self.prev_loc = self.cur_loc
        
        #else:
        #temp_throttle = 0.14 
        #print(self.angle_cmd)
        #self.prev_pos = cur_pos
        print(self.cur_loc, "      ", self.prev_loc,"        :",d_prev)
        print("to goal [m]", d_goal)
        print("[Steer, Throttl]",temp_angle, self.throttle_cmd_save)
        #self.angle_cmd_save = temp_angle
        #self.throttle_cmd_save = temp_throttle 
        return temp_angle, self.throttle_cmd_save 
    
    def get_steer_cmd(self,cur_loc,prev_loc):
        
        prev_pos = self.convertXYFromlatlon(self.prev_loc[0],self.prev_loc[1])
        cur_pos = self.convertXYFromlatlon(self.cur_loc[0],self.cur_loc[1])
        next_pos = self.convertXYFromlatlon(self.path_locs[self.path_index][0],self.path_locs[self.path_index][1])
        u = self.getNormalVector(cur_pos,prev_pos)
        #u = [u[0] + cur_pos[0], u[1] +cur_pos[1]]
        v = self.getNormalVector(next_pos,cur_pos)
        #v = self.getNormalVector(next_pos,prev_pos)
        temp_steer =  self.getAngleBetweenTwoVector(u,v)
        return temp_steer

    def convertXYFromlatlon(self,lat,lon):
        tempx,tempy,_,_ = utm.from_latlon(lat,lon)
        
        if None in self.init_pos:
            return tempx,tempy

        x = tempx - self.init_pos[0]
        y = tempy - self.init_pos[1]
        print("temp x,y ",x,y)
        return [x,y]

    def getNormalVector(self,cur_pos,prev_pos):
        dy = cur_pos[1] - prev_pos[1]
        dx = cur_pos[0] - prev_pos[0]
        u = [dx,dy]
        return u

    def getAngleBetweenTwoVector(self,u,v):
        uv = u[0] * v[0] + u[1] * v[1]
        dim = math.sqrt(u[0]**2 + u[1]**2) * math.sqrt(v[0]**2 +v[1]**2)
        if dim == 0:
            return 0
        theta = math.acos(uv / dim)
        
        if u[0]*v[1] - u[1]*v[0] > 0:
            theta = -theta
        
        return theta


    def shutdown(self):
        pass


    def steering_controller(self, currLocation, prevLocation):
        """
        steering_controller()
        Method to implement a steering proportional controller for donkeycar
        @params: bearing_setpoint, bearing_current
        @return: steering command
        """
        # TODO: add PID steering controller
        # Proportional controller
        bearingPrevToCurr = self.calc_bearing(prevLocation, currLocation)
        bearingCurrToGoal = self.calc_bearing(currLocation, self.goal_loc)
        self.bearing = bearingCurrToGoal - bearingPrevToCurr
        self.steer_cmd = self.steer_gain * self.bearing

        # hard limits
        """
        if self.steering_cmd > self.steering_right:
            self.steering_cmd = self.steering_right
        elif self.steering_cmd < self.steering_left:
            self.steering_cmd = self.steering_left
        """
        return self.steer_cmd

    def calc_bearing(self, pointA, pointB):
        """
        Method to calculate the bearing between two points A and B w.r.t. North
        @params: two gps points A and B (lat, long) (radians)
        @return: bearing from current location to goal (radians)
        """

        # extract lat and long coordinates
        lat1 = pointA[0]
        lon1 = pointA[1]
        lat2 = pointB[0]
        lon2 = pointB[1]

        diffLon = lon2 - lon1
        x = sin(diffLon) * cos(lat2)
        y = cos(lat1) * sin(lat2) - (sin(lat1)*cos(lat2)*cos(diffLon))

        initialBearing = arctan2(x, y)

        # remap from [-pi,pi] to [0, 2*pi] for compass bearing
        compassBearingRad = (initialBearing + 2*pi) % (2*pi)

        return compassBearingRad

    def dist_between_gps_points(self, pointA, pointB):
        """
        Method to calculate the straight-line approximation between two gps coordinates.
        Used for distances on the 10-1000m scale.
        @params: two gps points A & B (radians) defined by lat and long coordinates
        @return: distance between the two points in meters
        """

        # radius of earth (m)
        r_earth = 6371e3

        # extract lat and long coordinates
        lat1 = pointA[0]
        lon1 = pointA[1]
        lat2 = pointB[0]
        lon2 = pointB[1]

        dlat = lat2 - lat1  # change in latitude
        dlon = lon2 - lon1  # change in longitude

        dx = r_earth * dlon * cos((lat1+lat2)/2)
        dy = r_earth * dlat

        dist = sqrt(square(dx)+square(dy))  # straight line approximation

        return dist

    def update_distance(self):
        """
        Method to update the distanc from current location to goal.
        @params: currLocation, goalLocation
        @return: distance [m]
        """
        self.distance = self.dist_between_gps_points(self.cur_loc, self.path_locs[self.path_index])

        return self.distance

## saved 
        """
        if self.mode == 'a':
            self.throttle = self.throttle+0.01 
        elif self.mode == 'i':
            t = float(input("entre throttle:"))
            self.throttle = t 
        print("throttle: ",self.throttle)
        self.angle = ((278.47 + 278.04)/2)
        return self.angle, self.throttle
         
        if self.angle >= 278.47:
            self.dir = -self.dir
        if self.angle < 278.04:
            self.dir = -self.dir
           # self.angle = 278.0
        self.angle = self.angle + self.dir
       
    
        a = float(input("enter settering:"))
        self.angle = a
        #self.angle =0.6323740348521378
    
        print("run with angle: ",self.angle)
        """
