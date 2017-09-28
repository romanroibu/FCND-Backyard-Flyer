# -*- coding: utf-8 -*-

import numpy as np
import logger
import connection
import os
import frame_utils
from pymavlink import mavutil

class Drone:
    
    #This method will be provided to the students
    def __init__(self):
        self.global_position = np.array([None, None, None]) #Longitude, Latitude, Altitude
        self.global_home = np.array([None, None, None])
        self.motors_armed = False
        self.global_velocity = np.array([None, None, None])
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.heading = None
        self.mode = None
        self.connected = False
        
        self.in_mission = False
        
        #TODO: Change the log name
        logname = "navLog.txt"
        self.log = logger.Logger(os.path.join("Logs",logname))
		
		
    #Provided
    def connect(self, device):
        self.connection = connection.Connection(device, self.decode_gps_msg)
        self.connection.subscribe('HEARTBEAT',self.heartbeat_callback)        
    
    #Provided
    def disconnect(self):
        self.connection.disconnect()
        self.log.close()
        
    
    def calculate_box(self, home):
        
        return

    

    
    def heartbeat_callback(self, msg):
        self.connected = True
        self.motors_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        self.guided = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) != 0
    
    
    # Methods for the arming state
    #Sets the mode to guided, arms the vehicle (with checks) and save the home position as the position it is armed
    def arm_vehicle(self):
        #TODO: fill out this method
        
        self.connection.subscribe('GLOBAL_POSITION_INT',self.arming_callback)
        return   

    
    def arming_callback(self, msg):        
        # Trigger next state
        if self.motors_armed & self.guided:
            # Save the current position as global_position
            self.global_home[0] = float(msg.lon)/(10**7)
            self.global_home[1] = float(msg.lat)/(10**7)
            self.global_home[2] = float(msg.relative_alt)/1000
            self.unsubscribe('GLOBAL_POSITION_INT',self.arming_callback)
            self.takeoff()
        
    
    #Methods for the takeoff state
    #Command the vehicle to a specific height and trigger next state when the drone reaches the specified altitude
    def takeoff(self):
        # TODO: fill out this method
        
        self.subscribe('GLOBAL_POSITION_INT',self.takeoff_callback)
        return True
    
    def takeoff_callback(self, msg): 
        
        # Trigger next state
        if msg.relative_alt/1000 > 0.95*self.target_position[2]:
            self.unsubscribe('GLOBAL_POSITION_INT',self.takeoff_callback)
            self.all_waypoints = self.calculate_box(self.global_home)
            self.target_position = self.all_waypoints.pop()
            self.goto(self.target_position)
    
    #Methods for the Goto state
    # Command the vehicle to the target position, assign the target to self.target_position and trigger next state when the position is reached (within 1 m)
    def goto(self, target):
        #TODO: fill out this method
        
        self.subscribe('GLOBAL_POSITION_INT',self.goto_callback)
        return
    
    def goto_callback(self, msg):
        global_position = np.array((float(msg.lon)/(10**7),float(msg.lat)/(10**7),float(msg.relative_alt)/1000))
        local_position = frame_utils.global_to_local(global_position,self.global_home)
        
        # Trigger next state
        if(np.linalg.norm(local_position-self.target_position)<1.0):
            self.unsubscribe('GLOBAL_POSITION_INT',self.goto_callback)
            if len(self.all_waypoints>0):
                self.target_position = self.all_waypoints.pop()
                self.goto(self.target_positions)                
            else:
                self.land()
                
    
    # Methods for the landing state
    # Lands the vehicle in the current location and triggers disarming when the vehicle is on the ground
    def land(self):
        #TODO: fill out this method
        
        self.subscribe('GLOBAL_POSITION_INT',self.land_callback)
        return True

    def land_callback(self, msg):
        
        # Trigger next state
        if(np.abs(float(msg.relative_alt)/1000-self.target_position[2])<0.05 &
           float(msg.vz)<0.01):
            self.unsubscribe('GLOBAL_POSITION_INT',self.land_callback)
            self.disarm()
            
    
    #Disarming method callback
    #Disarms the vehicle, returns control to manual, and returns true when the motors report armed
    def disarm_vehicle(self):
        #TODO: fill out this method
        
        self.subscribe('HEARTBEAT',self.disarm_callback)
        return
    
    def disarm_callback(self, msg):
        self.motors_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        self.guided = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) != 0
        
        if ~self.motors_armed & ~self.guided:
            self.in_mission = False
            
    
        
            
    def init(self):
        if ~self.connected:
            print("Vehicle Not Connected")
        
        while True:
            if self.connected:
                print('Vehicle Connected')
                break
        
        # Set first state
        self.arm()
        
            
            
    def mainloop(self):
        
        self.init()       
        
        while True:
            if(~self.in_mission):
                break
            
        self.shutdown()
            
        
            
                
            
            



