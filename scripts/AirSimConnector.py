#!/usr/bin/env python

# Created on Wed Feb 19 16:17:21 2020
# @author: Patrick Gavigan

import airsim
import threading

# Thread safe connector to the AirSim simulator. 
# Should be treated as a singleton.
class AirSimConnector:
    
    def __init__(self):
        self.connectToAirSim()
        self.sem = threading.Semaphore()
        
    # Use destructor to ensure safe shutdown
    def __del__(self):
        self.disconnectFromAirSim()        
        
    def connectToAirSim(self):
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.car_controls = airsim.CarControls()
        
    def disconnectFromAirSim(self):
        self.sem.acquire()
        self.client.enableApiControl(False)
        self.client.armDisarm(False)
        self.sem.release()
        
    def getGpsData(self):
        self.sem.acquire()
        data = self.client.getGpsData()
        self.sem.release()
        return data
    
    def getImuData(self):
        self.sem.acquire()
        data = self.client.getImuData()
        self.sem.release()
        return data
    
    def brake(self, data):
        self.sem.acquire()
        self.car_controls.brake = data
        self.client.setCarControls(self.car_controls)
        self.sem.release()

    def throttle(self, data):
        self.sem.acquire()
        self.car_controls.throttle = data
        self.client.setCarControls(self.car_controls)
        self.sem.release()

    def steering(self, data):
        self.sem.acquire()
        self.car_controls.steering = data
        self.client.setCarControls(self.car_controls)
        self.sem.release()
    
    def getSpeed(self):
        self.sem.acquire()
        speed = self.client.getCarState().speed
        self.sem.release()
        return speed
