#!/usr/bin/env python

# Created on Mar 12 2020
# @author: Patrick Gavigan

import airsim
import threading

# Thread safe connector to the AirSim simulator. 
# Should be treated as a singleton.
class AirSimConnector:
    
    # Constructor, connect to AirSim, setup mutual exculsion
    def __init__(self):
        self.connectToAirSim()
        self.sem = threading.Semaphore()
        
    # Use destructor to ensure safe shutdown
    def __del__(self):
        self.disconnectFromAirSim()        
        
    # Connect to AirSim cleanly
    def connectToAirSim(self):
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.car_controls = airsim.CarControls()
        
    # Disconnect from AirSim cleanly
    def disconnectFromAirSim(self):
        self.sem.acquire()
        self.client.enableApiControl(False)
        self.client.armDisarm(False)
        self.sem.release()
        
    # Get the GPS data
    def getGpsData(self):
        self.sem.acquire()
        data = self.client.getGpsData()
        self.sem.release()
        return data
    
    # Get the speed data
    def getSpeed(self):
        self.sem.acquire()
        speed = self.client.getCarState().speed
        self.sem.release()
        return speed
    
    # Set the brake
    def brake(self, data):
        self.sem.acquire()
        self.car_controls.brake = data
        self.client.setCarControls(self.car_controls)
        self.sem.release()

    # Set the throttle
    def throttle(self, data):
        self.sem.acquire()
        self.car_controls.throttle = data
        self.client.setCarControls(self.car_controls)
        self.sem.release()

    # Set the steering
    def steering(self, data):
        self.sem.acquire()
        self.car_controls.steering = data
        self.client.setCarControls(self.car_controls)
        self.sem.release()
    

