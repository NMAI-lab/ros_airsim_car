#!/usr/bin/env python

# Created on Wed Feb 19 16:17:21 2020
# @author: Patrick Gavigan

from AirSimConnector import AirSimConnector
import rospy
import time

def tester():
    connector = AirSimConnector()

    speed = connector.getSpeed()
    print("speed: " + str(speed))
    
    gps = connector.getGpsData()
    print("GPS: " + str(gps))
    
    connector.steering(1)
    connector.throttle(1)
    
    time.sleep(10)
    
    connector.brake(1)
    
    

if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass