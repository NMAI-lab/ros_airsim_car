#!/usr/bin/env python

# Created on Mar 12 2020
# @author: Patrick Gavigan

import rospy
from std_msgs.msg import float64

def commnadBrake(data, airSimConnector):
    airSimConnector.brake(data.data)

def commandThrottle(data, airSimConnector):
    airSimConnector.throttle(data.data)
    
def commandSteering(data, airSimConnector):
    airSimConnector.steering(data.data)

def runActuators(airSimConnector):
    rospy.Subscriber('carCommnad/Brake', float64, commnadBrake, airSimConnector)
    rospy.Subscriber('carCommnad/Throttle', float64, commandThrottle, airSimConnector)
    rospy.Subscriber('carCommnad/Steering', float64, commandSteering, airSimConnector)