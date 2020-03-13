#!/usr/bin/env python

# Created on Mar 12 2020
# @author: Patrick Gavigan

import rospy
from std_msgs.msg import Float64

def commnadBrake(data, airSimConnector):
    rospy.loginfo("brake message: " + str(data.data))
    airSimConnector.brake(data.data)

def commandThrottle(data, airSimConnector):
    rospy.loginfo("throttle message: " + str(data.data))
    airSimConnector.throttle(data.data)
    
def commandSteering(data, airSimConnector):
    rospy.loginfo("steering message: " + str(data.data))
    airSimConnector.steering(data.data)

def runActuators(airSimConnector):
    rospy.Subscriber('carCommand/Brake', Float64, commnadBrake, airSimConnector)
    rospy.Subscriber('carCommand/Throttle', Float64, commandThrottle, airSimConnector)
    rospy.Subscriber('carCommand/Steering', Float64, commandSteering, airSimConnector)