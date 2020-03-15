#!/usr/bin/env python

# Created on Mar 12 2020
# @author: Patrick Gavigan

# Tests the actuators for this node. Do not run as part of normal operation.

import rospy
import time
from std_msgs.msg import Float64

def tester():
    
    rospy.init_node('Tester', anonymous=True)
    
    brakePublisher = rospy.Publisher('carCommand/Brake', Float64, queue_size=10)
    throttlePublisher = rospy.Publisher('carCommand/Throttle', Float64, queue_size=10)
    steeringPublisher = rospy.Publisher('carCommand/Steering', Float64, queue_size=10)
    
    message = Float64()
    message.data = 1
    
    rospy.loginfo("starting test")
    
    time.sleep(10)
    rospy.loginfo("throttle message: " + str(message.data))
    throttlePublisher.publish(message)
    
    time.sleep(10)
    
    rospy.loginfo("steering message: " + str(message.data))
    steeringPublisher.publish(message)
    
    time.sleep(10)
    rospy.loginfo("brake message: " + str(message.data))
    brakePublisher.publish(message)
    
    rospy.loginfo("test complete")
    
    

if __name__ == '__main__':
    try:
        tester()
    except rospy.ROSInterruptException:
        pass
