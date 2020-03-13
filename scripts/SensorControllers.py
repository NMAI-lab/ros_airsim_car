#!/usr/bin/env python

# Created on Mar 12 2020
# @author: Patrick Gavigan 

import rospy
from ros_airsim_drone.msg import GpsData
from std_msgs.msg import Float64

    
def runGPS(airSimConnector, publisher):
    # Get the data
    data = airSimConnector.getGpsData()
    
    # Prerpare the message
    message = GpsData()
    message.gnssReport.eph = data.gnss.eph
    message.gnssReport.epv = data.gnss.epv
    message.gnssReport.fixType = data.gnss.fix_type
    
    message.gnssReport.geoPoint.altitude = data.gnss.geo_point.altitude
    message.gnssReport.geoPoint.latitude = data.gnss.geo_point.latitude
    message.gnssReport.geoPoint.longitude = data.gnss.geo_point.longitude
    
    message.gnssReport.utcTime = data.gnss.time_utc
    
    message.gnssReport.velocity.x = data.gnss.velocity.x_val
    message.gnssReport.velocity.y = data.gnss.velocity.y_val
    message.gnssReport.velocity.z = data.gnss.velocity.z_val
    
    message.isValid = data.is_valid
    message.timeStamp = data.time_stamp
    
    # Publish the message
    rospy.loginfo(message)
    publisher.publish(message)
    
    
def runSpeed(airSimConnector, publisher):
    # Get the data
    data = airSimConnector.getSpeed()
    
    # Prerpare the message
    message = Float64()
    message.data = data
    
    # Publish the message
    rospy.loginfo(message)
    publisher.publish(message)
    

def runSensors(airSimConnector):

    # Initialize the publishers
    gpsPublisher = rospy.Publisher('carSensor/gps', GpsData, queue_size=10)
    speedPublisher = rospy.Publisher('carSensor/speed', Float64, queue_size=10)
    
    # Set the sensor publishing frequency
    rate = rospy.Rate(1) # 1hz

    # Publishing loop
    while not rospy.is_shutdown():
        runGPS(airSimConnector, gpsPublisher)
        runSpeed(airSimConnector, speedPublisher)
        rate.sleep()