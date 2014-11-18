#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from gps import gps

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+" I heard [lat: %f long: %f alt: %f]", data.latitude, data.longitude, data.altitude)
    
def gps_listener():

    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber("gps_chatter", NavSatFix, callback)
    rospy.spin()
        
if __name__ == '__main__':
    gps_listener()
