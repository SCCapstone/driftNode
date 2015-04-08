#!/usr/bin/env python

#to use the listener pipe the output to a text file and then upload the txt file to http://www.gpsvisualizer.com/ to make a path on google maps
#for example use the command to listen to a bag
#rosrun gps_node_py gps_listener_AltLatLon.py > GpsOutput.txt

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from gps import gps

def callback(data):
	print str(data.altitude) + ", " + str(data.latitude) + ", " + str(data.longitude)
    
def gps_listener():

    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber("gps_chatter", NavSatFix, callback)
    rospy.spin()
        
if __name__ == '__main__':
    print "elevation, latitude, longitude"
    gps_listener()
