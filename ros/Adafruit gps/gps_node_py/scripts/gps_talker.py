#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from gps import *
import gps

gpsd = None
gpsd = gps.gps("localhost", "2947")
gpsd.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

def gps_talker():
    gps_pub = rospy.Publisher('gps_chatter', NavSatFix, queue_size=10)
    rospy.init_node('gps_talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    


    while not rospy.is_shutdown():
	msg = NavSatFix()
	#msg.latitude = 4.2
	#msg.longitude = 4.2
	#msg.altitude = 4.2
	try:
		report = gpsd.next()
		#print report
	except Exception: 
		rospy.loginfo("gpsd except")
		pass
		
	msg.latitude = gpsd.fix.latitude
	msg.longitude = gpsd.fix.longitude
	msg.altitude = gpsd.fix.altitude
	rospy.loginfo("[lat: %f long: %f alt: %f]", msg.latitude, msg.longitude, msg.altitude)
	gps_pub.publish(msg)

	r.sleep()
        
if __name__ == '__main__':
    try:
	gps_talker()
    except rospy.ROSInterruptException: 
	rospy.loginfo("main except")
	pass
