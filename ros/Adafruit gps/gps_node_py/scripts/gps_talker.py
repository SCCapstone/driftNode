#!/usr/bin/env python

#********WoooHooo!!! Imports!!********#
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from gps import *
import gps
#***********End 'O Imports************#

freq = 10	#ros looping frequency

gpsd = None	#instantiating gpsd
gpsd = gps.gps("localhost", "2947")	#gosd is boradcasting to default tcp port 2497
gpsd.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)	#gpsd stuff

def gps_talker():
    gps_pub = rospy.Publisher('gps_chatter', NavSatFix, queue_size=10)	#instantiating publisher
    rospy.init_node('gps_talker', anonymous=True)	#initializing node
    r = rospy.Rate(freq) # 10hz   


    while not rospy.is_shutdown():
	gps_msg = NavSatFix()	#instantiating message
	try:	#making sure gpsd isn't being obtuse
		report = gpsd.next()
		#print report
		gps_msg.latitude = gpsd.fix.latitude
		gps_msg.longitude = gpsd.fix.longitude
		gps_msg.altitude = gpsd.fix.altitude
		rospy.loginfo("[lat: %f long: %f alt: %f]", gps_msg.latitude, gps_msg.longitude, gps_msg.altitude)
		gps_pub.publish(gps_msg)	#publish ros message
		
	except Exception:	#yup, gpsd is being obtuse
		rospy.loginfo("No GPSD Data")
		pass

	r.sleep()	#wait until next time to run
        
if __name__ == '__main__':
	try:
		gps_talker()
	except rospy.ROSInterruptException: 
		rospy.loginfo("main except")
		pass
