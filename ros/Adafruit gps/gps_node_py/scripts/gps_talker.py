#!/usr/bin/env python

#********WoooHooo!!! Imports!!********#
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix  #added gps_time string to the NavSatFix message until driftNode message package is complete
from gps import *
import gps
import numpy
import sys, os, subprocess
import calendar
from datetime import datetime
#***********End 'O Imports************#

time_set = False	#flag set to true if system time has been set
seq = 0
freq = 10	#ros looping frequency

gpsd = None	#instantiating gpsd
gpsd = gps.gps("localhost", "2947")	#gosd is boradcasting to default tcp port 2497
gpsd.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)	#gpsd stuff

def gps_talker():
    gps_pub = rospy.Publisher('gps_chatter', NavSatFix, queue_size=10)	#instantiating publisher
    rospy.init_node('gps_talker', anonymous=True)	#initializing node
    r = rospy.Rate(freq) # 10hz   
	
    while not rospy.is_shutdown():
	gps_msg = NavSatFix()	#instantiating gps message
	try:	#making sure gpsd isn't being obtuse
		report = gpsd.next()
		#print report
		if not numpy.isnan(gpsd.fix.altitude):
			global seq
			
			time = rospy.get_rostime()
			gps_time = datetime.strptime(gpsd.utc, '%Y-%m-%dT%H:%M:%S.%fZ')	#reading gps time and converting to datetime object for parsing
			#print(gps_time)
			gps_time_unix = calendar.timegm(gps_time.utctimetuple()) #converting utc time to unix time
			#print(gps_time_unix)
			
			global time_set
			if not time_set:
				pi_time = gps_time.strftime("%b %d %Y %H:%M:%S UTC")
				process = subprocess.Popen(["sudo", "date", "-s", pi_time])			
				#os.system('sudo date --set=string %s' % pi_time)
				time_set = True			
				rospy.loginfo("System time updated")
						
			gps_msg.header.seq = seq
						
			gps_msg.header.stamp = gps_time_unix
												
			gps_msg.latitude = gpsd.fix.latitude
			gps_msg.longitude = gpsd.fix.longitude
			gps_msg.altitude = gpsd.fix.altitude
			
			rospy.loginfo("[lat: %f long: %f alt: %f time: %i]", 
				gps_msg.latitude, gps_msg.longitude, gps_msg.altitude, gps_msg.header.stamp)

			gps_pub.publish(gps_msg)	#publish ros gps message
						
			seq +=1
		else:
			rospy.loginfo("GPSD is being obtuse.  Probably just no GPS fix yet.")
		
	except Exception:	#yup, gpsd is being obtuse
		rospy.loginfo("Exception when reading GPSD data.  The time is probably being stupid.")
		#print(sys.exc_info()[0])
		pass

	r.sleep()	#wait until next time to run
        
if __name__ == '__main__':
	try:
		gps_talker()
	except rospy.ROSInterruptException: 
		rospy.loginfo("main except")
		pass
