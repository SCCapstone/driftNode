#!/usr/bin/env python

#********WoooHooo!!! Imports!!********#
from gps import *
import gps
import numpy
import sys, os, subprocess
import calendar, time
from datetime import datetime
#***********End 'O Imports************#

gpsd = None	#instantiating gpsd
gpsd = gps.gps("localhost", "2947")	#gosd is boradcasting to default tcp port 2497
gpsd.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)	#gpsd stuff

print "starting to look for a GPS fix"

#while gpsd.fix.latitude == 0:
while ((gpsd.fix.mode < 2) or (numpy.isnan(gpsd.fix.track))) :
	report = gpsd.next()
	#print ""
	#print "waiting on a GPS fix"
	#print "the mode is"
	#print gpsd.fix.mode
	#print "we are waiting for a 2 or 3"
	#print ""
	#print "the GPS status is"
	#print ,
	#print ""
	#print "lat lon alt"
	#print gpsd.fix.latitude
	#print gpsd.fix.longitude
	#print gpsd.fix.altitude
	#print gpsd.utc, gpsd.fix.track, gpsd.fix.mode, gpsd.status, gpsd.fix.latitude, gpsd.fix.longitude, gpsd.fix.altitude
	#time.strftime("%I:%M:%s")

	#waiting 10 seconds before checking if there is a GPS fix
	#time.sleep(10)

#timecount = 0
#while timecount < 12:
#	report2 = gpsd.next()
#	print "timecount: " + str(timecount) + " gpsd.utc: " + str(gpsd.utc)
#	time.strftime("%I:%M:%s")
#	time.sleep(10)
#	timecount = timecount + 1	


print "got a GPS fix now setting the system time"

print gpsd.utc, gpsd.fix.track, gpsd.fix.mode, gpsd.status, gpsd.fix.latitude, gpsd.fix.longitude, gpsd.fix.altitude

#setting the system time


gps_time = datetime.strptime(gpsd.utc, '%Y-%m-%dT%H:%M:%S.%fZ')	#reading gps time and converting to datetime object for parsing
print "gps_time: " + str(gps_time)

gps_time_unix = calendar.timegm(gps_time.utctimetuple()) #converting utc time to unix time
print "gps_time_unix: " + str(gps_time_unix)

pi_time = gps_time.strftime("%b %d %Y %H:%M:%S UTC")
print "pi_time: " + str(pi_time)

process = subprocess.Popen(["sudo", "date", "-s", pi_time])
#process2 = subprocess.Popen(["date"])			
#os.system('sudo date --set=string %s' % pi_time)
		
print "System time updated"

exit(0)
