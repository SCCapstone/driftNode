#!/usr/bin/env python

#********WoooHooo!!! Imports!!********#
import os
import subprocess
import shlex

import rospy
from std_msgs.msg import String
#***********End 'O Imports************#

#**********Global Variables***********#

#*******End 'O Global Variables*******#


def wifi_talker():
	wifi_pub = rospy.Publisher('wifi_chatter', String, queue_size=10)  #declaring the publisher
	rospy.init_node('wifi_talker', anonymous=True)	#initializing the node
	r = rospy.Rate(10) # 10hz
	raw_str_data = ""


    	while not rospy.is_shutdown():	#pretty much just runs based on the frequency
		wifi_msg = String()	#declares the wifi message
		wifi_msg.data = str

		try:
			p1 = subprocess.Popen(["iwlist","wlan0","scan"],stdout=subprocess.PIPE)
			p2 = subprocess.Popen(["grep","Signal"], stdin=p1.stdout, stdout=subprocess.PIPE)
			raw_str_data = str(p2.stdout.readline())	#reading next line from the wifi process' raw data stream
			#rospy.loginfo(raw_str_data)	#printing raw data stream to ros console
			wifi_msg.data = raw_str_data
			wifi_pub.publish(wifi_msg)
			#rospy.loginfo(wifi_msg.data)	#printing raw data stream to ros console
		except subprocess.CalledProcessError as e:
			rospy.loginfo("error running wifi script")
		r.sleep()

if __name__ == '__main__':
	try:
		wifi_talker()
	except rospy.ROSInterruptException: pass
	
