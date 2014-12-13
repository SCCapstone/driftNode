#!/usr/bin/env python


#********WoooHooo!!! Imports!!********#
import os
import subprocess
import shlex

import rospy
from wifi_node.msg import WifiData
from wifi_node.msg import WifiMsgs
from std_msgs.msg import String
#***********End 'O Imports************#

#**********Global Variables***********#

#*******End 'O Global Variables*******#

rospy.loginfo("WIFI NODE")


def wifi_talker():
	wifi_pub = rospy.Publisher('wifi_chatter', WifiMsgs, queue_size=10)  #declaring the publisher
	rospy.init_node('wifi_talker', anonymous=True)	#initializing the node
	r = rospy.Rate(10) # 10hz
	raw_str_data = ""


    	while not rospy.is_shutdown():	#pretty much just runs based on the frequency
		wifi_msg = String()	#declares a wifi String message
		wifi_temp_msg = WifiData() #create a temp wifi message
		wifi_msg_array = WifiMsgs() #create the array that holds all of the wifi signals
		wifi_msg.data = str

		try:
			p1 = subprocess.Popen(["iwlist","wlan0","scan"],stdout=subprocess.PIPE)
			p2 = subprocess.Popen(["grep","Address\\|ESSID\\|Signal"], stdin=p1.stdout, stdout=subprocess.PIPE)
			raw_str_data = str(p2.stdout.read())	#reading next line from the wifi process' raw data stream
			#rospy.loginfo(raw_str_data)	#printing raw data stream to ros console
			#rospy.loginfo(raw_str_data.count("Cell"))

			wifi_msg_array.msgCount = raw_str_data.count("Cell")
			wifiList = raw_str_data.split("Cell")
			for x in range(1, raw_str_data.count("Cell")):
				wifi_temp_msg = WifiData() #create a new temp msg
				#rospy.loginfo(wifiList[(x)]) #prints each cell that contains Address,ESSID,Quality,Signal
				#rospy.loginfo(wifiList[(x)][15:32])#prints the MAC addresses one by one
				wifi_temp_msg.Address = wifiList[(x)][15:32]
				#rospy.loginfo(wifiList[(x)][wifiList[(x)].find("\"")+1:wifiList[(x)].rfind("\"")])#prints the ESSID
				wifi_temp_msg.ESSID = wifiList[(x)][wifiList[(x)].find("\"")+1:wifiList[(x)].rfind("\"")]
				#rospy.loginfo(wifiList[(x)][wifiList[(x)].find("=")+1:wifiList[(x)].find("=")+8])#prints the Quality
				wifi_temp_msg.Quality = wifiList[(x)][wifiList[(x)].find("=")+1:wifiList[(x)].find("=")+8]
				#rospy.loginfo(wifiList[(x)][wifiList[(x)].rfind("=")+1:wifiList[(x)].rfind("=")+8])#prints the Signal
				wifi_temp_msg.Signal = wifiList[(x)][wifiList[(x)].rfind("=")+1:wifiList[(x)].rfind("=")+8]
				wifi_msg_array.wifiMsgs.append(wifi_temp_msg)
			
			#example to assign the WifiMsgs.wifiMsgs
			#wifi_temp_msg = WifiData()
			#wifi_temp_msg.Address = "this is an address"	
			#wifi_temp_msg.ESSID = "this is an essid"
			#wifi_temp_msg.Quality = "this is an quality"
			#wifi_temp_msg.Signal = "this is an signal"
			#wifi_msg_array.wifiMsgs.append(wifi_temp_msg)

			wifi_msg.data = raw_str_data
			wifi_pub.publish(wifi_msg_array)
			#rospy.loginfo(wifi_msg.data)	#printing raw data stream to ros console
		except subprocess.CalledProcessError as e:
			rospy.loginfo("error running wifi script")
		r.sleep()

if __name__ == '__main__':
	try:
		wifi_talker()
	except rospy.ROSInterruptException: pass
