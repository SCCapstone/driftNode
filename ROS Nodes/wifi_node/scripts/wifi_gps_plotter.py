#!/usr/bin/env python

"""
This Node will take in two driftnode names as input and plot the wifi signal strength vs distance in meters.

The format to run this node is: rosrun wifi_node wifi_gps_plotter.py "dnX" "dnY"

Where dnX is the node that you want to use that retrieved the wifi signal strength from its sensor and dnY is the node that dnX saw with the sensor

You must also have the topics playing already synchronized. To do this just load the two bags into rqt and be sure to right click
on the topics and say publish all.

Author: Zachary Smith

"""




#********Imports********#
import rospy
import message_filters
import numpy as np
import matplotlib.pyplot as plt
import sys
import random
from matplotlib import colors as col
from std_msgs.msg import String, Int32
from wifi_node.msg import WifiMsgs
from sensor_msgs.msg import NavSatFix
from math import radians, cos, sin, asin, sqrt

#***********End 'O Imports************#


#Global Variables 
#999 means it does not currently have a valid answer
globWifiStrength_node1 = 999 #will hold node1's wifi strength that it sees for node2
globWifiStrength_node2 = 999 #will hold node2's wifi strength that it sees for node1
globLatitude_node1 = 999 #will hold node1's latitude as it sees from its sensors
globLongitude_node1 = 999 #will hold node1's longitude as it sees from its sensors
globLatitude_node2 = 999 #will hold node1's latitude as it sees from its sensors
globLongitude_node2 = 999 #will hold node1's longitude as it sees from its sensors
globSampler = 12 #will sample the wifi strength after this many points


def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    meters = 6378100 * c
    return meters

def tryToPlot():
    global globLatitude_node1 
    global globLongitude_node1 
    global globLatitude_node2 
    global globLongitude_node2 
    global globWifiStrength_node1
    global globWifiStrength_node2
    global globSampler
    global dataFile

    distance = 0
    zoomFactor = 1000
    #We first check if any of the values are 999, if so they are junk and we shouldn't plot this point
    if (globWifiStrength_node1 != 999 and globLatitude_node1 != 999 and globLongitude_node1 != 999 and globLatitude_node2 != 999 and globLongitude_node2 != 999):
	distance = haversine(globLongitude_node1,globLatitude_node1,globLongitude_node2,globLatitude_node2)


	plt.subplot(2, 1, 1) #This plot will hold the scatter plot of wifi strength vs distance
	plt.title('Wifi Signal Strength Vs Distance (meters)') #Title the graph
        plt.xlabel('distance (meters)')                        #and set up the x
	plt.ylabel('Signal Strength (XXX/100)')		       #and y labels
	plt.scatter(distance,globWifiStrength_node1) #create a new point on the graph

	plt.subplot(2, 1, 2) #This plot will hold the map of gps coordinates and their distance/wifistrength
	#plt.title('GPS Map With Signal Strength') #Title the graph
        plt.xlabel('Longitude Zoom = '+str(zoomFactor)+' times')  #and set up the x
        plt.ylabel('Latitude Zoom = '+str(zoomFactor)+' times')   #and y labels
	plt.scatter(globLongitude_node1*zoomFactor, globLatitude_node1*zoomFactor, color = (0,1,0))
	plt.scatter(globLongitude_node2*zoomFactor, globLatitude_node2*zoomFactor, color = (1,0,0))
        if(int(random.random()*globSampler+1) == globSampler):
		plt.plot([globLongitude_node1*zoomFactor, globLongitude_node2*zoomFactor], [globLatitude_node1*zoomFactor, globLatitude_node2*zoomFactor], color = (0,0,0),linewidth = (globWifiStrength_node1/10+1))
		plt.annotate('d = '+str(int(distance))+'m', xy=(globLongitude_node1*zoomFactor, globLatitude_node1*zoomFactor), xytext=(globLongitude_node1*zoomFactor, globLatitude_node1*zoomFactor))
	plt.draw() #draw the point on the graph
	rospy.loginfo('Distance: '+str(distance)+'  SignalStrength: '+ str(globWifiStrength_node1))
	plt.savefig('./gpsWifiGraph.jpg')

	#For writing a csv file of all the data
	dataFile.write(str(globLatitude_node1)+','+str(globLongitude_node1)+','+
			str(globLatitude_node2)+','+str(globLongitude_node2)+','+
			str(globWifiStrength_node1)+','+str(globWifiStrength_node2)+','+
			str(distance)+'\n')

	#reset our values and wait for them to be updated again
	globLatitude_node1 = 999 
    	globLongitude_node1 = 999
	globLatitude_node2 = 999
    	globLongitude_node2 = 999
	globWifiStrength_node1 = 999
	globWifiStrength_node2 = 999


def callbackWifi1(wifi_msg):	#wifi_msgs will become populated with the wifi message from the talker
    global globWifiStrength_node1
    #rospy.loginfo(rospy.get_caller_id())
    #rospy.loginfo(wifi_msg.data)
    globWifiStrength_node1 = wifi_msg.data
    tryToPlot()

def callbackWifi2(wifi_msg):	#wifi_msgs will become populated with the wifi message from the talker
    global globWifiStrength_node2
    #rospy.loginfo(rospy.get_caller_id())
    #rospy.loginfo(wifi_msg.data)
    globWifiStrength_node2 = wifi_msg.data


def callbackGPS1(gps_data1):	#gps_data1 will become populated with the gps message from the talker
    global globLatitude_node1 
    global globLongitude_node1 
    #rospy.loginfo(rospy.get_caller_id())
    #rospy.loginfo(gps_data1.latitude)
    #rospy.loginfo(gps_data1.longitude)
    globLatitude_node1 = gps_data1.latitude
    globLongitude_node1 = gps_data1.longitude


def callbackGPS2(gps_data2):	#gps_data2 will become populated with the gps message from the talker
    global globLatitude_node2 
    global globLongitude_node2 
    #rospy.loginfo(rospy.get_caller_id())
    #rospy.loginfo(gps_data2.latitude)
    #rospy.loginfo(gps_data2.longitude)
    globLatitude_node2 = gps_data2.latitude
    globLongitude_node2 = gps_data2.longitude
    
def wifi_gps_plotter():

    rospy.init_node('wifi_gps_listener', anonymous=True)	#declaring listener node
    wifi_sub1 = message_filters.Subscriber('/'+sys.argv[1]+'/'+sys.argv[2]+'SignalStrength', Int32)
    wifi_sub2 = message_filters.Subscriber('/'+sys.argv[2]+'/'+sys.argv[1]+'SignalStrength', Int32)
    gps_sub1 = message_filters.Subscriber('/'+sys.argv[1]+'/gps_chatter', NavSatFix)
    gps_sub2 = message_filters.Subscriber('/'+sys.argv[2]+'/gps_chatter', NavSatFix)

    wifi_sub1.registerCallback(callbackWifi1)
    wifi_sub1.registerCallback(callbackWifi2)
    gps_sub1.registerCallback(callbackGPS1)
    gps_sub2.registerCallback(callbackGPS2)
		
    #ts = message_filters.TimeSynchronizer([wifi_sub, gps_sub1, gps_sub2], 10) // would work if wifi_talker had a header in the message
    #ts.registerCallback(callback)
    rospy.spin()	#keeping the listener alive


if __name__ == '__main__':
    dataFile = open('./GPSandWifiData.csv','w') #file to store the data

    dataFile.write('Latitude_'+sys.argv[1]+','+'Longitude_'+sys.argv[1]+','+
			'Latitude_'+sys.argv[2]+','+'Longitude_'+sys.argv[2]+','+
			'WifiStrength_'+sys.argv[1]+'to'+sys.argv[2]+','+'WifiStrength_'+sys.argv[2]+'to'+sys.argv[1]+','+
			'Distance (meters)'+'\n')

    plt.ion() #Makes the graph be able to be updated in real time
    plt.show(block = False) #Shows the graph and doesn't block the node from running
    wifi_gps_plotter()




