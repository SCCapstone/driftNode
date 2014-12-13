#!/usr/bin/env python

#********Imports********#
import rospy
from std_msgs.msg import String
from wifi_node.msg import WifiMsgs
#***********End 'O Imports************#

def callback(wifi_msg):	#wifi_msgs will become populated with the wifi message from the talker

    rospy.loginfo(rospy.get_caller_id())
    rospy.loginfo(wifi_msg)
    
def wifi_listener():

    rospy.init_node('wifi_listener', anonymous=True)	#declaring listener node
    rospy.Subscriber("wifi_chatter", WifiMsgs, callback)	#subsrcibing to wifi_chatter topic
    rospy.spin()	#keeping the listener alive
        
if __name__ == '__main__':
    wifi_listener()
