#!/usr/bin/env python

#********WoooHooo!!! Imports!!********#
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
#***********End 'O Imports************#

def callback(imu_msg):	#imu_msgs will become populated with the imu message from the talker
     #this all commented out for now while we're playing around with what values from the imu go where
#    rospy.loginfo(rospy.get_caller_id()+
#	" I heard: [\nang_vel: %f %f %f  \nlin_accel: %f %f %f \norient: %f %f %f]\n", 
#	imu_msg.angular_velocity_covariance[0], 
#	imu_msg.angular_velocity_covariance[4], 
#	imu_msg.angular_velocity_covariance[8],

#	imu_msg.linear_acceleration_covariance[0], 
#	imu_msg.linear_acceleration_covariance[4],
#	imu_msg.linear_acceleration_covariance[8],

#	imu_msg.orientation_covariance[0],
#	imu_msg.orientation_covariance[4],
#	imu_msg.orientation_covariance[8])

	print str(imu_msg.vector[0]) + ", " + str(imu_msg.vector[4]) + ", " + str(imu_msg.vector[8])
	#rospy.loginfo(rospy.get_caller_id())
	#rospy.loginfo(imu_msg)
    
def imu_listener():

	rospy.init_node('imu_listener', anonymous=True)	#declaring listener node
    #rospy.Subscriber("/dn3/imu_chatter", Imu, callback)	#subsrcibing to imu_chatter topic
	rospy.Subscriber("/imu/rpy/filtered", Imu, callback)	#subsrcibing to imu_chatter topic
	rospy.spin()	#WooHoo!! spinning 
        
if __name__ == '__main__':
    imu_listener()
