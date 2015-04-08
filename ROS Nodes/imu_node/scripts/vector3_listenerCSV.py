#!/usr/bin/env python

#********WoooHooo!!! Imports!!********#
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
#***********End 'O Imports************#

def callback(vector3_msg):	#imu_msgs will become populated with the imu message from the talker
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

	print str(vector3_msg.x) + ", " + str(vector3_msg.y) + ", " + str(vector3_msg.z)
	#rospy.loginfo(rospy.get_caller_id())
	rospy.loginfo(vector3_msg)
    
def vector3_listener():

	rospy.init_node('vector3_listener', anonymous=True)	#declaring listener node
    #rospy.Subscriber("/dn3/imu_chatter", Imu, callback)	#subsrcibing to imu_chatter topic
	rospy.Subscriber("/imu/rpy/filtered", Vector3, callback)	#subsrcibing to imu_chatter topic
	rospy.spin()	#WooHoo!! spinning 
        
if __name__ == '__main__':
    vector3_listener()
