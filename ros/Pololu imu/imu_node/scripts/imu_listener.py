#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

def callback(imu_msg):
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

    rospy.loginfo(rospy.get_caller_id())
    rospy.loginfo(imu_msg)
    
def imu_listener():

    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber("imu_chatter", Imu, callback)
    rospy.spin()
        
if __name__ == '__main__':
    imu_listener()
