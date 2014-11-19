#!/usr/bin/env python

import os
import subprocess

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

seq = 0

def imu_talker():
    global seq
    imu_pub = rospy.Publisher('imu_chatter', Imu, queue_size=10)
    rospy.init_node('imu_talker', anonymous=True)
    r = rospy.Rate(10) # hz

    while not rospy.is_shutdown():
	imu_msg = Imu()

	try:
		raw_data_str = subprocess.check_output(["/home/pi/ros_catkin_ws/src/imu_node/scripts/ahrs_raw.sh"])
		#rospy.loginfo(raw_data_str)
	except subprocess.CalledProcessError as e:
		rospy.loginfo("error running imu script")

	raw_vals = []
	for t in raw_data_str.split():
		try:
			raw_vals.append(float(t))
			#rospy.loginfo(float(t))

		except ValueError: pass
	
	if len(raw_vals) >= 9:


		imu_msg.orientation.x = raw_vals[0]
		imu_msg.orientation.y = raw_vals[1]
		imu_msg.orientation.z = raw_vals[2]	
	
		imu_msg.angular_velocity.x = raw_vals[6]
		imu_msg.angular_velocity.y = raw_vals[7]
		imu_msg.angular_velocity.z = raw_vals[8]

		imu_msg.linear_acceleration.x = raw_vals[3]
		imu_msg.linear_acceleration.y = raw_vals[4]
		imu_msg.linear_acceleration.z = raw_vals[5]

		imu_msg.header.seq = seq
		time = rospy.get_rostime()
		imu_msg.header.stamp.secs = time.secs
		imu_msg.header.stamp.nsecs = time.nsecs 

		rospy.loginfo(imu_msg)
		imu_pub.publish(imu_msg)
		seq += 1
	else:
		rospy.loginfo("error parsing values")

	



	r.sleep()
        
if __name__ == '__main__':
    try:
	imu_talker()
    except rospy.ROSInterruptException: 
	rospy.loginfo("main except")
	pass
