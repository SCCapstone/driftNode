#!/usr/bin/env python

#********WoooHooo!!! Imports!!********#
import os
import subprocess

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
#***********End 'O Imports************#

#**********Global Variables***********#
freq = 10	#sensor polling frequency
seq = 0	#will keep track of the message sequence number
#*******End 'O Global Variables*******#


def imu_talker():
    global seq	#bringing seq into this scope
    imu_pub = rospy.Publisher('imu_chatter', Imu, queue_size=10)  #declaring the publisher
    rospy.init_node('imu_talker', anonymous=True)	#initializing the node
    global freq	#bringing freq into this scope
    r = rospy.Rate(freq)	#frequency in hz

    while not rospy.is_shutdown():	#pretty much just runs based on the frequency
	imu_msg = Imu()	#declares the imu message

	try:
		#raw_data_str = subprocess.check_output(["/home/pi/ros_catkin_ws/src/imu_node/scripts/ahrs_raw.sh"])
		raw_data_str = process.stdout.readline()	#reading next line from the imu process' raw data stream
		rospy.loginfo(raw_data_str)	#printing raw data stream to ros console
	except subprocess.CalledProcessError as e:
		rospy.loginfo("error running imu script")

	raw_vals = []	#array that imu data from raw data stream will be read into
	for t in raw_data_str.split():
		try:
			raw_vals.append(float(t))
			#rospy.loginfo(float(t))

		except ValueError: pass
	
	if len(raw_vals) >= 9:	#making sure we got all 9 values

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
		imu_msg.header.stamp.secs = time.secs	#time in seconds
		imu_msg.header.stamp.nsecs = time.nsecs #time in nanoseconds

		#rospy.loginfo(imu_msg)
		imu_pub.publish(imu_msg)	#publish imu message to the ros topic (imu_chatter)
		seq += 1	#incrementing sequence number
	else:
		rospy.loginfo("error parsing values")

	r.sleep()	#ros goes night night because of frequency stuff
        
if __name__ == '__main__':
    try:
	process = subprocess.Popen(["minimu9-ahrs", "--mode", "raw", "-b", "/dev/i2c-1"],
		stdout=subprocess.PIPE)	#runs an instance of minimu9-ahrs that we will grab the raw data stream from
	imu_talker()

    except rospy.ROSInterruptException: 
	rospy.loginfo("main except")
	pass