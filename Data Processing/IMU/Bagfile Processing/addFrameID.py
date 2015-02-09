#!/usr/bin/env python

#####################################
#Adds Missing frame_id to imu 
#	messages of bagfiles
#
#Matt Farich
#####################################

#********WoooHooo!!! Imports!!********#
import sys
import rosbag
import std_msgs.msg
import sensor_msgs.msg
#***********End 'O Imports************#
in_bag = "input.bag"
out_bag = "output.bag"

if len(sys.argv) == 2:
	in_bag = str(sys.argv[1])
	out_bag = "fixed_" + in_bag
	print "Adding imu frame_ids to " + in_bag
elif len(sys.argv) == 3:
	in_bag = str(sys.argv[1])
	out_bag = str(sys.argv[2])
	print "Adding imu frame_ids to " + in_bag
elif len(sys.argv) > 3:
	sys.exit("ERROR: Too Many Arguments!")
else:
	print "Defaulting input to input.bag and output to output.bag"


with rosbag.Bag(out_bag, 'w') as outbag:
	target_count = 0
	other_count = 0
	prefix = "imu_"
	for topic, msg, t in rosbag.Bag(in_bag).read_messages():
		if topic == "/dn3/imu_chatter":
			#msg.header.frame_id = prefix + str(target_count)
			msg.header.frame_id = prefix
			outbag.write(topic, msg, t)
			target_count = target_count + 1
		else:
			other_count = other_count + 1
			outbag.write(topic, msg, t)
	print "fixed " + str(target_count) + " messages of " + str(other_count) + " total" 