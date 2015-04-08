#!/bin/bash
# Plays all the relevant dn1 bags for this day (1/16/15).

if [ "$#" -eq  "0" ]
	then 
		printf '\n%s\n\n' "No speed specified, the bags will be played back at normal speed. (1x)"
		rosbag play dn1/dn1_2015-01-16-12-31-47_0.bag; rosbag play dn1/dn1_2015-01-16-12-48-41_1.bag; rosbag play dn1/dn1_2015-01-16-13-08-25_2.bag; rosbag play dn1/dn1_2015-01-16-13-26-16_3.bag
else
	printf '\n%s\n\n' "Playing the bags back at "$1"x speed."
	rosbag play -r $1 dn1/dn1_2015-01-16-12-31-47_0.bag; rosbag play -r $1 dn1/dn1_2015-01-16-12-48-41_1.bag; rosbag play -r $1 dn1/dn1_2015-01-16-13-08-25_2.bag; rosbag play -r $1 dn1/dn1_2015-01-16-13-26-16_3.bag
fi 
