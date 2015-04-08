#!/bin/bash
# Plays all the relevant dn3 bags for this day (1/16/15).

if [ "$#" -eq  "0" ]
	then 
		printf '\n%s\n\n' "No speed specified, the bags will be played back at normal speed. (1x)"
		rosbag play dn3/dn3_2015-01-16-12-24-28_1.bag; rosbag play dn3_2015-01-16-12-37-25_2.bag; rosbag play dn3/dn3_2015-01-16-12-55-56_3.bag; rosbag play dn3/dn3_2015-01-16-13-15-36_4.bag; rosbag play dn3/dn3_2015-01-16-13-30-24_5.bag
else
	printf '\n%s\n\n' "Playing the bags back at "$1"x speed."
	rosbag play -r $1 dn3/dn3_2015-01-16-12-24-28_1.bag; rosbag play -r $1 dn3/dn3_2015-01-16-12-37-25_2.bag; rosbag play -r $1 dn3/dn3_2015-01-16-12-55-56_3.bag; rosbag play -r $1 dn3/dn3_2015-01-16-13-15-36_4.bag; rosbag play -r $1 dn3/dn3_2015-01-16-13-30-24_5.bag
fi
