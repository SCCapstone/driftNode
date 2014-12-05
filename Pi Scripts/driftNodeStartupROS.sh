#!/bin/bash
#Starts driftNode ROS Stuff

echo "********driftNodeStartup running********"

#wait to make sure gpsd and other stuff is up and running
   sleep 5

echo "********hopefully everything else is running, so we're going for it now...********"

#making sure ros commands are added to bash
   source /opt/ros/indigo/setup.bash
   source /home/pi/ros_catkin_ws/devel/setup.bash

#setting up ROS master address
   export ROS_MASTER_URI=http://localhost:11311	#localhost until networking is done
   unset ROS_IP

#run the ROS master locally (until networking is done)
   echo "********starting ROS master********"
   roscore&	#will run continuously so we don't want to wait for a return
   sleep 45 	#leave time for the ROS master to start up
   echo "********started ROS master********"

#run imu node
   echo "********starting imu node********"
   rosrun imu_node imu_talker.py&
   sleep 10	#giving the pi some time to start node so it doesn't have a conniption
   echo "********started imu node********"

#starting gps node
   echo "********starting gps node********"
   rosrun gps_node_py gps_talker.py&
   sleep 10	#giving the pi some time to start node so it doesn't have a conniption
   echo "********started gps node********"

#starting camera node
   echo "********starting camera node********"
   rosrun raspicam raspicam_node _width:=1200 _height:=900 _framerate:=1 _quality:=75 _tf_prefix:=pi- &
   sleep 20	#giving the pi some time to start node so it doesn't have a conniption
   echo "********started camera node********"
   echo "********calling service to start camera capture********"
   rosservice call /camera/start_capture	#telling the camera to start

#starting the rosbag
   sleep 20	#giving the pi some time to start everything so the bag will catch it
   
   echo " "
   echo "@{Bow}========  The ROSBAG is about to start recording to /home/pi/bagfiles  ========"
   echo "@{Bow}========  After it starts recording CTRL+C Can be pressed to stop the driftNode  ========"
   echo "@{Bow}========  To Restart the driftNode Run : /etc/init.d/driftNodeStartupROS.sh  ========"
   echo " "


   cd /home/pi/bagfiles	#go to bagfiles directory
   rosbag record -a -q --split --duration=2m 	#record all topics (including images for now) in 2m bags
   sleep 15	#waiting enough time for rosbag's output to finish writing to terminal
   echo "********rosbag is recording to /home/pi/bagfiles********"

#echo "********Everything should be running now.  Let's hit it...********"


#exit 0
