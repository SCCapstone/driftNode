#!/bin/bash

echo "driftNode starting up"

#making sure GPS fix led is off before starting
sudo python /home/pi/deps/GPIOLEDoff.py

#setting node name
export ROS_NAMESPACE=dn2	#this is to prefix topics with the node id
echo "Node: $ROS_NAMESPACE"
#####BE SURE TO CHANGE NODE ID IN CAMERA CAPTURE SERVICE IN THIS FILE#####

#setting up ad-hoc network
sudo ifconfig wlan0 down
sudo ifconfig wlan0 up
sudo iwconfig wlan0 mode ad-hoc
sudo iwconfig wlan0 essid "$ROS_NAMESPACE"
sudo ifconfig wlan0 192.168.1.1 netmask 255.255.255.0
echo "ad-hoc network set up"
sleep 5

#starting the GPS reader that will wait until the GPS gets a fix before starting everything else
echo "starting the GPS reader. Once it gets a fix green led will illuminate and ROS will start"
#the program that will wait until the GPS has a fix before exiting
python /home/pi/deps/GPSreader.py > GPSstartLog.txt

#waits until the GPS reader exits so we know the GPS has a fix
wait $!

sudo python /home/pi/deps/GPIOLEDon.py

#Starts driftNode ROS Stuff

echo "********driftNodeStartup running********"
echo "********Please Allow 10 min for Startup********"

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
   sleep 120 	#leave time for the ROS master to start up
   echo "********started ROS master********"

#starting gps node
   echo "********starting gps node********"
   rosrun gps_node_py gps_talker.py&
   sleep 60	#giving the pi some time to start node so it doesn't have a conniption
   echo "********started gps node********"

#run imu node
   echo "********starting imu node********"
   rosrun imu_node imu_talker.py&
   sleep 60	#giving the pi some time to start node so it doesn't have a conniption
   echo "********started imu node********"

#starting wifi_node
   echo "********starting wifi node********"
   rosrun wifi_node wifi_talker.py&
   sleep 60	#giving the pi some time to start node so it doesn't have a conniption
   echo "********started wifi node********"

#starting camera node
   echo "********starting camera node********"
   rosrun raspicam raspicam_node _width:=1200 _height:=900 _framerate:=2 _quality:=75 _tf_prefix:=pi- &
   sleep 120	#giving the pi some time to start node so it doesn't have a conniption
   echo "********started camera node********"
   echo "********calling service to start camera capture********"
   rosservice call /$ROS_NAMESPACE/camera/start_capture	#telling the camera to start

#starting the rosbag
   sleep 120	#giving the pi some time to start everything so the bag will catch it
   
   echo " "
   echo "========  The ROSBAG is about to start recording to /home/pi/bagfiles  ========"
   echo "========  After it starts recording CTRL+C Can be pressed to stop the driftNode  ========"
   echo "========  To Restart the driftNode Run : /etc/init.d/driftNodeStartupROS.sh  ========"
   echo " "


   cd /home/pi/bagfiles	#go to bagfiles directory
   rosbag record -a -q --split --size=500000 -o $ROS_NAMESPACE 	#record all topics (including images for now) in 2m bags
   sleep 30	#waiting enough time for rosbag's output to finish writing to terminal
   echo "********rosbag is recording to /home/pi/bagfiles********"

#echo "********Everything should be running now.  Let's hit it...********"


#exit 0
