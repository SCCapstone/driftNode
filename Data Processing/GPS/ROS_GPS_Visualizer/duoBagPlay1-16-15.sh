#!/bin/bash
 
# Automates playing the bags from both nodes. This script will launch rosbridge and then ask if it is finsihed launching as well as if the user has the website ready. It then asks for the speed to play the bags at. Using this speed value it calculates how long till dn1 should wait before starting its bags so they are relatively close to each other. After dn3 is done, it waits on dn1 to finish and the closes rosbridge.

# This script assumes that this script as well as the dn1/3BagPlayer scripts are in the same directory and that the bags for each node are in a directory below named after its node "dn1" or "dn3".

answer=""
speed=1

xterm -e roslaunch rosbridge_server rosbridge_websocket.launch &
pid=$!

echo -n "Once rosbridge is running, open the website and toggle the connection. Are you ready to continue? [y/n] "

while [ "$answer" != "y"  -a "$answer" != "Y" ]
do
	read answer
done

echo -n "Enter the speed at which you would like to play the bags: "
read speed

(sleep $((480/$speed)); xterm -e ./dn1BagPlayer1-16-15.sh $speed; exit) &
pid2=$!
source dn3BagPlayer1-16-15.sh $speed

printf '\n%s\n\n' "Waiting for the other node to finish playing its bag."
wait $pid2

kill $pid
clear; clear
