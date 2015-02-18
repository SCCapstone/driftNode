#!/bin/bash

source /opt/ros/indigo/setup.bash

echo "Extracting Data from the bag."
for topic in `rostopic list -b $1 | grep wifi_chatter` ; 
do rostopic echo -p -b $1 $topic > data.txt
done

echo "Done extracting data."

echo "Parsing data."
perl ./ParseRobotPoop.pl "data.txt" "parsed_data.txt"

echo "Done parsing data."

echo "Adding parsed data to bag."
python ./addDataToBag.py $2 $3 $1 "outputBag.bag" "parsed_data.txt"

echo "Done.
