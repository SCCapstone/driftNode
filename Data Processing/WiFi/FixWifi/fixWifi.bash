#!/bin/bash

source /opt/ros/indigo/setup.bash

echo "Extracting Data from the bag: $1"
for topic in `rostopic list -b $1 | grep wifi_chatter` ; 
do rostopic echo -p -b $1 $topic > data.txt
done

echo "Done extracting data, saved to file: data.txt"

echo "Parsing data from file: data.txt"
perl ./fixWifi_parse.pl "data.txt" "parsed_data.txt" $3

echo "Done parsing data, saved to file: parsed_data.txt"

echo "Adding parsed data to bag: $1"
python ./fixWifi_addDataToBag.py $2 $3 $1 "outputBag.bag" "parsed_data.txt"

echo "Done adding parsed data to bag, saved to file: outputBag.bag"
echo "Finished fixing bag."
