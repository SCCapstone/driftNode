import rosbag
import sys
from std_msgs.msg import Int32, String 



nodeName = str(sys.argv[1])
otherNodeName = str(sys.argv[2])
in_bag = str(sys.argv[3])
out_bag = rosbag.Bag(sys.argv[4],'w')
parsedDataPath = str(sys.argv[5])

try:
	openfile = open(parsedDataPath,'r')

	for topic, msg, t in rosbag.Bag(in_bag).read_messages():
		if topic == "/"+nodeName+"/wifi_chatter":
			i = Int32()
			currentLine = openfile.readline();
			dataLines = currentLine.split(",")

			i.data = int(dataLines[2])
			out_bag.write("/"+nodeName+"/"+otherNodeName+"SignalStrength", i, t)
			out_bag.write(topic, msg, t)
		else:
			out_bag.write(topic, msg, t)
finally:
	out_bag.close()
	openfile.close()
