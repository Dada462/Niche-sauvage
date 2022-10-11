#!/usr/bin/env python
import roslib
import rospy
import os

rospy.init_node("play_node", anonymous=True)


path = rospy.get_param("~path")
repeat = rospy.get_param("~repeat")
delay = str(rospy.get_param("~delay"))
clock = str(rospy.get_param("~clock"))

if repeat:
    rep = "-l"
else :
    rep = ""

if clock:
    time = "--clock"
else :
    time = ""

os.system("cd "+path)
os.system("rosbag play "+rep+" -s "+delay+" "+time+" "+path)