#!/usr/bin/env python
import roslib
import rospy
import os

rospy.init_node("save_node", anonymous=True)

compress_mode = rospy.get_param("~compress_mode") # " ", "lz4" or "bz2"
path = rospy.get_param("~path")

if compress_mode == "lz4":
    mode = "--lz4"
elif compress_mode == "bz4":
    mode = "--bz4"
else :
    mode = ""

os.system("rosbag record --split --duration 5m "+mode+" --chunksize=1024 --output-prefix="+path+" --all")