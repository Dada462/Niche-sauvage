#!/usr/bin/env python3

import rospy
import numpy as np
import sensor_msgs.msg
import std_msgs.msg
import os
import math

msg_compass = 0
msg_alt = 0
msg_joy_switch = 0
lvl_lumiere = 0

def listen():
    rospy.Subscriber("/mavros/global_position/compass_hdg", std_msgs.msg.Float64, callback_compass)  
    rospy.Subscriber("/mavros/global_position/rel_alt", std_msgs.msg.Float64, callback_alt)
    rospy.Subscriber("/num_joy_control", std_msgs.msg.Float64, callback_joy_switch)
    rospy.Subscriber("/enregistrement_camera", std_msgs.msg.Float32MultiArray, callback_enregistrement) 
    # /mavros/imu/data
    # /joy sensor_msg/Joy
    
def callback_compass(msg):
    global msg_compass
    msg_compass = msg.data

def callback_alt(msg):
    global msg_alt
    msg_alt = msg.data

def callback_joy_switch(msg):
    global msg_joy_switch
    msg_joy_switch = msg.data

def callback_enregistrement(msg):
    global lvl_lumiere 
    lvl_lumiere = msg.data[1]

def save_log_msg(log):
    log.write(
        "compass: "+str(msg_compass)+" \n"+
        "altitude: "+str(msg_alt)+" \n"+
        "joy_control: "+str(msg_joy_switch)+" \n"+
        "lumiere: "+str(lvl_lumiere)+" \n"+
        " \n"
    )

if __name__ == '__main__':
    np.set_printoptions(precision=6)
    rospy.init_node('recup_log')

    log_name = os.path.expanduser("~")+"/catkin_ws/src/Niche-sauvage/log_generator/logs/logs_1.txt"
    log = open(log_name,"w")

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        listen()
        save_log_msg(log)
        rate.sleep()