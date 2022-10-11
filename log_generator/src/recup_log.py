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
lvl_light = 0
accel = [0,0,0]
gyro = [0,0,0]
time = 0
buttons = np.zeros((11,1))
axes = np.zeros((8,1))

def listen():
    rospy.Subscriber("/mavros/global_position/compass_hdg", std_msgs.msg.Float64, callback_compass)
    rospy.Subscriber("/mavros/global_position/rel_alt", std_msgs.msg.Float64, callback_alt)
    rospy.Subscriber("/num_joy_control", std_msgs.msg.Float64, callback_joy_switch)
    rospy.Subscriber("/enregistrement_camera", std_msgs.msg.Float32MultiArray, callback_enregistrement)
    rospy.Subscriber("/mavros/imu/data", sensor_msgs.msg.Imu, callback_imu)
    rospy.Subscriber("/joy", sensor_msgs.msg.Joy, callback_joy)
    
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
    global lvl_light 
    lvl_light = msg.data[1]

def callback_imu(msg):
    global gyro
    global accel
    global time
    gyro[0] = msg.angular_velocity.x
    gyro[1] = msg.angular_velocity.y
    gyro[2] = msg.angular_velocity.z
    accel[0] = msg.linear_acceleration.x
    accel[1] = msg.linear_acceleration.y
    accel[2] = msg.linear_acceleration.z
    time = msg.header.stamp.secs

def callback_joy(msg):
    global buttons
    global axes
    for i in range(buttons):
        buttons[i] = msg.buttons[i]
    for i in range(axes):
        axes[i] = msg.axes[i]

def save_log_msg(log):
    log.write(
        "time: "+str(time)+" \n"+
        "compass: "+str(msg_compass)+" \n"+
        "altitude: "+str(msg_alt)+" \n"+
        "joy_control: "+str(msg_joy_switch)+" \n"+
        "light: "+str(lvl_light)+" \n"+
        "accel: "+str(accel[0])+" "+str(accel[1])+" "+str(accel[2])+" \n"+
        "gyro: "+str(gyro[0])+" "+str(gyro[1])+" "+str(gyro[2])+" \n"+
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