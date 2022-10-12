# #!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import rospy
import numpy as np
import sensor_msgs.msg
import std_msgs.msg
import os
import math
from mavros_msgs.msg import State

"""
Simple Python subscriber to a ROS topic (/front_angle_low_res) which in my case is outputing an integer result 
and displays the result in a tkinter window
credit: https://www.pythontutorial.net/tkinter/tkinter-after/
credit: https://www.youtube.com/watch?v=EAAd5vXA8lE&t=260s
before running make sure to $ rostopic echo /front_angle_low_res to make sure data is available
This script can be started from the folder it resides in with $ python3 sensor_display.py
"""

class Display_Sensor_1(tk.Tk):

    def __init__(self):
        super().__init__()
        self.sub = rospy.Subscriber("/mavros/global_position/compass_hdg", std_msgs.msg.Float64, self.callback_compass)
        self.compass_data = tk.IntVar()
        self.sub = rospy.Subscriber("/mavros/global_position/rel_alt", std_msgs.msg.Float64, self.callback_alt)
        self.alt_data = tk.IntVar()
        self.sub = rospy.Subscriber("/enregistrement_camera", std_msgs.msg.Float32MultiArray, self.callback_light)
        self.light_data = tk.IntVar()
        self.sub = rospy.Subscriber("/mavros/battery", sensor_msgs.msg.BatteryState, self.callback_batt)
        self.batt_data = tk.IntVar()
        self.sub = rospy.Subscriber("/mavros/state", State, self.callback_arm)
        self.arm_data = tk.IntVar()

        # configure the root window
        self.title('[arm; cap; alt; light; batt]')
        self.resizable(0, 0)
        self.geometry('250x240')
        self['bg'] = 'black'
        
        # change the background color to black
        self.style = ttk.Style(self)
        self.style.configure('TLabel', background='black', foreground='red')

        self.label_arm = ttk.Label(self, text=self.get_arm_data(), font=('Digital-7', 20))
        self.label_arm.pack(expand=True)
        self.label_arm.after(20, self.update)

        self.label_compass = ttk.Label(self, text=self.get_compass_data(), font=('Digital-7', 20))
        self.label_compass.pack(expand=True)

        self.label_alt = ttk.Label(self, text=self.get_alt_data(), font=('Digital-7', 20))
        self.label_alt.pack(expand=True)
        self.label_alt.after(20, self.update)

        self.label_light = ttk.Label(self, text=self.get_light_data(), font=('Digital-7', 20))
        self.label_light.pack(expand=True)
        self.label_light.after(20, self.update)

        self.label_batt = ttk.Label(self, text=self.get_batt_data(), font=('Digital-7', 20))
        self.label_batt.pack(expand=True)
        self.label_batt.after(20, self.update)

    def callback_compass(self, data):   
        self.compass_data = data.data
        #print(self.compass_data)
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def callback_alt(self, data):   
        self.alt_data = data.data

    def callback_light(self, data):   
        self.light_data = data.data[1]

    def callback_batt(self, data):   
        self.batt_data = math.trunc(data.voltage*100)/100

    def callback_arm(self, data):   
        self.arm_data = data.armed

    def get_compass_data(self):
        return self.compass_data

    def get_alt_data(self):
        return self.alt_data

    def get_light_data(self):
        return self.light_data

    def get_batt_data(self):
        return self.batt_data

    def get_arm_data(self):
        return self.arm_data

    def update(self):
        """ update the label every 1 second """
        self.label_compass.configure(text=self.get_compass_data())
        self.label_compass.after(1000, self.update)     # schedule another timer
        self.label_alt.configure(text=self.get_alt_data())
        self.label_alt.after(1000, self.update)
        self.label_light.configure(text=self.get_light_data())
        self.label_light.after(1000, self.update)
        self.label_batt.configure(text=self.get_batt_data())
        self.label_batt.after(1000, self.update)
        self.label_arm.configure(text=self.get_arm_data())
        self.label_arm.after(1000, self.update)

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    sensor = Display_Sensor_1()
    sensor.mainloop()