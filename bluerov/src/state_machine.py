#!/usr/bin/env python3

import os
import rospy
import numpy
import time
from std_msgs.msg import Bool
from bluerov_msgs.msg import CommandBluerov


class ROS1Subscriber:
    def __init__(self):
        self.subscriber = rospy.Subscriber("is_qrcode", Bool, self.callback_bool_qrcode)
        self.subscriber = rospy.Subscriber("is_lights", Bool, self.callback_bool_lights)
        self.subscriber = rospy.Subscriber("command_usbl", CommandBluerov, self.callback_command_usbl)
        self.subscriber = rospy.Subscriber("command_lights", CommandBluerov, self.callback_command_lights)
        self.subscriber = rospy.Subscriber("command_qrcode", CommandBluerov, self.callback_command_qrcode)
        self.pub = rospy.Publisher('commande', CommandBluerov, queue_size=10)   
        self.detect_qrcode = False
        self.detect_lights = False
        self.usbl_command = CommandBluerov()
        self.lights_command = CommandBluerov()
        self.qrcode_command = CommandBluerov()

    def callback_bool_qrcode(self, msg):
        self.detect_qrcode = msg.data

    def callback_bool_lights(self, msg):
        self.detect_lights = msg.data

    def callback_command_usbl(self, msg):
        self.usbl_command = msg

    def callback_command_lights(self, msg):
        self.lights_command = msg

    def callback_command_qrcode(self, msg):
        self.qrcode_command = msg

    def start_subscriber(self):
        rospy.init_node('state_machine', anonymous=True)
        # rospy.spin()

        while not rospy.is_shutdown():
            if self.detect_qrcode:
                self.pub.publish(self.qrcode_command)

            elif self.detect_lights:
                self.pub.publish(self.lights_command)

            else:
                self.pub.publish(self.usbl_command)





if __name__ == '__main__':
    subscriber = ROS1Subscriber()
    subscriber.start_subscriber()