#!/usr/bin/env python3

import os
from PIL import Image, ImageFont, ImageDraw


import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64

from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32MultiArray

from mavros_msgs.msg import State

import datetime

import cv2
import gi
import numpy as np
import math

gi.require_version('Gst', '1.0')
from gi.repository import Gst

import time

import sensor_msgs.msg

ROV_name = ""

class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.resolution = ( 640, 480) # ajout de ma part

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')



    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        

        
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
            
        resolution = (
                caps.get_structure(0).get_value('width'),  
                caps.get_structure(0).get_value('height')
                
                )        
        
        return array, resolution

    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf,   

            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame, new_resolution = self.gst_to_opencv(sample)
        self._frame = new_frame
        
        self.resolution = new_resolution

        return Gst.FlowReturn.OK


# class Node:
#     def __init__(self,port_udp):
#         self.pub = rospy.Publisher('/bluerov_video', sensor_msgs.msg.Image, queue_size=1)
#         self.msg = sensor_msgs.msg.Image
#         self.video = Video(port_udp) # connection établie
#         frame_width = int(self.video.resolution[0])
#         frame_height = int(self.video.resolution[1])
#         self.msg.width = frame_width
#         self.msg.height = frame_height

#     def callback(self, msg):
#         self.msg = msg
#         self.publish()

#     def publish(self):
#         frame = self.video.frame()
#         self.msg.encoding = frame
#         self.pub.publish(self.msg)
        
#     def run(self):
#         rate = rospy.Rate(50)
#         while not rospy.is_shutdown():
#             rate.sleep()
        
    
# if __name__ == '__main__':
#     print("Start publish bluerov video")
#     np.set_printoptions(precision=6)
#     port_udp = rospy.get_param('~camera_port_udp',5600)
#     ROV_name = rospy.get_param('~ROV_name',"")
#     ID = rospy.get_param('~ID',"1")
#     ID0 = int(ID)
#     rospy.init_node('video_publisher')
#     Node(port_udp).run()

def talker():
    pub = rospy.Publisher('/bluerov_video', sensor_msgs.msg.Image, queue_size=1)
    rospy.init_node('video_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = sensor_msgs.msg.Image()

    port_udp = rospy.get_param('~camera_port_udp',5600)
    video = Video(port_udp)
    while not rospy.is_shutdown():
        frame = video.frame()
        if np.any(frame != None) :
            # ['header', 'height', 'width', 'encoding', 'is_bigendian', 'step', 'data']
            frame_width = int(video.resolution[0])
            frame_height = int(video.resolution[1])
            # print(type(frame))
            msg.width = frame_width
            msg.height = frame_height
            # now = rospy.Time.now()
            # msg.header.stamp = now
            # print("Image = ",frame)
            # print(type(frame))
            # print(tuple(frame))
            msg.data = tuple(map(tuple, frame))
            pub.publish(msg)
            rate.sleep()
        else :
            print("no camera")
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass