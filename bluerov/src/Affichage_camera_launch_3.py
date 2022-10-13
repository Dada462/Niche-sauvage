#!/usr/bin/env python3
"""
BlueRov video capture class
"""

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

import sensor_msgs.msg

gi.require_version('Gst', '1.0')
from gi.repository import Gst

import time

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

        self.resolution = ( 1280, 720) # ajout de ma part

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


######################
arm = False
def callback_ROV_state(data):
    global arm 
    arm = data.armed
    
joy_active = 1
def callback_joy_switch(data):
    global joy_active 
    joy_active = data.data
    
cap = 0
def callback_compass(data):
    global cap 
    cap = data.data


altitude = 0
def callback_alt(data):
    global altitude
    altitude = -data.data
    
enregistrement = 0
lvl_lumiere = 0
def callback_enregistrement(data):
    global enregistrement
    global lvl_lumiere 
    enregistrement = data.data[0]
    lvl_lumiere = data.data[1]
    
voltage = 0

def callback_battery(data):
    global voltage 
    voltage  = math.trunc(data.voltage*100)/100
    
               	
def listener():

    ns = ""
    # ns = "/"+ ROV_name
    rospy.Subscriber(ns+"/enregistrement_camera", Float32MultiArray, callback_enregistrement) 
    rospy.Subscriber(ns+"/mavros/global_position/compass_hdg", Float64, callback_compass)  
    rospy.Subscriber(ns+"/mavros/global_position/rel_alt", Float64, callback_alt)

    rospy.Subscriber(ns+"/mavros/battery", BatteryState, callback_battery)
    rospy.Subscriber("/num_joy_control", Float64, callback_joy_switch)
    rospy.Subscriber(ns+"/mavros/state", State, callback_ROV_state)


######################



if __name__ == '__main__':


    rospy.init_node('MyProgram_camera', anonymous=True, disable_signals=True)
    
    Affichage = 1
    enregistrement = 0
    
    port_udp = rospy.get_param('~camera_port_udp',5600)
    ROV_name = rospy.get_param('~ROV_name',"")
    ID = rospy.get_param('~ID',"1")
    ID0 = int(ID)

### Initialisation
    # Create the video object
    # Add port= if is necessary to use a different one
    print('connexion camera...')
    video = Video(port_udp)
    test = 0
    
    test_enregistrement = 0
    
    while test ==0:
        # Wait for the next frame
        if not video.frame_available():
            continue

        frame = video.frame() #pour mettre à jour les parametres comme la résolution
        test = 1
        print('connexion etablie')
    # print(video.resolution)
        
    
       
#### Afficher la camera
    if Affichage == 1:
        
        while True:
        
            # on écoute les consignes
            listener()
            # print(enregistrement)
            
#            print(ROV_name,' profondeur = ', altitude)
#            print(ROV_name,' cap = ', cap)               
                
            
            # Wait for the next frame
            if not video.frame_available():
                continue

            frame = video.frame()
            
            # ajout des informations
            font = cv2.FONT_HERSHEY_SIMPLEX
            text1 = "cap = " + str(cap) + " deg"
            text2 = "depth = " + str(altitude) + " m"
            text3 = "battery = " + str(voltage) + " V"
            text4 = "lumiere = " + str(lvl_lumiere)
            taille_font = 0.5
            current_date_and_time = datetime.datetime.now()
            text_date = str(current_date_and_time)[0:19]         
            cv2.putText(frame, text1, (10,20), font, taille_font, (0, 255, 0), 1, cv2.LINE_AA)  
            cv2.putText(frame, text2, (10,40), font, taille_font, (0, 255, 0), 1, cv2.LINE_AA) 
            cv2.putText(frame, text3, (10,60), font, taille_font, (0, 255, 0), 1, cv2.LINE_AA)  
            cv2.putText(frame, text4, (10,80), font, taille_font, (0, 255, 0), 1, cv2.LINE_AA)  
            cv2.putText(frame, text_date, (400,20), font, taille_font, (0, 255, 0), 1, cv2.LINE_AA)
            
            # eclairage
            if (lvl_lumiere == 9):
                text4 = "MAX LUMINOSITY"
                cv2.putText(frame, text4, (610,20), font, taille_font, (0, 0, 255), 1, cv2.LINE_AA)
                
            # commande joystick
            if (joy_active == ID0):
                text5 = "ROV piloted"
                cv2.putText(frame, text5, (200,20), font, taille_font, (255, 255, 0), 1, cv2.LINE_AA)
            else:
                text5 = "ROV not piloted"
                cv2.putText(frame, text5, (200,20), font, taille_font, (0, 0, 255), 1, cv2.LINE_AA)
                
            # armement du ROV
            if arm :
                text5 = "armed: TRUE"
                cv2.putText(frame, text5, (200,40), font, taille_font, (0, 0, 255), 1, cv2.LINE_AA)
            else:
                text5 = "armed: FALSE"
                cv2.putText(frame, text5, (200,40), font, taille_font, (0, 255, 0), 1, cv2.LINE_AA)               
                
                
            frame2 = frame
            if (enregistrement == 1): 
                cv2.putText(frame, 'ENREGISTREMENT', (400,40), font, taille_font, (0, 0, 255), 1, cv2.LINE_AA)
                
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
            
                
### Enregistrer une video

    

            if (enregistrement == 1): # &&(test_enregistrement = =0):

                if (test_enregistrement == 0):
                    fps = 10  # NE PAS METTRE < 10fps !!
                    frame_width = int(video.resolution[0])
                    frame_height = int(video.resolution[1])
                    size = (frame_width, frame_height)
                    
                    # titre de la video
                    current_date_and_time = datetime.datetime.now()
                    current_date_and_time_string = str(current_date_and_time)[0:19]  #[0:19] : on ne garde que jusqu'aux seconds
                    
                    # creation d un dossier ou tout stocker
                    path = "~/catkin_ws/src/Niche-sauvage/videos/" + ROV_name + '_'+ current_date_and_time_string
                    os.mkdir(path)
                    
                    extension = ".avi"
                    file_name =  path + "/Video_camera_"+ ROV_name + '_' + current_date_and_time_string + extension
                   
                    result = cv2.VideoWriter(file_name, 
                                         cv2.VideoWriter_fourcc(*'XVID'),
                                         fps, size)
                                         
                                         
                    # creation d'autre fichier
                    name_file_cap = path + "/cap_" + ROV_name + '_'+ current_date_and_time_string + ".txt" 
                    name_file_alt = path + "/altitude_" + ROV_name + '_'+ current_date_and_time_string + ".txt"                
                    name_file_time = path + "/time_" + ROV_name + '_'+ current_date_and_time_string + ".txt" 
                    
                    file_cap = open(name_file_cap,'w')
                    file_alt = open(name_file_alt,'w')
                    file_time = open(name_file_time,'w')
                    
                    
                    print('resolution video : ', size)
                    print('début de l enregistrement')
                    test_enregistrement = 1
                    
                    


                # Write the frame into the file 'filename.avi'
                file_cap.write(str(cap)+' \n')
                file_alt.write(str(altitude)+' \n' )
                file_time.write(str(current_date_and_time)+' \n' )
                
                result.write(frame2)

                time.sleep(1/fps)
                
            elif test_enregistrement == 1:
                test_enregistrement = 0
                result.release()
                file_cap.close()
                file_alt.close()
                file_time.close()
                
                print('fin enregistrement')
                print(' ')
            #    # Closes all the frames
            #    cv2.destroyAllWindows()

                     

         
            
