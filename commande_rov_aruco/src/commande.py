#!/usr/bin/env python3

import queue
import sys
import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv2.aruco as aruco
from bluerov_msgs.msg import CommandBluerov
import time

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def callback(data): ## callback qui recupere la pose du robot dans le repere du marqueur et renvoie la commande associée
    global id
    global x,y,z,roll_x, pitch_y, yaw_z
    global accel_x, accel_y, accel_z



    desire_pos = 0.5

    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z

    roll_x, pitch_y, yaw_z = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
    
    """
       disposition : 0 : interieur fond de la cage
                     1 : interieur gauche
                     2 : interieur droit
                     3 : entrée gauche
                     4 : entrée droite
                     5 : entrée derriere gauche
                     6 : entrée derriere droit
                     7 : exterieur gauche
                     8 : exterieur droit
                     9 : non utilisé
                    
    """
    
    if id != -1 :   ## se placer devant
        kx=1
        lx=-np.tanh(kx*((desire_pos-z)))
        accel_z = lx

    else :
        accel_x = 0
        accel_y = 0
        accel_z = 0
        #print("on stoppe")

 

def callback2(data): ## callback qui récupère l'id du marqueur
    global id
    id = data.data



def callback3(data):                 ##callback pour ajuster le rov dans l'espace plan aruco
    global depths
    centrex = data.pose.position.x
    centrey = data.pose.position.y
    pos=np.array([centrex,centrey])
    desire_pos=np.array([160,400])
    
    #################### Robot frame #################### 
    try:
        Vz=(depths[-1][0]-depths[-2][0])/(depths[-1][1]-depths[-2][1])
    except:
        Vz=0
    ky=1
    ly=-np.tanh(ky*((desire_pos[1]-centrey)/100))
    kz=0.5
    dkz=.2
    lz=np.tanh(kz*((desire_pos[0]-centrex)/100) -dkz*Vz)

    #################### Robot frame #################### 


    global accel_x, accel_y, accel_z,rot_z
    accel_y=lz
    accel_x=ly
    #rot_z = - ly
    # if centrex == 0.0 :
    #     accel_y = 0.0
    #     rot_z = 0.0
    #     return 0

    # if centrex>140:
    #     accel_y = -0.02

    # if centrex<120:
    #     accel_y = 0.02

    # if centrey >420:
    #     rot_z = -0.3

    # if centrey <380:
    #     rot_z = 0.3

def callback_depth(msg):
    global depths
    depths.append([msg.data,time.time()])

def listener_and_talker():
    global depths
    rospy.init_node('aruco_commande', anonymous=True)


    pub = rospy.Publisher('commande',CommandBluerov, queue_size=10)
    depths=[]
    rospy.Subscriber('bluerov_pose_aruco', PoseStamped, callback)
    rospy.Subscriber('id_qr_code_aruco', Float64, callback2)
    rospy.Subscriber('centre_aruco_img', PoseStamped, callback3)
    rospy.Subscriber('/mavros/global_position/rel_alt', Float64, callback_depth)
    

    #spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    rate = rospy.Rate(20) 
    
    
    
    while not rospy.is_shutdown():    
        global id
        global x,y,z,roll_x, pitch_y, yaw_z
        global accel_x, accel_y, accel_z, rot_z
        msg = CommandBluerov()
        ##deplacement en cap msg.pose.orientation.z
        msg.arming = 1
        msg.power = 100
        msg.light = 0
        msg.pose.position.x = accel_z
        msg.pose.position.y = accel_x
        msg.pose.position.z = accel_y
        msg.pose.orientation.z = rot_z

        print("avance =",msg.pose.position.x)
        print("haut/bas =",msg.pose.position.z)
        print("gauche/droite =",msg.pose.position.y)
        print("rot =",msg.pose.orientation.z)

        

        pub.publish(msg)


        
        rate.sleep()

if __name__ == '__main__':
    x = 0
    y = 0
    z = 0
    roll_x = 0
    pitch_y = 0
    yaw_z = 0
    id = -1
    accel_x = 0
    accel_y = 0
    accel_z = 0
    rot_z = 0
  

   
   
    listener_and_talker()

  