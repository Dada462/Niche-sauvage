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
    ## Méthode bang bang
    if z == 0.0:
        return 0 
    if id == 0 :   ## se placer devant
        if z>0.5:
            accel_z = 0.2 ##avance
            #print("on avance")
        if z<0.1:
            accel_z = -0.2 ##recule 
            #print("on recule")
        #if x > 0.3 :
        #    accel_x = -0.2 ##droite
        #    #print("va a droite")
        #if x < -0.3 :
        #    accel_x = 0.2 ##gauche
        #    #print("va a gauche")
        #if y > 0.3 :
        #    accel_y = -0.2 ##bas
        #    #print("en bas")
        #if y < -0.3 :
        #    accel_y = 0.2 ##haut
        #    #print("en haut")



    elif id == 1 : ## se placer à droite
            if z>0.5:
                accel_z = 0.2 ##avance
                #print("on avance")
            if z<0.1:
                accel_z = -0.2 ##recule 
                #print("on recule")

    elif id == 2 : ## se placer à droite
            if z>0.5:
                accel_z = 0.2 ##avance
                #print("on avance")
            if z<0.1:
                accel_z = -0.2 ##recule 
                #print("on recule")

    elif id == 3 : ## se placer à droite
        if z>0.5:
            accel_z = 0.2 ##avance
            #print("on avance")
        if z<0.1:
            accel_z = -0.2 ##recule 
            #print("on recule")
        #if x > -0.5 :
        #    accel_x =-0.2 ##droite
        #    #print("va a droite")
        #if x < -0.3 :
        #    accel_x = 0.2 ##gauche
        #    print("a gauche")
        #if y > 0.3 :
        #    accel_y = -0.2 ##bas
        #    #print("en bas")
        #if y < -0.3 :
        #    accel_y = 0.2 ##haut
        #    #print("en haut")

    elif id == 4 : ## se placer à gauche
        if z>0.5:
            accel_z = 0.2 ##avance
            #print("on avance")
        if z<0.1:
            accel_z = -0.2 ##recule 
            #print("on recule")
        #if x > -0.8 :
        #    accel_x =-0.2 ##droite
        #    print("a droite")
        #if x < 0.5 :
        #    accel_x = 0.2 ##gauche
        #    #print("va a gauche")
        #if y > 0.3 :
        #    accel_y = -0.2 ##bas
        #    #print("en bas")
        #if y < -0.3 :
        #    accel_y = 0.2 ##haut
        #    #print("en haut")


    else :
        accel_x = 0
        accel_y = 0
        accel_z = 0
        #print("on stoppe")

 

def callback2(data): ## callback qui récupère l'id du marqueur
    global id
    id = data.data



def callback3(data):                 ##callback pour ajuster le rov dans l'espace plan aruco
    centrex = data.pose.position.x
    centrey = data.pose.position.y
    global accel_x, accel_y, accel_z,rot_z
    if centrex == 0.0 :
        return 0
    if centrex>140:
        accel_y = -0.2

    if centrex<120:
        accel_y = 0.2

    if centrey >420:
        rot_z = -0.2

    if centrey <380:
        rot_z = 0.2


def listener_and_talker():
    
    rospy.init_node('aruco_commande', anonymous=True)


    pub = rospy.Publisher('commande',CommandBluerov, queue_size=10)

    rospy.Subscriber('bluerov_pose_aruco', PoseStamped, callback)
    rospy.Subscriber('id_qr_code_aruco', Float64, callback2)
    rospy.Subscriber('centre_aruco_img', PoseStamped, callback3)
    

    #spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    rate = rospy.Rate(20) 
    
    
    
    while not rospy.is_shutdown():    
        global msg
        global id
        global x,y,z,roll_x, pitch_y, yaw_z
        global accel_x, accel_y, accel_z, rot_z

        ##deplacement en cap msg.pose.orientation.z
        msg.arming = 1
        msg.power = 100
        msg.pose.position.x = accel_z
        msg.pose.position.y = accel_x
        msg.pose.position.z = accel_y
        msg.pose.orientation.z = rot_z

        print("avance =",accel_z)
        print("haut/bas =",accel_y)
        print("rot =",rot_z)

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

    msg = CommandBluerov()
   
    listener_and_talker()

  