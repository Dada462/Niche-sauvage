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

def callback(data):
    global id
    global x,y,z,roll_x, pitch_y, yaw_z
    global accel_x, accel_y, accel_z
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    roll_x, pitch_y, yaw_z = euler_from_quaternion(data.orientation.x,data.orientation.x,data.orientation.x,data.orientation.x)
        
    if id>=0 and id<=5 :
        if z>0.7:
            accel_z =1 ##avance
            print("on avance")
        if z<0.7:
            accel_z = -1##recule 
            print("on recule")
        if x > 0 :
            accel_x =1 ##droite
        if x < 0 :
            accel_x = -1##gauche
        if y > 0 :
            accel_y = 1##bas
        if y < 0 :
            accel_y = -1##haut
    else :
        accel_x = 0
        accel_y = 0
        accel_z = 0
        print("on stoppe")
 

def callback2(data):
    global id
    id = data.data

def listener_and_talker():
    
    rospy.init_node('aruco_commande', anonymous=True)


    pub = rospy.Publisher('commande_qr_aruco',PoseStamped, queue_size=10)

    rospy.Subscriber('bluerov_pose_aruco', PoseStamped, callback)
    rospy.Subscriber('id_qr_code_aruco', Float64, callback2)
    
    

    #spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    rate = rospy.Rate(20) 
    
    
    
    while not rospy.is_shutdown():    
        global msg
        msg.pose.position.x = accel_x
        msg.pose.position.y = accel_y
        msg.pose.position.z = accel_z
  

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
    msg = PoseStamped
   
    listener_and_talker()

  