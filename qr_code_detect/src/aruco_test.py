#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import numpy as np
import cv2
import cv2.aruco as aruco


bridge = CvBridge()


def get_quaternion_from_euler(roll, pitch, yaw):

    """
    Convert an Euler angle to a quaternion.
    
    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
    
    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qx, qy, qz, qw]


def callback(data):
    
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8") #conversion du message ROS en cv image
    except CvBridgeError as e:
      print(e)

    # CLAHE (Contrast Limited Adaptive Histogram Equalization)
    clahe = cv2.createCLAHE(clipLimit=3., tileGridSize=(8,8))
    
    lab = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)  # convert from BGR to LAB color space
    l, a, b = cv2.split(lab)  # split on 3 different channels
    
    l2 = clahe.apply(l)  # apply CLAHE to the L-channel
    
    lab = cv2.merge((l2,a,b))  # merge channels
    cv_image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)  # convert from LAB to BGR

    # Gamma correction
    lookUpTable = np.empty((1,256), np.uint8)
    gamma = 0.7                               ##corrige la brightness non linéairement
    for i in range(256):
        lookUpTable[0,i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)
    cv_image = cv2.LUT(cv_image, lookUpTable)


    global mtx
    global dist
    global ids

    size_marker = 0.15 #en mètres !!

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)         ##indiquer le dictionnaire aruco utilisé ici il s'agit du 4x4_50 !
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)       ##ATTENTION : MISE A JOUR de la bibliotheque aruco : ce sont les ANCIENNES méthodes d'instanciation codée ici !!
    print('ids =', ids)

    
    cv_image = aruco.drawDetectedMarkers(cv_image, corners)

    global rvecs
    global tvecs
    global msg
    global msg_id
    global msg_bool
    global msg_pose
    global msg_centre
    global marqueur
    global centerx
    global centery
    global compteur
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, size_marker, mtx, dist)
    print(tvecs,'et',rvecs)
    

    if(rvecs is not None and tvecs is not None):
        compteur = 0
        print(len(ids))
        
        msg_id = ids[0][0]
        marqueur = 0
        for i in range(len(ids)):
            if msg_id>ids[i][0]:
                msg_id = ids[i][0]
                marqueur = i
   
        print("marqueur :",marqueur)

        print("corners =",corners)
        corners = corners[marqueur]
        centery = (corners[0][0][0]+corners[0][1][0]+corners[0][2][0]+corners[0][3][0])/4
        centerx = (corners[0][0][1]+corners[0][1][1]+corners[0][2][1]+corners[0][3][1])/4
        msg_centre.pose.position.x = centerx
        msg_centre.pose.position.y = centery
        
        for i in range(len(tvecs)):                                                                  
            length_of_axis = 0.10                                                                    
            cv_image = cv2.drawFrameAxes(cv_image, mtx, dist, rvecs[i,:,:], tvecs[i,:,:], length_of_axis)  

        msg_bool = True
        msg_pose.pose.position.x = tvecs[marqueur][0][0]
        msg_pose.pose.position.y = tvecs[marqueur][0][1]
        msg_pose.pose.position.z = tvecs[marqueur][0][2]
        msg_pose.pose.orientation.x = get_quaternion_from_euler(rvecs[marqueur][0][0],rvecs[marqueur][0][1],rvecs[marqueur][0][2])[0]   #checker le format de rvecs et tvecs
        msg_pose.pose.orientation.y = get_quaternion_from_euler(rvecs[marqueur][0][0],rvecs[marqueur][0][1],rvecs[marqueur][0][2])[1]
        msg_pose.pose.orientation.z = get_quaternion_from_euler(rvecs[marqueur][0][0],rvecs[marqueur][0][1],rvecs[marqueur][0][2])[2]
        msg_pose.pose.orientation.w = get_quaternion_from_euler(rvecs[marqueur][0][0],rvecs[marqueur][0][1],rvecs[marqueur][0][2])[3]
        
    
    else : 
        
        compteur = compteur + 1
    print("is_qrcode :",msg_bool)
    print("msg_id =",msg_id)
    print("centers = ",msg_centre.pose.position.y," ", msg_centre.pose.position.x)
    print("profondeur aruco :",msg_pose.pose.position.z)
    msg = bridge.cv2_to_imgmsg(cv_image, "bgr8") #rossification du message

    if compteur>200:
        msg_bool = False
        msg_id = -1

    return 1

def listener_and_talker():
    
    rospy.init_node('aruco_detection', anonymous=True)

    rospy.Subscriber("/bluerov_camera", Image, callback)   #ou /webcam/image_raw  pour tester  sinon bluerov_camera pour le robot

    #spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()


    pub = rospy.Publisher('bluerov_camera_aruco', Image, queue_size=10)
    pub2 = rospy.Publisher('bluerov_pose_aruco', PoseStamped,queue_size=10)
    pub3 = rospy.Publisher('id_qr_code_aruco',Float64,queue_size=10)
    pub4 = rospy.Publisher('is_qrcode',Bool,queue_size=10) #booleen de detection de qr_code
    pub5 = rospy.Publisher('centre_aruco_img',PoseStamped, queue_size=10)
    rate = rospy.Rate(20) 
    
    
    
    while not rospy.is_shutdown():    

        global msg
        global msg_pose
        global msg_id
        global msg_bool
        global msg_centre
        global rvecs
        global tvecs
        global marqueur
        #if(rvecs is not None and tvecs is not None):
        #    msg_pose.pose.position.x = tvecs[0][marqueur][0]
        #    msg_pose.pose.position.y = tvecs[0][marqueur][1]
        #    msg_pose.pose.position.z = tvecs[0][marqueur][2]
        #    msg_pose.pose.orientation.x = get_quaternion_from_euler(rvecs[0][marqueur][0],rvecs[0][marqueur][1],rvecs[0][marqueur][2])[0]   #checker le format de rvecs et tvecs
        #    msg_pose.pose.orientation.y = get_quaternion_from_euler(rvecs[0][marqueur][0],rvecs[0][marqueur][1],rvecs[0][marqueur][2])[1]
        #    msg_pose.pose.orientation.z = get_quaternion_from_euler(rvecs[0][marqueur][0],rvecs[0][marqueur][1],rvecs[0][marqueur][2])[2]
        #    msg_pose.pose.orientation.w = get_quaternion_from_euler(rvecs[0][marqueur][0],rvecs[0][marqueur][1],rvecs[0][marqueur][2])[3]

        pub.publish(msg)
        pub2.publish(msg_pose)
        pub3.publish(msg_id)
        pub4.publish(msg_bool)
        pub5.publish(msg_centre)
        rate.sleep()

if __name__ == '__main__':
    mtx = np.load("/home/hugo/projet_niche/src/Niche-sauvage/aruco_detect/src/calibration_cam_bluerov/matrices/camera_matrix.npy")           ##indiquer les valeurs de la calibration de la camera
    dist = np.load("/home/hugo/projet_niche/src/Niche-sauvage/aruco_detect/src/calibration_cam_bluerov/matrices/distortion_coeffs.npy")
    marqueur = 0
    rvecs = [[[0,0,0]]]
    tvecs = [[[0,0,0]]]
    ids  = -1
    centerx=0
    centery=0
    marqueur = 0
    compteur = 0
    msg = Image()
    msg_pose = PoseStamped()
    msg_id = Float64()
    msg_bool = Bool()
    msg_centre = PoseStamped()
    listener_and_talker()

  
