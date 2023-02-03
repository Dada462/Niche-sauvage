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
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, size_marker, mtx, dist)
    print(tvecs,'et',rvecs)
    msg_bool = False

    if(rvecs is not None and tvecs is not None):
        for i in range(len(tvecs)):                                                                  
            length_of_axis = 0.10                                                                    
            cv_image = cv2.drawFrameAxes(cv_image, mtx, dist, rvecs[i,:,:], tvecs[i,:,:], length_of_axis)  
        msg_bool = True
        
        msg_id = ids[0][0]
    else : 
        msg_id = -1
    print("is_qrcode :",msg_bool)
    msg = bridge.cv2_to_imgmsg(cv_image, "bgr8") #rossification du message
    return 1

def listener_and_talker():
    
    rospy.init_node('aruco_detection', anonymous=True)

    rospy.Subscriber("/webcam/image_raw", Image, callback)   #ou /webcam/image_raw  pour tester  sinon bluerov_camera pour le robot

    #spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()


    pub = rospy.Publisher('bluerov_camera_aruco', Image, queue_size=10)
    pub2 = rospy.Publisher('bluerov_pose_aruco', PoseStamped,queue_size=10)
    pub3 = rospy.Publisher('id_qr_code_aruco',Float64,queue_size=10)
    pub4 = rospy.Publisher('is_qrcode',Bool,queue_size=10) #booleen de detection de qr_code
    rate = rospy.Rate(20) 
    
    
    
    while not rospy.is_shutdown():    

        global msg
        global msg_pose
        global msg_id
        global msg_bool
        if(rvecs is not None and tvecs is not None):
            msg_pose.pose.position.x = tvecs[0][0][0]
            msg_pose.pose.position.y = tvecs[0][0][1]
            msg_pose.pose.position.z = tvecs[0][0][2]
            msg_pose.pose.orientation.x = get_quaternion_from_euler(rvecs[0][0][0],rvecs[0][0][1],rvecs[0][0][2])[0]   #checker le format de rvecs et tvecs
            msg_pose.pose.orientation.y = get_quaternion_from_euler(rvecs[0][0][0],rvecs[0][0][1],rvecs[0][0][2])[1]
            msg_pose.pose.orientation.z = get_quaternion_from_euler(rvecs[0][0][0],rvecs[0][0][1],rvecs[0][0][2])[2]
            msg_pose.pose.orientation.w = get_quaternion_from_euler(rvecs[0][0][0],rvecs[0][0][1],rvecs[0][0][2])[3]

        pub.publish(msg)
        pub2.publish(msg_pose)
        pub3.publish(msg_id)
        pub4.publish(msg_bool)
        rate.sleep()

if __name__ == '__main__':
    mtx = np.load("/home/hugo/projet_niche/src/Niche-sauvage/qr_code_detect/src/calibration_camera/camera_matrix.npy")           ##indiquer les valeurs de la calibration de la camera
    dist = np.load("/home/hugo/projet_niche/src/Niche-sauvage/qr_code_detect/src/calibration_camera/distortion_coeffs.npy")
    
    rvecs = [[[0,0,0]]]
    tvecs = [[[0,0,0]]]
    ids  = -1
    msg = Image()
    msg_pose = PoseStamped()
    msg_id = Float64()
    msg_bool = Bool()
    listener_and_talker()

  
