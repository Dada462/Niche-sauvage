import sys
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
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
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    mtx = np.load("camera_calibration/camera_matrix.npy")           ##indiquer les valeurs de la calibration de la camera
    dist = np.load("camera_calibration/distortion_coeffs.npy")
    length = 0.15
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)         ##indiquer le dictionnaire aruco utilisé ici il s'agit du 4x4_50 
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, cameraMatrix=mtx, distCoeff=dist, parameters=arucoParameters)
    cv_image = aruco.drawDetectedMarkers(cv_image, corners)
    global rvecs
    global tvecs
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, length, mtx, dist)

    #cv2.imshow('Detected QR code', cv_image)
    
    #cv2.waitKey(3) 
    if(rvecs is not None and tvecs is not None):
        cv_image = aruco.drawAxis(cv_image, mtx, dist, rvecs, tvecs, length)
    global msg

    msg = bridge.cv2_to_imgmsg(cv_image, "bgr8") #rossification du message
    
    return msg

def listener_and_talker():
    
    rospy.init_node('aruco_detection', anonymous=True)

    rospy.Subscriber("bluerov_camera", Image, callback)

    #spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()


    pub = rospy.Publisher('bluerov_camera_aruco', Image, queue_size=10)
    pub2 = rospy.Publisher('bluerov_pose_aruco', Pose)            ## TO DO : Création de son propre message pour ajouter l'id du marqueur et connaitre type de l'ids
    rate = rospy.Rate(20) 
    
    
    
    while not rospy.is_shutdown():    
        global msg
        global msg_pose
        if (rvecs is not None and tvecs is not None) :
            msg_pose.position = rvecs
            msg_pose.orientation = get_quaternion_from_euler(tvecs[0],tvecs[1],tvecs[2])   #checker le format de rvecs et tvecs
        else :
            msg_pose.position = None
            msg_pose.orientation = None

        pub.publish(msg)
        pub2.publish(msg_pose)
        rate.sleep()

if __name__ == '__main__':
    rvecs = None
    tvecs = None
    msg = Image()
    msg_pose = PoseStamped()
    listener_and_talker()

  