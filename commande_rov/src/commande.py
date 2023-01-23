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
    

    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    roll_x, pitch_y, yaw_z = euler_from_quaternion(data.orientation.x,data.orientation.x,data.orientation.x,data.orientation.x)
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
    
    rospy.init_node('aruco_commande', anonymous=True)

    rospy.Subscriber("bluerov_pose_aruco", Pose, callback)

    #spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()


    pub = rospy.Publisher('bluerov_commande_suivi_aruco', Message, queue_size=10)
    
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

  