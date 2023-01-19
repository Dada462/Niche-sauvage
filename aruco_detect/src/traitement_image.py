#!/usr/bin/env python3
import sys
import rospy
import cv2
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

bridge = CvBridge()

def callback(data):
    global points0
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    qrDecoder = cv2.QRCodeDetector()

    
    info, points1, _  = qrDecoder.detectAndDecode(cv_image)
    if (points0 is not None) and (points1 is not None):
        points1 = points1[0]
        for i in range(len(points1)):
            pt1 = [int(val) for val in points1[i]]
            pt2 = [int(val) for val in points1[(i + 1) % 4]]
            #if (abs(pt1[0]-pt2[1])) < 200 and (abs(pt2[0]-pt1[1])) < 200 :
            cv2.line(cv_image, pt1, pt2, color=(255, 0, 0), thickness=3)
    points0 = points1
    #cv2.imshow('Detected QR code', cv_image)
    #cv2.waitKey(3) 
    
    global msg
    msg = bridge.cv2_to_imgmsg(cv_image, "bgr8") #rossification du message
    
    return msg

def listener_and_talker():
    
    rospy.init_node('tracker', anonymous=True)

    rospy.Subscriber("bluerov_camera", Image, callback)

    #spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()


    pub = rospy.Publisher('bluerov_camera_qr_code', Image, queue_size=10)
    
    rate = rospy.Rate(20) 
    
    
    
    while not rospy.is_shutdown():    
        global msg
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    points0 = None
    msg = Image()
    listener_and_talker()

  
