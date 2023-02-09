import rospy
from geometry_msgs.msg import Point


import numpy as np

def talker():
    pub = rospy.Publisher('chatter', Point, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    p=Point()
    t=0
    dt=0.05
    while not rospy.is_shutdown():
        x,y=5*np.cos(t),5*np.sin(2*t)
        t+=dt
        p.x=x
        p.y=y
        p.z=np.random.rand()
        pub.publish(p)
        rate.sleep()

talker()