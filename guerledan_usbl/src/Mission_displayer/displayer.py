from turtle import heading
import numpy as np
from matplotlib import pyplot as plt
from geometry_msgs.msg import Point
import rospy
from tools import R
from numpy import pi
from guerledan_usbl.msg import USBL




def callback(msg):
    global Path
    try:
        # x,y,heading,depth,azimuth,elevation,range=data
        data=msg.position
        x,y,heading,depth,azimuth,elevation,range=data.x, data.y, 1,msg.depth,msg.azimuth, msg.elevation, msg.range
        Path.append([x,y])
        plt.clf()
        xmin,xmax,ymin,ymax=-25,25,-25,25
        plt.xlim(xmin,xmax)
        plt.ylim(ymin,ymax)
        X=np.array([x,y])
        heading=135
        robot_body= X.reshape((2,1))+3*R(pi*heading/180)@(np.array([[0.5,0], [-0.5,0.25], [-0.5,-0.25]]).T)
        t1 = plt.Polygon(robot_body.T[:3,:],color='#F15020')
        plt.gca().add_patch(t1)
        info_1='heading (deg): '  + str(round(heading,2)) + '\n' + 'depth (m):'+str(round(depth,2))
        info_2='azimuth (m):'+str(round(azimuth,2))+ '\n'+'elevation (m):'+str(round(elevation,2))+ '\n'+ 'range (m):'+str(round(range,2))
        plt.text(15,27, info_1, style='italic',
                bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 5})
        plt.text(-23,26, info_2, style='italic',
                bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 5})
        plt.plot(*np.array(Path).T,color='red')
        plt.draw()
        plt.pause(0.00000000001)
    except RuntimeError:
        print('Error')


if __name__ == '__main__':
    Path=[]
    rospy.init_node("plotter")
    rospy.Subscriber("Informations", USBL, callback)
    plt.ion()
    plt.show()
    rospy.spin()