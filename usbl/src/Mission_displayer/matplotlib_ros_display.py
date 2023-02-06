import numpy as np
from matplotlib import pyplot as plt
from geometry_msgs.msg import Point
import rospy

def plot_x(msg):
    global Path
    plt.clf()
    plt.xlim(-1,1)
    plt.ylim(-1,1)
    Path.append([msg.x,msg.y])
    plt.plot(*np.array(Path).T,color='red')
    plt.draw()
    plt.pause(0.00000000001)


if __name__ == '__main__':
    Path=[]
    rospy.init_node("plotter")
    rospy.Subscriber("chatter", Point, plot_x)
    plt.ion()
    plt.show()
    rospy.spin()