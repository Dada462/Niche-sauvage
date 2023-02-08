from tools import sawtooth, R, mat_reading, sawtooth
from bluerov_msgs.msg import CommandBluerov
from geometry_msgs.msg import Point, Quaternion, Pose
from std_msgs.msg import Float64
from usbl.msg import Usbl
import numpy as np
from numpy import tanh, pi, cos, sin
import rospy


class Controller():
    def __init__(self):
        self.usbl_data = Usbl()
        self.heading = 0.
        self.path_to_follow = mat_reading(lambda x, y: (x, x))

        ################ Initial Control set to 0 so that the robot does not move ################
        self.command = CommandBluerov()
        self.command.pose.position = Point(0., 0., 0.)
        self.command.pose.orientation = Quaternion(0., 0., 0., 0.)
        self.command.light = 0
        self.command.power = 0
        self.command.arming = 0
        self.desired_position = Quaternion(0., 0., 0., 0)
        ################ Initial Control set to 0 so that the robot does not move ################

    def heading_test(self, desired_heading):
        k = .5
        u = -2*tanh(k*sawtooth((desired_heading-self.heading)/180*np.pi))
        print('Heading=', self.heading, 'command', u)
        if self.desired_position.w == 1:
            self.create_control_message(
                Point(0., 0., 0.), Quaternion(0., 0., u, 0.))
        else:
            self.create_control_message(
                Point(0., 0., 0.), Quaternion(0., 0., 0., 0.))

    def NED_to_cage(self, heading):
        cage_heading = 90  # deg
        return -sawtooth((heading-cage_heading)/180*pi)

    def join_a_point_test(self):
        # print(self.NED_to_cage(self.heading)/pi*180)
        if self.desired_position.w == 1:
            s = 0.
            X = np.array([self.usbl_data.position.x,
                         self.usbl_data.position.y])
            heading = self.NED_to_cage(self.heading)
            ################ To be implemented later ################
            # F = path_info_update(self.path_to_follow, s)
            # theta_c = F.psi
            # s1, y1 = R(theta_c).T@(X-F.X)
            # theta = sawtooth(theta-theta_c)
            # psi = sawtooth(theta)
            # ks = 1
            # nu=1
            # ds = np.cos(psi)*nu + ks*s1
            ################ To be implemented later ################

            ################ Control ################
            x_desired = np.array(
                [self.desired_position.x, self.desired_position.y])
            k = .7  # Saturation gain
            u = R(heading).T@(x_desired-X)
            u = tanh(k*u)
            ################ Control ################
            self.create_control_message(
                Point(u[0], u[1], 0.), Quaternion(0., 0., 0., 0.))
        else:
            self.create_control_message(
                Point(0., 0., 0.), Quaternion(0., 0., 0., 0.))

    def create_control_message(self, P, Q, arming=1, power=100, light=0):
        self.command = CommandBluerov()
        self.command.pose.position = P
        self.command.pose.orientation = Q
        self.command.light = light
        self.command.power = power
        self.command.arming = arming


def ros_usbl(data):
    global ROV_Controller
    try:
        ROV_Controller.usbl_data = data
    except:
        pass


def ros_compass(data):
    global ROV_Controller
    try:
        ROV_Controller.heading = data.data
    except:
        pass


def ros_desired_position(data):
    global ROV_Controller
    try:
        ROV_Controller.desired_position = data
    except:
        pass


def main():
    global ROV_Controller
    ROV_Controller = Controller()
    rospy.init_node('command_tester', anonymous=True)
    pub = rospy.Publisher('/commande', CommandBluerov, queue_size=10)
    rospy.Subscriber("/usbl", Usbl, ros_usbl)
    rospy.Subscriber("/mavros/global_position/compass_hdg",
                     Float64, ros_compass)
    rospy.Subscriber("/desired_position", Quaternion, ros_desired_position)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ROV_Controller.join_a_point_test()
        # ROV_Controller.heading_test()
        command = ROV_Controller.command
        pub.publish(command)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
