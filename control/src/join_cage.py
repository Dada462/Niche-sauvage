from tools import sawtooth, R, mat_reading, path_info_update, sawtooth
from bluerov_msgs.msg import CommandBluerov
from geometry_msgs.msg import Point, Quaternion, Pose
from std_msgs.msg import Float64
from usbl.msg import Usbl
import numpy as np
from numpy import tanh, pi, cos, sin
import rospy
import time


class Controller():
    def __init__(self):
        self.usbl_data = Usbl()
        self.heading = 0.
        self.depths=[[0.,0.]]
        self.state = np.array([0., 0., 0., 0.]) # x,y,heading relative to the cage,s
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

    def heading_command(self, desired_heading):
        # print("Heading=", self.heading)
        # print("Heading from cage=", self.NED_to_cage(self.heading)*180/pi)
        k = .5
        u = -2*tanh(k*sawtooth(desired_heading-self.heading/180*pi))
        return u

    def depth_test(self):
        # if self.desired_position.w == 1:
        X = self.depths[-1][0]
        V=(self.depths[-1][0]-self.depths[-2][0])/(self.depths[-1][1]-self.depths[-2][1])
        x_desired = self.desired_position.z
        k = 1  # Saturation gain
        dk=0.7
        u = tanh(k*(x_desired-X)-dk*V)
        return u
        print(u,'V',V,'xdes',x_desired,'z',X)
            # self.create_control_message(
                # Point(0.,0., u), Quaternion(0., 0., 0., 0.))
        # else:
            # self.create_control_message(
                # Point(0., 0., 0.), Quaternion(0., 0., 0., 0.))
    
    def join_the_cage(self):
        if self.desired_position.w == 1:
            X = np.array([self.usbl_data.position.x,
                          self.usbl_data.position.y])
            cage_heading = self.NED_to_cage(self.heading)

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
            u = R(cage_heading).T@(x_desired-X)
            u = tanh(k*u)

            desired_heading = -np.arctan2(-X[1], -X[0])*180/pi
            k = .5
            heading_control = -2 * tanh(k*sawtooth((desired_heading-self.heading+90)/180*np.pi))
            ################ Control ################
            self.create_control_message(
                Point(u[0], u[1], self.depth_test()), Quaternion(0., 0., heading_control, 0.))
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

    def update_state(self):
        X = np.array([self.usbl_data.position.x,
                      self.usbl_data.position.y])
        cage_heading = self.NED_to_cage(self.heading)
        self.state[:2] = X
        self.state[2] = cage_heading


def ros_usbl(data):
    global ROV_Controller
    try:
        ROV_Controller.usbl_data = data
    except:
        pass

def ros_depth(data):
    global ROV_Controller
    try:
        ROV_Controller.depths.append([data.data,time.time()])
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
    rospy.init_node('command_giver', anonymous=True)
    pub = rospy.Publisher('/commande', CommandBluerov, queue_size=10)
    rospy.Subscriber("/usbl", Usbl, ros_usbl)
    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, ros_depth)
    rospy.Subscriber("/mavros/global_position/compass_hdg",
                     Float64, ros_compass)
    rospy.Subscriber("/desired_position", Quaternion, ros_desired_position)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ROV_Controller.join_the_cage()
        # ROV_Controller.depth_test()
        # ROV_Controller.heading_test(180)
        # ROV_Controller.join_a_point_test()
        command = ROV_Controller.command
        pub.publish(command)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
