import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
import random
import urx
import time
import math
import os
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

# robot
robot = urx.Robot("192.168.1.66")

class DopeController(object):
    def __init__(self):
        #self.bridge = CvBridge()
        self.pan_pose = None
        pan_handle_topic = '/dope/pose_pan_handle_handle'
        oil_bowl_topic = '/dope/pose_oil_bowl'

        rospy.Subscriber(pan_handle_topic, PoseStamped, self.pan_handle_callback)
        rospy.Subscriber(oil_bowl_topic, PoseStamped, self.oil_bowl_callback)

        # rospy.Subscriber('/dope/rgb_points', Image, self.dope_image_callback)

    def pan_handle_callback(self, data):
        self.pan_pose = data.pose

    def get_pan_pose(self):
        if self.pan_pose is None:
            return None
        return self.pan_pose.position, self.pan_pose.orientation

    def oil_bowl_callback(self, data):
        self.oil_bowl_pose = data.pose

    def get_oil_bowl_pose(self):
        if self.oil_bowl_pose is None:
            return None
        return self.oil_bowl_pose.position, self.oil_bowl_pose.orientation


def convert(dope):
    x = dope[0] / 20-0.06
    y = -(dope[1] / 20 - 0.07)
    z = -(dope[2] / 20 - 0.15)

    h = [0, 0, 0, 0, 0, 0]
    v = [0, 0, 0, 0, 0, 0]

    b = x / math.sqrt(2)
    c = -x / math.sqrt(2)
    a = y

    h[0] = a
    h[1] = b
    h[2] = c

    b = - z / math.sqrt(2)
    c = - z / math.sqrt(2)
    a = 0

    v[0] = a
    v[1] = b
    v[2] = c

    return h, v

def convert_c(dope):
    x = dope[0] / 20 -0.05
    y = -(dope[1] / 20-0.1)
    z = -(dope[2] / 20 - 0.22)

    h = [0, 0, 0, 0, 0, 0]
    v = [0, 0, 0, 0, 0, 0]

    b = x / math.sqrt(2)
    c = -x / math.sqrt(2)
    a = y

    h[0] = a
    h[1] = b
    h[2] = c

    b = - z / math.sqrt(2)
    c = - z / math.sqrt(2)
    a = 0

    v[0] = a
    v[1] = b
    v[2] = c

    return h, v

def robot_move(h, v, acc, vel):
    robot.movel(h, acc, vel, relative=True)
    time.sleep(1)
    robot.movel(v, acc, vel, relative=True)
    gripper.close_gripper()
    time.sleep(3)
    gripper.open_gripper()



if __name__ == '__main__':
    rospy.init_node('dope_rospy_youngjae_test')

    dope_confroller = DopeController()
    rospy.sleep(1)
    gripper = Robotiq_Two_Finger_Gripper(robot)
    # init position joint state
    init = [0.39821290969848633, -4.948345323602194, 1.713571850453512, 4.439863844508789, -2.273259941731588,
            2.5900216102600098]

    a = raw_input("object : ")
    robot.movej(init, 0.2, 0.2, relative=False)
    gripper.open_gripper()
    if int(a) == 1:
        pan_pose, pan_orient = dope_confroller.get_pan_pose()
        print('pan_pose x, y, z', pan_pose.x, pan_pose.y, pan_pose.z)
        print('pan_orient x, y, z, w', pan_orient.x, pan_orient.y, pan_orient.z, pan_orient.w)
        dope_list = [pan_pose.x, pan_pose.y,pan_pose.z,0,0,0]
        h, v = convert(dope_list)
        robot_move(h, v, 0.1, 0.1)
    else :
        oil_bowl_pose, oil_bowl_orient = dope_confroller.get_oil_bowl_pose()
        print('oil_bowl_pose x, y, z', oil_bowl_pose.x, oil_bowl_pose.y, oil_bowl_pose.z)
        print('oil_bowl_orient x, y, z, w', oil_bowl_orient.x, oil_bowl_orient.y, oil_bowl_orient.z, oil_bowl_orient.w)

        dope_list = [oil_bowl_pose.x, oil_bowl_pose.y, oil_bowl_pose.z, 0, 0, 0]
        h, v = convert_c(dope_list)
        robot_move(h, v, 0.1, 0.1)
