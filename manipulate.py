import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
import random
import urx
import time
import math
import os
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
# robot
robot = urx.Robot("192.168.1.66")

class DopeController(object):
    def __init__(self):
        #self.bridge = CvBridge()
        self.objects = []
        self.obj_poses = {}

        pan_handle_topic = '/dope/pose_pan_handle_handle'
        oil_bowl_topic = '/dope/pose_oil_bowl'
        carrot_topic = '/dope/pose_carrot'

        rospy.Subscriber(pan_handle_topic, PoseStamped, self.pan_handle_callback)
        rospy.Subscriber(oil_bowl_topic, PoseStamped, self.oil_bowl_callback)
        rospy.Subscriber(carrot_topic, PoseStamped, self.carrot_callback)

        # rospy.Subscriber('/dope/rgb_points', Image, self.dope_image_callback

    def carrot_callback(self, data):
        self.obj_poses['carrot'] = data.pose

    def pan_handle_callback(self, data):
        self.obj_poses['pan'] = data.pose

    def oil_bowl_callback(self, data):
        self.obj_poses['oil_bowl'] = data.pose

    def get_obj_pos(self, obj):
        if not obj in self.obj_poses:
            return None
        p = self.obj_poses[obj].position
        o = self.obj_poses[obj].orientation
        return [p.x, p.y, p.z], [o.x, o.y, o.z, o.w]

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

    obj = 'carrot'
    #obj = raw_input("object : ")
    robot.movej(init, 0.2, 0.2, relative=False)
    #gripper.open_gripper()

    v = np.array([0, 0, 0])
    r = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
    L_B_C = np.array([[0, -1, 0], [1 / math.sqrt(2), 0, 1 / math.sqrt(2)], [-1/math.sqrt(2), 0, 1/math.sqrt(2)]])

    p_C, L_C_O = dope_confroller.get_obj_pos(obj)

    print(p_C, L_C_O)

    #rxyz = robot.get_orientation()
    rxyz = robot.getl()
    rxyz = np.array(rxyz) * -2.62
    #print(np.array(rxyz) * -2.62)

    #print(robot.secmon.get_all_data())
    #L_B_C = R.from_euler('zyx', [rxyz[3], rxyz[4], rxyz[5]])
    #L_B_C = L_B_C.as_dcm()

    L_C_O = R.from_quat(L_C_O)
    L_C_O = L_C_O.as_dcm()

    target_position = (np.matmul(L_B_C, p_C) + np.matmul(L_B_C, np.matmul(L_C_O, v))) / 20
    target_orientation = np.matmul(L_B_C, np.matmul(L_C_O, r))

    target_orientation = R.from_dcm(target_orientation)
    target_orientation = target_orientation.as_euler('zyx')

    print(target_position, target_orientation)

    target_orientation = np.array([0, 0, 0])

    #robot.movel(np.concatenate((target_position, target_orientation)), 0.1, 0.1, relative=True)

    '''if int(a) == 1:
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
        robot_move(h, v, 0.1, 0.1)'''
