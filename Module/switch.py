import rospy
import urx
import time
import math
import os
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from scipy.spatial.transform import Rotation as R
import numpy as np
import utils
from dope_reader import DopeReader


class left():
    init_doma = [0.5786294937133789, -3.6818372211852015, 1.6158259550677698, -1.6948534450926722, 2.4871182441711426, 5.794440269470215]
    init_switch = [1.3417811393737793, -3.7575494251646937, 1.7968886534320276, 2.6484896379658203, -0.6016557852374476, 1.7406048774719238]
    init_top = [0.39821290969848633, -4.948345323602194, 1.713571850453512, 4.439863844508789, -2.273259941731588, 2.5900216102600098]

    def __init__(self):
        rospy.init_node('dope_reader')
        self.robot = urx.Robot("192.168.1.66")
        self.gripper = Robotiq_Two_Finger_Gripper(self.robot)
        self.dope_reader = DopeReader()
        rospy.sleep(1)

    def ready(self):
        self.gripper.open_gripper()
        self.robot.movej(self.initial_top, 0.5, 0.5, relative=False)

    def grip(self, obj): #grip_food
        L_C_O = utils.dope_to_affine(self.dope_reader.get_obj_pos(obj))
        L_B_W = utils.tcp_to_affine(self.robot.getl())
        print(L_C_O)
        print(L_B_W)

        target_position = np.matmul(
            L_B_W, np.matmul(
                self.L_W_C, np.matmul(
                    L_C_O, self.v_center_dict[obj]
                )  # + self.v_wrist_dict[obj]
            )
        )[:3]
        target_orientation = R.from_dcm(utils.to_33(np.matmul(
            L_B_W, np.matmul(
                # rotation offset
                utils.to_44(R.from_rotvec([0, -0, 0]).as_dcm()), np.matmul(
                    self.L_W_C, np.matmul(
                        L_C_O, self.r_O_W_dict[obj]
                    )
                )
            )
        ))).as_rotvec()
        print(target_position, target_orientation)
        self.robot.movel(np.concatenate([target_position, target_orientation]), 0.2, 0.2, relative=False)
        self.gripper.close_gripper()

    def turnSwitch(self, angle):
        self.robot.movej(self.init_switch, 0.5, 0.5, relative=False)
        rospy.sleep(2)
        L_C_O = utils.dope_to_affine(self.dope_reader.get_obj_pos(switch))
        L_B_W = utils.tcp_to_affine(self.robot.getl())
        print(L_C_O)
        print(L_B_W)

        target_position = np.matmul(
            L_B_W, np.matmul(
                self.L_W_C, np.matmul(
                    L_C_O, self.v_center_dict[switch]
                )  # + self.v_wrist_dict[obj]
            )
        )[:3]
        target_orientation = R.from_dcm(utils.to_33(np.matmul(
            L_B_W, np.matmul(
                # rotation offset
                utils.to_44(R.from_rotvec([0, -0, 0]).as_dcm()), np.matmul(
                    self.L_W_C, np.matmul(
                        L_C_O, self.r_O_W_dict[switch]
                    )
                )
            )
        ))).as_rotvec()
        print(target_position, target_orientation)

        self.robot.movel(np.concatenate([target_position, target_orientation]), 0.2, 0.2, relative=False)
        self.gripper.close_gripper()
        self.robot.movej([0, 0, 0, 0, 0, math.radians(angle)], 0.2, 0.2, relative=True)#clock_wise
        rospy.sleep(2)
        self.gripper.open_gripper()

if __name__ == '__main__':
    agent =  left()
    agent.ready()
    agent.turnSwitch(30)#degree to radian
    agent.robot.close()