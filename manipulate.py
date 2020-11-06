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

'''
Definitions:
    L_A_B stands for an affine transformation from A to B
    O : object
    C : camera(attached to wrist)
    W : robot wrist 
    B : robot base   
'''
class Agent():
    initial_joint = [0.39821290969848633, -4.948345323602194, 1.713571850453512, 4.439863844508789, -2.273259941731588, 2.5900216102600098]
    initial_joint_90 = [0.398248, -4.94837, 1.71355, 4.43986, -2.27327, 1.01928]
    L_W_C = np.array([[-1, 0, 0, 0.05], [0, -1, 0, 0.085], [0, 0, 1, 0.03], [0, 0, 0, 1]])
    # object center position relative to object pivot(in object coordinate system)
    v_center_dict = {
        'carrot' : np.array([0, 0, 0, 1]),
        'pan_handle_handle' : np.array([0, 0, -0.02, 1])
    }
    # appropriate wrist position relative to object center(in wrist coordinate system) for gripping
    v_wrist_dict = {
        'carrot' : np.array([0, 0, -0.165, 0]),
        'pan_handle_handle' : np.array([0, 0, -0.165, 0])
    }
    # appropriate wrist orientation relative to object orientation for gripping
    r_O_W_dict = {
        'carrot': np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]]),
        'pan_handle_handle': np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
    }

    def __init__(self):
        # robot
        rospy.init_node('dope_reader')
        self.robot = urx.Robot("192.168.1.66")
        self.gripper = Robotiq_Two_Finger_Gripper(self.robot)
        self.dope_reader = DopeReader()
        rospy.sleep(1)

    def ready(self):
        self.gripper.open_gripper()
        self.robot.movej(self.initial_joint, 0.5, 0.5, relative=False)

    def grip(self, obj):
        L_C_O = utils.dope_to_affine(self.dope_reader.get_obj_pos(obj))
        L_B_W = utils.tcp_to_affine(self.robot.getl())
        print(L_C_O)
        print(L_B_W)

        target_position = np.matmul(
            L_B_W, np.matmul(
                self.L_W_C, np.matmul(
                    L_C_O, self.v_center_dict[obj]
                )
            ) + self.v_wrist_dict[obj]
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

if __name__ == '__main__':
    agent = Agent()
    agent.ready()
    agent.grip('pan_handle_handle')
    agent.robot.close()
