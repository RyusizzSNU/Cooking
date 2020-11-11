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
    L_B_A stands for an affine transformation from A to B
    O : object
    C : camera(attached to wrist)
    W : robot wrist 
    B : robot base   
'''
class Agent():
    initial_pos = [0.4, 0.1, -0.25]
    initial_rot = [[0, 1, 0], [-math.sqrt(2) / 2, 0, math.sqrt(2) / 2],
                   [math.sqrt(2) / 2, 0, math.sqrt(2) / 2]]
    initial_joint = [0.39821290969848633, -4.948345323602194, 1.713571850453512, 4.439863844508789, -2.273259941731588, 2.5900216102600098]
    initial_joint_for_board_handle = [0.847, -3.475, 1.747, 3.979, -4.248, 5.67]
    integrade_joint = [0.3664, -3.1741, 2.3328, 3.6704, 1.8027, -0.5058]
    #initial_joint_90 = [0.398248, -4.94837, 1.71355, 4.43986, -2.27327, 1.01928]
    L_W_C = np.array([[-1, 0, 0, 0.05], [0, -1, 0, 0.085], [0, 0, 1, 0.03], [0, 0, 0, 1]])
    # appropriate wrist position relative to object pivot(in object coordinate system) for gripping
    v_wrist_dict = {
        'carrot' : np.array([0.165, 0, -0.03, 1]),
        'pan_handle_handle' : np.array([0, 0.17, -0.02, 1]),
        'rice_bowl' : np.array([-0.035, -0.24, 0, 1]),
        'oil_bowl' : np.array([-0.20, -0.03, 0, 1]),
        'salt_bowl' : np.array([-0.18, 0, 0, 1]),
        'board_handle' : np.array([-0.17, 0, 0, 1])
    }
    # appropriate wrist orientation relative to object orientation for gripping
    r_O_W_dict = {
        'carrot': np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]]),
        'pan_handle_handle': np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]]),
        'rice_bowl' : np.array([[0, -1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]),
        'oil_bowl' : np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]),
        'salt_bowl' : np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]),
        'board_handle' : np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
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
        self.robot.movej(self.initial_joint, 0.1, 0.1)
        #self.robot.movej(self.initial_joint_for_board_handle, 0.1, 0.1)

    def grip(self, obj):
        print(self.robot.getl())
        print(self.robot.getj())
        # scale : 1 for board handle, 1/20 for others
        scale = 0.05
        if obj == 'board_handle':
            scale = 1
        L_C_O = utils.dope_to_affine(self.dope_reader.get_obj_pos(obj), scale=scale)
        L_B_W = utils.tcp_to_affine(self.robot.getl())
        print(L_C_O)
        print(L_B_W)

        target_position = np.matmul(
            L_B_W, np.matmul(
                self.L_W_C, np.matmul(
                    L_C_O, self.v_wrist_dict[obj]
                )
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
        if obj == 'rice_bowl':
            self.robot.movej(self.integrade_joint, 0.1, 0.1)
        self.robot.movel(np.concatenate([target_position, target_orientation]), 0.1, 0.1, relative=False)
        self.gripper.close_gripper()

if __name__ == '__main__':
    agent = Agent()
    agent.ready()
    time.sleep(1)
    agent.grip('oil_bowl')
    agent.robot.close()
