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
    initial_pos = {'left' : [0.4, 0.1, -0.25], 'right' : [-0.7035, -0.0028, -0.1604]}
    initial_rot = {'left' : [[0, 1, 0], [-math.sqrt(2) / 2, 0, math.sqrt(2) / 2], [math.sqrt(2) / 2, 0, math.sqrt(2) / 2]],
                'right' : [[0, -1, 0], [math.sqrt(2) / 2, 0, math.sqrt(2) / 2], [-math.sqrt(2) / 2, 0, math.sqrt(2) / 2]]}

    initial_joint = {'left' : [0.3982, -4.9483, 1.7136, 4.4399, -2.2733, 2.5900],
                     'right' : [-2.3372, -4.3113, 0.9498, -0.7260, -1.0605, 1.3825]}

    initial_joint_for_board_handle = {'left' : [0.847, -3.475, 1.747, 3.979, -4.248, 5.67]}
    integrade_joint_for_rice_bowl = {'left' : [0.3664, -3.1741, 2.3328, 3.6704, 1.8027, -0.5058]}


    L_W_C = {'left' : np.array([[-1, 0, 0, 0.05], [0, -1, 0, 0.085], [0, 0, 1, 0.03], [0, 0, 0, 1]]),
            'right' : np.array([[-1, 0, 0, 0.05], [0, -1, 0, 0.085], [0, 0, 1, 0.03], [0, 0, 0, 1]])}
    # appropriate wrist position relative to object pivot(in object coordinate system) for gripping
    v_wrist_dict = {
        'carrot' : np.array([0.165, 0, -0.03, 1]),
        'pan_handle_handle' : np.array([0, 0.17, -0.02, 1]),
        'rice_bowl' : np.array([-0.035, -0.24, 0, 1]),
        'oil_bowl' : np.array([-0.20, -0.03, 0, 1]),
        'salt_bowl' : np.array([-0.18, 0, 0, 1]),
        'board_handle' : np.array([-0.17, 0, 0, 1]),
        'knife_handle' : np.array([-0.18, 0.02, 0, 1]),
        'paddle_handle' : np.array([-0.0, 0.18, 0, 1])
    }
    # appropriate wrist orientation relative to object orientation for gripping
    r_O_W_dict = {
        'carrot': np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]]),
        'pan_handle_handle': np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]]),
        'rice_bowl' : np.array([[0, -1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]),
        'oil_bowl' : np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]),
        'salt_bowl' : np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]),
        'board_handle' : np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]]),
        'knife_handle' : np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]]),
        'paddle_handle' : np.array([[-1, 0, 0, 0], [0, 0, -1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
    }

    def __init__(self):
        # robot
        rospy.init_node('dope_reader')
        self.robot = {}
        self.dope_reader = {'left' : DopeReader('left'), 'right' : DopeReader('right')}
        rospy.sleep(1)

    def ready(self, side):
        if side == 'left':
            self.robot['left'] = urx.Robot("192.168.1.66")
            self.gripper = Robotiq_Two_Finger_Gripper(self.robot['left'])
            self.gripper.open_gripper()
        if side == 'right':
            self.robot['right'] = urx.Robot("192.168.1.109")

        self.robot[side].movej(self.initial_joint[side], 0.1, 0.1)
        #self.robot.movej(self.initial_joint_for_board_handle, 0.1, 0.1)
        print(self.robot[side].getl())
        print(self.robot[side].getj())

    def grip(self, side, obj):
        # scale : 1 for handles (except for pan_handle_handle), 1/20 for others
        scale = 0.05
        if obj == 'board_handle' or obj == 'knife_handle' or obj == 'paddle_handle':
            scale = 1
        L_C_O = utils.dope_to_affine(self.dope_reader[side].get_obj_pos(obj), scale=scale)
        L_B_W = utils.tcp_to_affine(self.robot[side].getl())
        print(L_C_O)
        print(L_B_W)

        target_position = np.matmul(
            L_B_W, np.matmul(
                self.L_W_C[side], np.matmul(
                    L_C_O, self.v_wrist_dict[obj]
                )
            )
        )[:3]

        target_orientation = R.from_dcm(utils.to_33(np.matmul(
            L_B_W, np.matmul(
                # rotation offset
                utils.to_44(R.from_rotvec([0, -0, 0]).as_dcm()), np.matmul(
                    self.L_W_C[side], np.matmul(
                        L_C_O, self.r_O_W_dict[obj]
                    )
                )
            )
        ))).as_rotvec()
        print(target_position, target_orientation)
        if obj == 'rice_bowl':
            self.robot[side].movej(self.integrade_joint_for_rice_bowl[side], 0.1, 0.1)
        self.robot[side].movel(np.concatenate([target_position, target_orientation]), 0.1, 0.1, relative=False)
        if side == 'left' :
            self.gripper.close_gripper()

    def close(self):
        for side in self.robot:
            self.robot[side].close()

if __name__ == '__main__':
    agent = Agent()
    agent.ready('right')
    time.sleep(1)
    agent.grip('right', 'paddle_handle')
    agent.close()
