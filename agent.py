import rospy
import urx
import time
import math
import os

#from typing import Dict, List
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
    idle_pos = {'left' : [0.4, 0.1, -0.25], 'right' : [-0.7035, -0.0028, -0.1604]}
    idle_rot = {'left' : [[0, 1, 0], [- 1 / math.sqrt(2), 0, 1 / math.sqrt(2)], [1 / math.sqrt(2), 0, 1 / math.sqrt(2)]],
                'right' : [[0, -1, 0], [1 / math.sqrt(2), 0, 1 / math.sqrt(2)], [- 1 / math.sqrt(2), 0, 1 / math.sqrt(2)]]}

    # 'left': [-0.3252, -4.4345, 1.2298, 5.0867, -2.2970, -2.7121]
    idle_joint = {'left' : [-0.3812902609454554, -4.0254041157164515, 1.2768119017230433, 4.692070646876953, -2.2540810743915003, -2.6285222212420862],
                    'right' : [-2.3372, -4.3113, 0.9498, -0.7260, -1.0605, 1.3825]}

    idle_joint_side_view = {'left' : [0.2415, -2.7355, 0.8113, 3.6897, -3.8907, -1.2848]}
    idle_joint_side_view2 = {'left' : [0.5178, -3.2067, 1.4637, 3.7682, -3.9780, -0.9182]}
    idle_joint_side_view3 = {'left' : [0.5051450729370117, -3.0968877277769984, 1.5314925352679651, 3.579940004939697, -4.045039002095358, -0.9176719824420374]}

    integrade_joint_side = {'left' : [0.3664, -3.1741, 2.3328, 3.6704, 1.8027, -0.5058]}

    # Camera to Wrist transformation matrix
    L_W_C = {'left' : np.array([[-1, 0, 0, 0.05], [0, -1, 0, 0.085], [0, 0, 1, 0.03], [0, 0, 0, 1]]),
            'right' : np.array([[-1, 0, 0, 0.05], [0, -1, 0, 0.085], [0, 0, 1, 0.03], [0, 0, 0, 1]])}

    L_B_D = {'left' : np.array([[0, 1, 0, 0.25], [1 / math.sqrt(2), 0, - 1 / math.sqrt(2), 0.2121], [- 1 / math.sqrt(2), 0, - 1 / math.sqrt(2), 0.6364], [0, 0, 0, 1]]),
             'right' : np.array([[0, -1, 0, -0.25], [- 1 / math.sqrt(2), 0, - 1 / math.sqrt(2), 1.0323], [1 / math.sqrt(2), 0, - 1 / math.sqrt(2), -0.1838], [0, 0, 0, 1]])}

    # appropriate wrist position relative to object pivot(in object coordinate system) for gripping
    v_O_dict = {
        'carrot' : np.array([0.185, 0, -0.035, 1]),
        'onion' : np.array([0.183, 0, -0.035, 1]),
        'pan_handle_handle' : np.array([0, -0.17, -0.03, 1]),
        'rice_bowl' : np.array([-0.19, -0.09, 0, 1]),
        'oil_bowl' : np.array([-0.21, -0.03, 0, 1]),
        'salt_bowl' : np.array([-0.203, 0, 0, 1]),
        'board_handle' : np.array([-0.14, 0, 0, 1]),
        'knife_handle' : np.array([-0.18, 0.02, 0, 1]),
        'paddle_handle' : np.array([-0.0, 0.18, 0, 1]),
        'switch' : np.array([-0.157, 0, 0, 1])
    }
    # appropriate wrist orientation relative to object orientation for gripping
    r_O_W_dict = {
        'carrot': np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]]),
        'onion' : np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]]),
        'pan_handle_handle': np.array([[0, -1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]),
        'rice_bowl' : np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]),
        'oil_bowl' : np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]),
        'salt_bowl' : np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]),
        'board_handle' : np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]]),
        'knife_handle' : np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]]),
        'paddle_handle' : np.array([[-1, 0, 0, 0], [0, 0, -1, 0], [0, -1, 0, 0], [0, 0, 0, 1]]),
        'switch' : np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
    }

    def __init__(self):
        # robot
        rospy.init_node('dope_reader')
        self.robot = {}
        self.dope_reader = {'left' : DopeReader('L'), 'right' : DopeReader('R')}
        rospy.sleep(1)

    # Initialize robot, and go to idle
    def ready(self, side):
        if side == 'left':
            self.robot['left'] = urx.Robot("192.168.1.66")
            self.gripper = Robotiq_Two_Finger_Gripper(self.robot['left'])
        if side == 'right':
            self.robot['right'] = urx.Robot("192.168.1.109")

        print('Robot tcp : ', self.robot[side].getl())
        print('Robot joints : ', self.robot[side].getj())

    # Go to idle position.
    # If side_view is True, robot will go to side view position. Otherwise it will go to top view position.
    # start_closed or start_opened could be given in order to prevent intermediate collision.
    def idle(self, side, side_view=False, start_closed=False, start_opened=False):
        if side == 'left':
            if start_closed:
                self.gripper.close_gripper()
            elif start_opened:
                self.gripper.open_gripper()

        if side_view:
            self.robot[side].movej(self.idle_joint_side_view3[side], 0.1, 0.1)
        else:
            self.robot[side].movej(self.idle_joint[side], 0.1, 0.1)

        if side == 'left':
            self.gripper.open_gripper()

    def desk_to_base(self, side, pose, translation=1):
        pose = [pose[0], pose[1], pose[2], translation]
        return np.matmul(self.L_B_D[side], pose)[:3]

    # This function will transform the given 6d pos(v_O, r_O_W)
    #  from object coordinates into base coordinates
    def get_target_6d_pos(self, side, obj, v_O, r_O_W):
        scale = 0.05
        if obj == 'board_handle' or obj == 'knife_handle' or obj == 'paddle_handle':
            scale = 1
        if obj == 'switch':
            scale = 1.18
        L_C_O = utils.dope_to_affine(self.dope_reader[side].get_obj_pos(obj), scale=scale)
        print(L_C_O)
        p_C_O = L_C_O[:, 3]
        if obj == 'carrot' or obj == 'onion':
            L_C_O = self.align_axis(R.from_dcm(utils.to_33(L_C_O)), 0, [0, 0.5, -math.sqrt(3) / 2]).as_dcm()
            L_C_O = utils.to_44(L_C_O, p_C_O)
        if obj == 'switch' or 'bowl' in obj:
            L_C_O = self.align_axis(R.from_dcm(utils.to_33(L_C_O)), 0, [0, 0, 1]).as_dcm()
            L_C_O = utils.to_44(L_C_O, p_C_O)

        L_B_W = utils.tcp_to_affine(self.robot[side].getl())
        print('L_C_O : \n', L_C_O)
        print('L_B_W : \n', L_B_W)
        target_position = np.matmul(
            L_B_W, np.matmul(
                self.L_W_C[side], np.matmul(
                    L_C_O, v_O
                )
            )
        )[:3]

        target_orientation = R.from_dcm(utils.to_33(np.matmul(
            L_B_W, np.matmul(
                # rotation offset
                utils.to_44(R.from_rotvec([0, -0, 0]).as_dcm()), np.matmul(
                    self.L_W_C[side], np.matmul(
                        L_C_O, r_O_W
                    )
                )
            )
        ))).as_rotvec()

        return np.concatenate([target_position, target_orientation])

    def movel_with_position(self, side, pos, vel, acc, relative):
        if relative:
            current_pos = self.robot[side].getl()[:3]
            pos = np.array(current_pos) + pos
            self.robot[side].movel(np.concatenate([pos, self.robot[side].getl()[3:]]), vel, acc, False)
        else:
            self.robot[side].movel(np.concatenate([pos, self.robot[side].getl()[3:]]), vel, acc, False)

    def align_axis(self, r, from_axis, to_axis):
        z = R.as_dcm(r)[:, from_axis]
        cross_product = np.cross(z, to_axis)
        a = R.from_rotvec(cross_product) * r
        return a

    # Reach the given object.
    # The desired wrist 6d pos(in object coordinates) are given as a constant for each object,
    # and then transformed into base coordinates with self.get_target_6d_pos.
    def reach(self, side, obj):
        target_6d_pos = self.get_target_6d_pos(side, obj, self.v_O_dict[obj], self.r_O_W_dict[obj])
        print('target 6d pos : ', target_6d_pos)
        self.robot[side].movel(target_6d_pos, 0.1, 0.1, relative=False)

    def close(self):
        for side in self.robot:
            self.robot[side].close()

if __name__ == '__main__':
    agent = Agent()
    agent.ready('left')
    agent.idle('left', side_view=False, start_closed=True)
    time.sleep(1)
    agent.reach('left', 'rice_bowl')
    agent.gripper.close_gripper()
    agent.close()
