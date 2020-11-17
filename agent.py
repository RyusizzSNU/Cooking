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
    D : desk 
'''
class Agent():
    idle_joint = {'left' : [-0.3813, -4.0254, 1.2768, 4.6921, -2.2541, -2.6285],
                    'right' : [-2.3372, -4.3113, 0.9498, -0.7260, -1.0605, 1.3825]}

    idle_joint_side_view = {'left' : [0.5051, -3.0969, 1.5315, 3.5800, -4.0450, -0.9177]}

    # Camera to Wrist transformation matrix
    L_W_C = {'left' : np.array([[-1, 0, 0, 0.05], [0, -1, 0, 0.085], [0, 0, 1, 0.03], [0, 0, 0, 1]]),
            'right' : np.array([[-1, 0, 0, 0.05], [0, -1, 0, 0.085], [0, 0, 1, 0.03], [0, 0, 0, 1]])}

    L_B_D = {'left' : np.array([[0, 1, 0, 0.25], [1 / math.sqrt(2), 0, - 1 / math.sqrt(2), 0.2121], [- 1 / math.sqrt(2), 0, - 1 / math.sqrt(2), 0.6364], [0, 0, 0, 1]]),
             'right' : np.array([[0, -1, 0, -0.25], [- 1 / math.sqrt(2), 0, - 1 / math.sqrt(2), 1.0323], [1 / math.sqrt(2), 0, - 1 / math.sqrt(2), -0.1838], [0, 0, 0, 1]])}

    # Appropriate wrist position relative to object pivot(in object coordinate system) for gripping
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
    # Appropriate wrist orientation relative to object orientation for gripping
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
            self.robot[side].movej(self.idle_joint_side_view[side], 0.1, 0.1)
        else:
            self.robot[side].movej(self.idle_joint[side], 0.1, 0.1)

        if side == 'left':
            self.gripper.open_gripper()

    # This function will transform the given 6d pos(v_O, r_O_W)
    # from object coordinates into base coordinates
    def get_target_6d_pos(self, side, obj, v_O, r_O_W):
        scale = 0.05
        if obj == 'board_handle' or obj == 'knife_handle' or obj == 'paddle_handle':
            scale = 1
        if obj == 'switch':
            scale = 1.18
        L_C_O = utils.dope_to_affine(self.dope_reader[side].get_obj_pos(obj), scale=scale)
        print('dope L_C_O : \n', L_C_O)
        p_C_O = L_C_O[:, 3]
        if obj == 'carrot' or obj == 'onion':
            L_C_O = self.align_axis(R.from_dcm(utils.to_33(L_C_O)), 0, [0, 0.5, -math.sqrt(3) / 2]).as_dcm()
            L_C_O = utils.to_44(L_C_O, p_C_O)
        if obj == 'switch' or 'bowl' in obj:
            L_C_O = self.align_axis(R.from_dcm(utils.to_33(L_C_O)), 0, [0, 0, 1]).as_dcm()
            L_C_O = utils.to_44(L_C_O, p_C_O)

        L_B_W = utils.tcp_to_affine(self.robot[side].getl())
        print('Axis aligned L_C_O : \n', L_C_O)
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
                self.L_W_C[side], np.matmul(
                    L_C_O, r_O_W
                )
            )
        ))).as_rotvec()

        return np.concatenate([target_position, target_orientation])

    # Transform desk coordinates into base coordinates
    def desk_to_base(self, side, position=None, direction=None, scipy_R=None, dcm=None, affine=None):
        return utils.transform(self.L_B_D[side], position, direction, scipy_R, dcm, affine)

    # Transform base coordinates into desk coordinates
    def base_to_desk(self, side, position=None, direction=None, scipy_R=None, dcm=None, affine=None):
        return utils.transform(np.linalg.inv(self.L_B_D[side]), position, direction, scipy_R, dcm, affine)

    # Given r : scipy.spatial.transform.Rotation, from_axis : int, to_axis : vector,
    # this function will align r's from_axis'th axis to to_axis, in order to compensate DOPE orientation error.
    def align_axis(self, r, from_axis, to_axis):
        z = R.as_dcm(r)[:, from_axis]
        cross_product = np.cross(z, to_axis)
        a = R.from_rotvec(cross_product) * r
        return a

    # Reach the given object.
    # The desired wrist 6d pos(in object coordinates) are given as a constant for each object,
    # and then transformed into base coordinates with self.get_target_6d_pos.
    def reach(self, side, obj):
        time.sleep(1)
        target_6d_pos = self.get_target_6d_pos(side, obj, self.v_O_dict[obj], self.r_O_W_dict[obj])
        print('target 6d pos : ', target_6d_pos)
        self.robot[side].movel(target_6d_pos, 0.1, 0.1, relative=False)

    ''' Wrappers '''
    # Wrapper for urx.Robot.getj
    def getj(self, side):
        return self.robot[side].getj()

    # Wrapper for urx.Robot.getl
    def getl(self, side):
        return self.robot[side].getl()

    # Wrapper for urx.Robot.movej
    def movej(self, side, joints, acc=0.1, vel=0.1, wait=True, relative=False):
        self.robot[side].movej(joints, acc=acc, vel=vel, wait=wait, relative=relative)

    # Wrapper for urx.Robot.movel, handling also 3-dimensional input(as position)
    def movel(self, side, tcp, acc=0.1, vel=0.1, wait=True, relative=False):
        if len(tcp) == 3:
            if relative:
                tcp = np.array(self.robot[side].getl()[:3]) + tcp
            self.robot[side].movel(np.concatenate([tcp, self.robot[side].getl()[3:]]), acc=acc, vel=vel, wait=wait, relative=False)
        elif len(tcp) == 6:
            self.robot[side].movel(tcp, acc=acc, vel=vel, wait=wait, relative=relative)
        else:
            assert False, 'length of tcp input must be 3 or 6'

    # Wrapper for gripper.open_gripper
    def open_gripper(self):
        self.gripper.open_gripper()

    # Wrapper for gripper.close_gripper
    def close_gripper(self):
        self.gripper.close_gripper()

    # Wrapper for gripper.gripper_action. 0 for open and 255 for close.
    def gripper_action(self, val):
        self.gripper.gripper_action(val)

    # Move in Desk Coordinate system(as if the robot base is at desk corner, axis aligned)
    # Also handles 3-dimensional input(as position)
    def moveD(self, side, tcp, acc=0.1, vel=0.1, wait=True, relative=False):
        if len(tcp) == 3:
            if relative:
                dir = self.desk_to_base(side, direction=tcp)
                self.movel(side, dir, acc=acc, vel=vel, wait=wait, relative=True)
            else:
                pos = self.desk_to_base(side, position=tcp)
                self.movel(side, pos, acc=acc, vel=vel, wait=wait, relative=False)
        elif len(tcp) == 6:
            if relative:
                current_tcp_D = utils.affine_to_tcp(self.base_to_desk(side, affine=utils.tcp_to_affine(tcp)))
                tcp_D = current_tcp_D + tcp
                tcp_B = utils.affine_to_tcp(self.desk_to_base(side, affine=utils.tcp_to_affine(tcp_D)))
                self.movel(side, tcp_B, acc=acc, vel=vel, wait=wait, relative=False)
            else:
                tcp_B = utils.affine_to_tcp(self.desk_to_base(side, affine=utils.tcp_to_affine(tcp)))
                self.movel(side, tcp_B, acc=acc, vel=vel, wait=wait, relative=False)
        else:
            assert False, 'length of tcp input must be 3 or 6'

    # Wrapper for urx.Robot.close
    def close(self):
        for side in self.robot:
            self.robot[side].close()

if __name__ == '__main__':
    agent = Agent()
    agent.ready('left')
    agent.idle('left', side_view=False, start_closed=True)
    time.sleep(1)
    agent.reach('left', 'rice_bowl')
    agent.close_gripper()
    agent.close()
