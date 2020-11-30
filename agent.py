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
from hand import AllegroHandController

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
    idle_joint = {
        'left' : {
            'top' : [-0.3813, -4.0254, 1.2768, 4.6921, -2.2541, -2.6285],
            'lateral' : [0.3382, -3.028,   1.5124,  3.4266, -3.9868, -1.1204],
            #'lateral' : [0.2014, -3.1329, 1.6326, 3.2768, -3.9478, -1.2993],
            'top2' : [0.0577, -4.5743, 1.4047, 4.7180, -2.3311, -3.2124]
        },
        'right' : {
            'top' : [-2.3372, -4.3113, 0.9498, -0.7260, -1.0605, 1.3825],
            'top2' : [-2.0728, -4.4702,  1.1529, -1.1107, -1.3792,  1.13  ],
            #'top2' : [-2.0309, -3.985,   0.7016, -0.698,  -1.2536,  1.1223],
            'board' : [-2.2879, -4.511,   1.3044, -0.8525, -1.0675, 1.2349]
        }
    }

    # Camera to Wrist transformation matrix
    L_W_C = {'left' : np.array([[-1, 0, 0, 0.05], [0, -1, 0, 0.085], [0, 0, 1, 0.03], [0, 0, 0, 1]]),
            'right' : np.array([[-1, 0, 0, 0.05], [0, -1, 0, 0.085], [0, 0, 1, 0.03], [0, 0, 0, 1]])}

    L_B_D = {'left' : np.array([[0, 1, 0, 0.25], [1 / math.sqrt(2), 0, - 1 / math.sqrt(2), 0.2121], [- 1 / math.sqrt(2), 0, - 1 / math.sqrt(2), 0.6364], [0, 0, 0, 1]]),
             'right' : np.array([[0, -1, 0, -0.25], [- 1 / math.sqrt(2), 0, - 1 / math.sqrt(2), 1.0323], [1 / math.sqrt(2), 0, - 1 / math.sqrt(2), -0.1838], [0, 0, 0, 1]])}

    objects = ['carrot', 'onion', 'oil_bowl', 'rice_bowl', 'salt_bowl',
               'board_handle', 'knife_handle', 'paddle_handle', 'pan_handle_handle', 'switch', 'spam']

    # Appropriate wrist position and orientation(in affine matrix form) relative to object for gripping
    L_O_W_dict = {
        'carrot': [np.array([[0, 0, -1, 0.17], [-1, 0, 0, 0], [0, 1, 0, -0.035], [0, 0, 0, 1]])],
        'onion' : [np.array([[0, 0, -1, 0.17], [-1, 0, 0, 0], [0, 1, 0, -0.0], [0, 0, 0, 1]])],
        'pan_handle_handle': [np.array([[0, -1, 0, 0], [0, 0, 1, -0.17], [-1, 0, 0, -0.03], [0, 0, 0, 1]])],
        'rice_bowl' : [np.array([[0, 0, 1, -0.19], [0, 1, 0, -0.09], [-1, 0, 0, 0], [0, 0, 0, 1]])],
        'oil_bowl' : [np.array([[0, 0, 1, -0.21], [0, 1, 0, -0.03], [-1, 0, 0, 0], [0, 0, 0, 1]])],
        'salt_bowl' : [np.array([[0, 0, 1, -0.203], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])],
        'board_handle' : [np.array([[0, 0, 1, -0.16], [0, -1, 0, 0.02], [1, 0, 0, 0], [0, 0, 0, 1]])],
        'knife_handle' :
            [np.array([[0, 0, 1, -0.17], [-1, 0, 0, 0.09], [0, -1, 0, -0.16], [0, 0, 0, 1]]),
            np.array([[0, 0, 1, -0.17], [-1, 0, 0, 0.07], [0, -1, 0, -0.06], [0, 0, 0, 1]]),
            np.array([[0, 0, 1, -0.135], [-1, 0, 0, 0.07], [0, -1, 0, -0.06], [0, 0, 0, 1]])],
        'paddle_handle' :
            [np.array([[1, 0, 0, -0.1], [0, 0, -1, 0.16], [0, 1, 0, 0.1], [0, 0, 0, 1]]),
            np.array([[1, 0, 0, -0.03], [0, 0, -1, 0.16], [0, 1, 0, 0.1], [0, 0, 0, 1]]),
            np.array([[1, 0, 0, -0.02], [0, 0, -1, 0.08], [0, 1, 0, 0.02], [0, 0, 0, 1]])],
        'switch' : [np.array([[0, 0, 1, -0.157], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]])],
        'spam' : [np.array([[1, 0, 0, 0], [0, 0, -1, 0.15], [0, 1, 0, -0.03], [0, 0, 0, 1]])]
    }

    dope_scale_dict = {
        'board_handle' : 1,
        'paddle_handle' : 1,
        'knife_handle' : 1.25,
        'spam' : 1.18, # 1.12
        'switch' : 1.18,
        'oil_bowl' : 0.04878,
        'rice_bowl' : 0.04968,
        'carrot' : 0.04953,
        'onion' : 0.04930,
        'others' : 0.05
    }

    def __init__(self):
        # robot
        np.set_printoptions(precision=4)
        rospy.init_node('dope_reader')
        self.robot = {}
        self.dope_reader = {'left' : DopeReader('L'), 'right' : DopeReader('R')}

        for obj in self.objects:
            if obj not in self.dope_scale_dict:
                self.dope_scale_dict[obj] = self.dope_scale_dict['others']

        rospy.sleep(1)

    # Initialize robot, and go to idle
    def ready(self, side):
        if side == 'left':
            self.robot['left'] = urx.Robot("192.168.1.66")
            self.gripper = Robotiq_Two_Finger_Gripper(self.robot['left'])
            self.gripper.force = 100
            # self.gripper = Robotiq_Two_Finger_Gripper(self.robot['left'], payload=0.25, speed=50, force=10)
        if side == 'right':
            self.robot['right'] = urx.Robot("192.168.1.109")
            self.hand = AllegroHandController()

        print 'Robot tcp:', np.array(self.robot[side].getl())
        print 'Robot joints:', np.array(self.robot[side].getj())

    # side: str(either 'left' or 'right', view: str('top', 'lateral', ...), start_closed: bool, start_opened: bool
    # Send robot to an idle position, with given view.
    # start_closed or start_opened could be given in order to prevent intermediate collision.
    def idle(self, side, view='top', start_closed=False, start_opened=False, acc=0.25, vel=0.25, wait=True):
        if side == 'left':
            if start_closed:
                self.close_gripper()
            elif start_opened:
                self.open_gripper()

        self.movej(side, self.idle_joint[side][view], acc=acc, vel=vel, wait=wait)

        if side == 'left':
            self.open_gripper()
        elif side == 'right':
            self.hand.lib_cmd('home')
            self.hand.movej(joint_num=12, val=1.5)
            self.hand.movej(joint_num=14, val=0)
            self.hand.movej(joint_num=15, val=0)

    # Reach the given object.
    # The desired wrist 6d pos(in object coordinates) are given as a constant for each object,
    # and then transformed into base coordinates with self.get_target_6d_pos.
    # For align_axis_from and align_axis_to, see the comments at Agent.align_axis function.
    def reach(self, side, obj, align_axis_from=None, align_axis_to=None):
        side_str = 'L'
        if side == 'right':
            side_str = 'R'
        rospy.set_param('/dope/activities/%s/%s'%(side_str, obj), True)
        time.sleep(1)
        L_C_O = utils.dope_to_affine(self.dope_reader[side].get_obj_pos(obj), scale=self.dope_scale_dict[obj])
        L_B_W = utils.tcp_to_affine(self.robot[side].getl())
        print 'dope L_C_O:\n', L_C_O

        if align_axis_from is not None and align_axis_to is not None:
            L_C_O = self.align_axis(L_C_O, align_axis_from, align_axis_to)
        print 'Axis aligned L_C_O:\n', L_C_O
        print 'L_B_W:\n', L_B_W

        for L_O_W in self.L_O_W_dict[obj]:
            target_6d_pos = self.get_target_6d_pos(side, L_B_W, L_C_O, L_O_W)
            print 'target 6d pos:', target_6d_pos
            self.robot[side].movel(target_6d_pos, 0.1, 0.1, relative=False)

        rospy.set_param('/dope/activities/%s/%s' % (side_str, obj), False)

    # This function will transform the given affine matrix(L_O_W)
    # from object coordinates into base coordinates, given object to camera transformation(L_C_O)
    def get_target_6d_pos(self, side, L_B_W, L_C_O, L_O_W):
        target_L_B_W = np.matmul(
            L_B_W, np.matmul(
                self.L_W_C[side], np.matmul(
                    L_C_O, L_O_W
                )
            )
        )
        return utils.affine_to_tcp(target_L_B_W)

    # Transform desk coordinates into base coordinates
    def desk_to_base(self, side, position=None, direction=None, scipy_R=None, dcm=None, affine=None):
        return utils.transform(self.L_B_D[side], position, direction, scipy_R, dcm, affine)

    # Transform base coordinates into desk coordinates
    def base_to_desk(self, side, position=None, direction=None, scipy_R=None, dcm=None, affine=None):
        return utils.transform(np.linalg.inv(self.L_B_D[side]), position, direction, scipy_R, dcm, affine)

    # Given r : scipy.spatial.transform.Rotation, from_axis : int, to_axis : vector,
    # this function will align r's from_axis'th axis to to_axis, in order to compensate DOPE orientation error.
    def align_axis(self, affine, from_axis, to_axis):
        t, r = affine[:3, 3], affine[:3, :3]

        z = r[:, from_axis]
        cross_product = np.cross(z, to_axis)
        new_r = np.matmul(R.from_rotvec(cross_product).as_dcm(), r)

        return utils.to_44(new_r, t)

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
        time.sleep(0.2)

    # Wrapper for gripper.close_gripper
    def close_gripper(self):
        self.gripper.close_gripper()
        time.sleep(0.2)

    # Wrapper for gripper.gripper_action. 0 for open and 255 for close.
    def gripper_action(self, val):
        self.gripper.gripper_action(val)
        time.sleep(0.2)

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
    #agent.ready('left')
    #agent.idle('left', view='top', start_closed=True)

    #agent.reach('left', 'onion')
    #agent.gripper_action(int(0.5 * 255))

    agent.ready('right')
    #agent.idle('right', view='board', start_closed=True)

    #agent.reach('right', 'knife_handle')
    #agent.hand.grab()

    #agent.close_gripper()
    agent.close()
