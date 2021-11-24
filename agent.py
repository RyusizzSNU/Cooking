import rospy
import urx
import time
import math
import os

#from typing import Dict, List
from Dependencies.urx_custom.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from scipy.spatial.transform import Rotation as R
import numpy as np
import utils
from dope_reader import DopeReader
from AllegroHand import AllegroHandController
from HYHand import HYHandController
from Module.Take import Take
from Module.Pour import Pour
from Module.Put import Put
from Module.Hold import Hold
from Module.Sprinkle import Sprinkle
from Module.Cut import Cut
from Module.Mix import Mix
from Module.Turn import Turn
import json

from collections import OrderedDict

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
    objects = ['carrot', 'onion', 'oil_bowl', 'rice_bowl', 'salt_bowl',
               'board_handle', 'knife_handle', 'paddle_handle', 'pan_handle_handle', 'switch', 'spam', 'kettle_handle']
    def __init__(self):
        # robot
        np.set_printoptions(precision=4)
        rospy.init_node('dope_reader')
        self.robot = {}
        self.dope_reader = {'left' : DopeReader('L'), 'right' : DopeReader('R')}
        self.read_poses()
        self.read_transforms()
        self.read_trajectories()

        for obj in self.objects:
            if obj not in self.dope_scale_dict:
                self.dope_scale_dict[obj] = self.dope_scale_dict['others']

        rospy.sleep(1)

    def read_poses(self):
        self.poses = {}
        self.hand_poses = OrderedDict()
        with open('./poses.csv', 'r') as f:
            lines = f.readlines()
            for line in lines[1:]:
                tokens = line.split(',')
                side, name = tokens[0], tokens[1]
                if side not in self.poses:
                    self.poses[side] = OrderedDict()
                joint = []
                for i in range(2, 8):
                    joint.append(float(tokens[i]))
                self.poses[side][name] = joint
        with open('./hand_poses.csv', 'r') as f:
            lines = f.readlines()
            for line in lines[1:]:
                tokens = line.split(',')
                name = tokens[0]
                joint = []
                for i in range(1, 17):
                    joint.append(float(tokens[i]))
                self.hand_poses[name] = joint

    def read_transforms(self):
        self.L_W_C = {}
        self.L_B_D = {}
        self.L_O_W_dict = {}
        self.dope_scale_dict = {}

        with open('./transforms.csv', 'r') as f:
            lines = f.readlines()
            for i in range(len(lines)):
                line = lines[i].replace('\n', '')
                i += 1
                if line == '':
                    continue
                tokens = line.split(',')

                # L_W_C : Camera to Wrist transformation matrix
                # L_B_D : Robot Base to Desk transformation matrix
                # L_O_W : Appropriate wrist position and orientation(in affine matrix form) relative to object for gripping. List for sequential movement

                if tokens[0] == 'L_W_C' or tokens[0] == 'L_B_D' or tokens[0] == 'L_O_W':
                    n = int(tokens[2]) if len(tokens) >= 3 else None
                    if tokens[0] == 'L_W_C':
                        dic = self.L_W_C
                    elif tokens[0] == 'L_B_D':
                        dic = self.L_B_D
                    else:
                        dic = self.L_O_W_dict

                    if n is None:
                        dic[tokens[1]] = np.array([[float(x) for x in lines[i + k].split(',')] for k in range(4)])
                        i += 4
                    else:
                        dic[tokens[1]] = [np.array([[float(x) for x in lines[i + 4*j + k].split(',')] for k in range(4)]) for j in range(n)]
                        i += 4 * n
                elif tokens[0] == 'scale':
                    self.dope_scale_dict[tokens[1]] = float(tokens[2])

    def read_trajectories(self):
        with open('./trajectories.json', 'r') as f:
            self.obj_trajectories = json.loads(f.read())

    def get_instructions(self):
        if not hasattr(self, 'instructions'):
            self.instructions = [
                Take(),
                Pour(),
                Put(),
                Sprinkle(),
                Hold(),
                Cut(),
                Mix(),
                Turn()
            ]

        return self.instructions

    def execute_instruction(self, instruction):
        verb, noun = instruction[0], instruction[1]
        print(verb + " " + noun)
        if noun == 'kettle' or noun == 'knife' or noun == 'ladle':
            noun = noun + '_handle'
        if noun == 'pepper':
            noun = 'pepper_bowl'
        if noun == 'sausage':
            noun = 'spam'

        if noun == 'sauce':
            noun = 'oil_bowl'

        for inst in self.get_instructions():
            if verb.lower() == inst.display_name.lower():
                inst_ = inst.copy()
                inst_.function(self, noun)
                break

    def state_description(self):
        return 'Left : \n' + '\ttcp : %s\n'%np.array(self.robot['left'].getl()) + '\tjoints : %s\n'%np.array(self.robot['left'].getj()) + 'Right : \n' + '\ttcp : %s\n'%np.array(self.robot['right'].getl()) + '\tjoints : %s\n'%np.array(self.robot['right'].getj()) + '\thand target : %s\n'%self.hand.target + '\thand joint : %s\n'%self.hand.joint

    # Initialize robot, and go to idle
    def ready(self, side):
        if side == 'left':
            self.robot['left'] = urx.Robot("192.168.1.66")
            self.gripper = Robotiq_Two_Finger_Gripper(self.robot['left'], speed=200, force=20)
            self.gripper.force = 100
            # self.gripper = Robotiq_Two_Finger_Gripper(self.robot['left'], payload=0.25, speed=50, force=10)
        if side == 'right':
            self.robot['right'] = urx.Robot("192.168.1.109")
            self.hand = HYHandController()
            self.hand.connect()

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

        self.dope_reader[side].reset()
        if side == 'right' and view == 'top3':
            self.hand_action('grab')
        self.movej(side, self.poses[side][view], acc=acc, vel=vel, wait=wait)
        if side == 'right' and view == 'top3':
            self.hand_action('default')

        if side == 'left':
            self.open_gripper()
        # elif side == 'right' and view == 'top2':
        #     self.ready_hand()

    def put_paddle_position(self, acc=0.2, vel=0.2, wait=True):
        self.movej('right', self.poses['right']['put_paddle'], acc=acc, vel=vel, wait=wait)

    def put_knife_position(self, acc=0.2, vel=0.2, wait=True):
        self.movej('right', self.poses['right']['put_knife'], acc=acc, vel=vel, wait=wait)

    def memorize_object_trajectory(self, obj, trajectory):
        if not hasattr(self, 'obj_trajectories'):
            self.obj_trajectories = {}
        self.obj_trajectories[obj] = trajectory
        with open('trajectories.json', 'w') as f:
            f.write(json.dumps(self.obj_trajectories))

    # Reach the given object.
    # The desired wrist 6d pos(in object coordinates) are given as a constant for each object,
    # and then transformed into base coordinates with self.get_target_6d_pos.
    # For align_axis_from and align_axis_to, see the comments at Agent.align_axis function.
    def reach(self, side, obj, acc=0.1, vel=0.1, align_axis_from=None, align_axis_to=None):
        side_str = 'L' if side == 'left' else 'R'
        rospy.set_param('/dope/activities/%s/%s'%(side_str, obj), True)
        time.sleep(1)
        L_C_O = utils.dope_to_affine(self.dope_reader[side].get_obj_pos(obj), scale=self.dope_scale_dict[obj])
        L_B_W = utils.tcp_to_affine(self.robot[side].getl())
        print 'dope L_C_O:\n', L_C_O

        if align_axis_from is not None and align_axis_to is not None:
            L_C_O = self.align_axis(L_C_O, align_axis_from, align_axis_to)
        print 'Axis aligned L_C_O:\n', L_C_O
        print 'L_B_W:\n', L_B_W

        if obj == 'pan_handle_handle':
            self.movej(side, self.poses[side]['stopover'])

        for L_O_W in self.L_O_W_dict[obj]:
            target_6d_pos = self.get_target_6d_pos(side, L_B_W, L_C_O, L_O_W)
            print 'target 6d pos:', target_6d_pos
            self.robot[side].movel(target_6d_pos, acc=acc, vel=vel, relative=False)

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
    def movej(self, side, joints, acc=0.2, vel=0.2, wait=True, relative=False):
        self.robot[side].movej(joints, acc=acc, vel=vel, wait=wait, relative=relative)

    # Wrapper for urx.Robot.movel, handling also 3-dimensional input(as position)
    def movel(self, side, tcp, acc=0.2, vel=0.2, wait=True, relative=False):
        if len(tcp) == 3:
            if relative:
                tcp = np.array(self.robot[side].getl()[:3]) + tcp
            self.robot[side].movel(np.concatenate([tcp, self.robot[side].getl()[3:]]), acc=acc, vel=vel, wait=wait, relative=False)
        elif len(tcp) == 6:
            self.robot[side].movel(tcp, acc=acc, vel=vel, wait=wait, relative=relative)
        else:
            assert False, 'length of tcp input must be 3 or 6'

    def movels(self, side, pos_list, acc=0.2, vel=0.2, wait=True):
        self.robot[side].movels(pos_list, acc=acc, vel=vel, wait=wait)

    # Wrapper for gripper.open_gripper
    def open_gripper(self):
        self.gripper.open_gripper()
        time.sleep(1)

    # Wrapper for gripper.close_gripper
    def close_gripper(self):
        self.gripper.close_gripper()
        time.sleep(1)

    # Wrapper for gripper.gripper_action. 0 for open and 1 for close.
    def gripper_action(self, val):
        self.gripper.gripper_action(int(val * 255))
        time.sleep(1)

    def move_hand(self, val, relative=True):
        self.hand.move_joint(val, relative)

    def hand_action(self, name):
        self.move_hand(self.hand_poses[name], relative=False)

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
                current_tcp_D = utils.affine_to_tcp(self.base_to_desk(side, affine=utils.tcp_to_affine(self.getl(side))))
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
    agent.idle('left', view='top', start_closed=True)
    # agent.ready('right')
    # agent.idle('right', view='board', start_closed=True)

    agent.close()
