from Instruction import Instruction, Param
from Take import Take
import utils
import math
import numpy as np
import time

class Put(Instruction):
    def __init__(self):
        self.display_name = 'Put'
        self.function = self.put
        self.cursor = 0
        self.params = [Param('Object', {
                    'Onion' : 'onion',
                    'Kettle Handle' : 'kettle_handle',
                    'Knife Handle' : 'knife_handle',
                    'Pepper Bowl': 'pepper_bowl',
                    'Ladle Handle': 'ladle_handle',
                    'Oil Bowl' : 'oil_bowl',
                    'Spam' : 'spam',
                    'Noodle' : 'noodle',
                    'Pot' : 'pot'
            })
        ]

    def put_on_board(self, agent, obj):
        rot1 = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
        rot2 = [[0, math.sqrt(3) / 2, 0.5], [1, 0, 0], [0, 0.5, -math.sqrt(3) / 2]]

        # Move above the board
        tcp = utils.affine_to_tcp(dcm=rot1, t=[0.58, 0.23, 0.61])
        agent.moveD('left', tcp)

        # Move downwards right onto the board
        agent.moveD('left', [0, 0, -0.20], vel=0.2, relative=True)

        agent.open_gripper()
        time.sleep(1)
        pos = agent.base_to_desk('left', agent.getl('left')[:3])

        new_tcp = utils.affine_to_tcp(dcm=rot2, t=pos + np.array([-0.08, 0, -0.02]))
        agent.moveD('left', new_tcp)
        agent.gripper_action(0.6)

    def put(self, agent, obj):
        if obj == 'spam' or obj == 'onion' or obj == 'spring_onion' :
            Take().take(agent, obj)
            self.put_on_board(agent, obj)

        elif obj == 'pot':
            agent.open_gripper()
            agent.moveD('left', [0, 0, 0.3], relative=True)

            agent.idle('left', view='lateral', start_closed=True)
            # Reach and grip board handle
            agent.reach('left', 'board_handle', align_axis_from=0, align_axis_to=[0, 0, 1])
            agent.close_gripper()
            j1 = agent.getj('left')
            time.sleep(1)

            a = math.radians(30)
            b = math.radians(45)

            a2 = math.radians(5)
            rot0 = np.array([[math.cos(b), -math.sin(b), 0],
                             [-math.sin(b) * math.cos(a), -math.cos(a) * math.cos(b), -math.sin(a)],
                             [math.sin(b) * math.sin(a), math.cos(b) * math.sin(a), -math.cos(a)]])

            #rot0 = np.array([[0, -math.cos(a2), -math.sin(a2)], [0, -math.sin(a2), math.cos(a2)], [-1, 0, 0]])
            #rot0 = rot0.T

            agent.moveD('right', utils.affine_to_tcp(dcm=rot0, t=[0.90, 0.36, 0.7]))
            #agent.moveD('right', utils.affine_to_tcp(dcm=rot0, t=[0.77, 0.38, 0.567]))
            #agent.moveD('right', [0.2, 0, 0.2], relative=True)

            # Raise
            agent.moveD('left', [0, 0, 0.1], relative=True)
            j2 = agent.getj('left')

            # Move above the pan
            agent.moveD('left', [0.32, 0.35, 0.37])

            # Rotate to pour and rotate back
            agent.movej('left', [0, 0, 0, 0, 0, -a], relative=True, wait=False)

            d = 0.12
            '''pos = np.array([0.8179, 0.3306, 0.529]) + np.array([0, 0, 0.1])

            rot1 = np.array([[-1, 0, 0], [0, math.sin(a), math.cos(a)], [0, math.cos(a), -math.sin(a)]])
            rot2 = np.array([[math.cos(a2), -math.sin(a2), 0], [math.sin(a2), math.cos(a2), 0], [0, 0, 1]])

            rot = np.matmul(rot2, rot1).T

            agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=pos))'''
            agent.moveD('right', utils.affine_to_tcp(dcm=rot0, t=[0.82, 0.30, 0.70]))
            for i in range(3):
                agent.moveD('right', [0, 0, -0.1], relative=True)
                agent.moveD('right', [0, d * math.cos(a), -d * math.sin(a)], relative=True)
                agent.moveD('right', [0, -d * math.cos(a), d * math.sin(a) + 0.1], relative=True)

            agent.movej('left', [0, 0, 0, 0, 0, a], relative=True)

            # Rewind the moves
            agent.movej('left', j2)
            agent.movej('left', j1)
            agent.open_gripper()
            agent.moveD('left', [-0.2, 0, 0], relative=True)
            #agent.moveD('right', [0, -0.1, 0.1], relative=True)
            top = [-2.5499, -4.3371, 0.5642, -0.4306, -0.9456, 2.3821]
            agent.movej('right', top)
            raw_input()
        elif obj == 'noodle':
            agent.open_gripper()
        else:
            tr = agent.obj_trajectories[obj]
            print(tr)
            # [side, [j1~j6], [j1~j6]]
            side = tr[0]
            for i in reversed(range(len(tr))):
                if i == 0:
                    break
                agent.movej(side, tr[i])
                if side == 'right' and i == len(tr) - 1:
                    raw_input()
            if side == 'left':
                agent.open_gripper()
            else:
                agent.hand_action('default')
            if obj == 'kettle_handle' :
                agent.moveD(side, [0, -0.2, 0], relative=True)

            if obj == 'knife_handle' :
                agent.moveD(side, [0.06, 0, 0], relative=True)
                agent.moveD(side, [0, -0.05, 0], relative=True)
            agent.moveD(side, [0, 0, 0.2], relative=True)