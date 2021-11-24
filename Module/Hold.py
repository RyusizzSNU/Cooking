from Instruction import Instruction, Param
import math
import utils
import numpy as np

class Hold(Instruction):
    def __init__(self):
        self.display_name = 'Hold'
        self.function = self.hold
        self.cursor = 0
        self.params = [Param('Object', {
                    'Noodle' : 'noodle'
            })
        ]

    def hold(self, agent, obj):
        agent.movej('left', agent.poses['left']['stopover_noodle'], acc=0.2, vel=0.2, wait=True)
        pos_left = [0.48, 0.42, 0.53]
        pos_right = [0.82, 0.39, 0.57]
        rot_left = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
        rot_right = [[0, 0.7071, -0.7071], [0, 0.7071, 0.7071], [1, 0, 0]]
        rot_right = [[0.5, 0.5, -0.7071], [0.5, 0.5, 0.7071], [0.7071, -0.7071, 0]]
        agent.moveD('left', utils.affine_to_tcp(dcm=rot_left, t=pos_left))
        agent.moveD('right', utils.affine_to_tcp(dcm=rot_right, t=pos_right))
        agent.hand_action('grab')

        current_pos_D = agent.base_to_desk('left', affine=utils.tcp_to_affine(agent.getl('left')))
        a = math.radians(20)
        rot = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        target_pos_D = utils.transform(rot, dcm=current_pos_D)
        agent.moveD('left', utils.affine_to_tcp(dcm=target_pos_D, t=pos_left))
        agent.open_gripper()
