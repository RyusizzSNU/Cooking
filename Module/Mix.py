from Instruction import Instruction, Param
import math
import utils
import numpy as np

class Mix(Instruction):
    def __init__(self):
        self.display_name = 'Mix'
        self.function = self.mix
        self.cursor = 0
        self.params = [Param('Object', {
                'Ladle Handle' : 'ladle_handle',
            })
        ]

    def mix(self, agent, obj):
        n = 4

        rot = [[0.7071, 0.7071, 0], [0, 0, 1], [0.7071, -0.7071, 0]]

        # rot = np.array([[0, -math.sin(a), -math.cos(a)], [-1, 0, 0], [0, math.cos(a), -math.sin(a)]])
        # rot = rot.T
        agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.65, 0.45, 0.49]))

        # Butterfly move
        for i in range(n):
            agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.6, 0.4, 0.39]), vel=0.5)
            agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.7, 0.5, 0.39]), vel=0.5)
            agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.7, 0.4, 0.39]), vel=0.5)
            agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.6, 0.5, 0.39]), vel=0.5)
        agent.moveD('right', [0, -0.1, 0.1], relative=True)
