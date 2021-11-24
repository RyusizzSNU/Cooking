from Instruction import Instruction, Param
import math
import utils
import numpy as np

class Sprinkle(Instruction):
    def __init__(self):
        self.display_name = 'Sprinkle'
        self.function = self.sprinkle
        self.cursor = 0
        self.params = [Param('Object', {
                    'Pepper Bowl' : 'pepper_bowl'
            })
        ]

    def sprinkle(self, agent, obj):
        #if obj == 'rice_bowl' or obj == 'salt_bowl':
        #    agent.movej('left', [0, 0, 0, 0, 0, 0.5], relative=True)

        # Position and Orientations for pouring
        pos = [0.64, 0.40, 0.53]
        agent.moveD('left', pos)
        angle1 = math.radians(45)
        angle2 = math.radians(160)
        agent.movej('left', [0, 0, 0, 0, angle1, 0], vel=1, relative=True, wait=True)
        agent.movej('left', [0, 0, 0, 0, 0, angle2], vel=1 ,relative=True, wait=True)
        l1 = agent.getl('left')
        l2 = np.copy(l1)
        l2[1] -= 0.05
        l2[2] -= 0.05
        ls = [l1, l2, l1, l2, l1, l2, l1]
        agent.movels('left', ls, acc=2, vel=5)
        agent.movej('left', [0, 0, 0, 0, 0, -angle2], vel=1 ,relative=True, wait=True)
        agent.movej('left', [0, 0, 0, 0, -angle1, 0], vel=1, relative=True, wait=True)
