from Instruction import Instruction, Param
import math
import utils
import numpy as np

class Pour(Instruction):
    def __init__(self):
        self.display_name = 'Pour'
        self.function = self.pour
        self.cursor = 0
        self.params = [Param('Object', {
                    'Oil Bowl' : 'oil_bowl',
                    'Salt Bowl' : 'salt_bowl',
                    'Rice Bowl' : 'rice_bowl',
                    'Kettle Handle' : 'kettle_handle'
            })
        ]

    def pour(self, agent, obj):
        #if obj == 'rice_bowl' or obj == 'salt_bowl':
        #    agent.movej('left', [0, 0, 0, 0, 0, 0.5], relative=True)

        # Position and Orientations for pouring
        pos = [0.64, 0.40, 0.53]
        a = math.pi / 9
        rot1 = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
        rot2 = [[0, 1, 0], [-math.sin(a), 0, math.cos(a)], [math.cos(a), 0, math.sin(a)]]
        rot3 = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
        rot4 = [[1, 0, 0], [0, math.sin(a), math.cos(a)], [0, -math.cos(a), math.sin(a)]]

        # Get to the position, rotate to pour and rotate back
        if obj == 'rice_bowl' or obj == 'oil_bowl':
            print(utils.affine_to_tcp(dcm=rot1, t=pos))
            agent.moveD('left', utils.affine_to_tcp(dcm=rot1, t=pos))
            agent.moveD('left', utils.affine_to_tcp(dcm=rot2, t=pos))
            agent.moveD('left', utils.affine_to_tcp(dcm=rot1, t=pos))
        elif obj == 'salt_bowl':
            agent.moveD('left', utils.affine_to_tcp(dcm=rot1, t=pos))
            agent.moveD('left', utils.affine_to_tcp(dcm=rot3, t=pos))
            agent.moveD('left', utils.affine_to_tcp(dcm=rot4, t=pos))
            agent.moveD('left', utils.affine_to_tcp(dcm=rot3, t=pos))
            agent.moveD('left', utils.affine_to_tcp(dcm=rot1, t=pos))
        elif obj == 'kettle_handle':
            pos2 = [0.64, 0.40, 0.63]
            agent.moveD('left', pos2)
            angle = math.radians(60)
            agent.movej('left', [0, 0, 0, 0, 0, angle], vel=0.2, relative=True, wait=True)
            agent.movej('left', [0, 0, 0, 0, 0, -angle], vel=0.2, relative=True, wait=True)
