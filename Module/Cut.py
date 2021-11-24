from Instruction import Instruction, Param
import math
import utils
import numpy as np

class Cut(Instruction):
    def __init__(self):
        self.display_name = 'Cut'
        self.function = self.cut
        self.cursor = 0
        self.params = [Param('Object', {
                'Spam' : 'spam',
                'Onion' : 'onion',
                'Spring Onion' : 'spring_onion'
            })
        ]

    def cut(self, agent, obj):
        top = [-2.5499, -4.3371, 0.5642, -0.4306, -0.9456, 2.3821]
        agent.movej('right', top)
        agent.moveD('right', [0, 0.1, -0.1], relative=True)
        agent.moveD('right', [0, 0, -0.27], relative=True)
        rot1 = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]

        a = math.radians(-5)
        b = math.radians(45)

        rot = [[0, 1, 0],
               [math.cos(a), 0, -math.sin(a)],
               [-math.sin(a), 0, -math.cos(a)]]
        rot = [[math.sin(b), math.sin(b), 0],
               [math.cos(a) * math.cos(b), -math.cos(a) * math.cos(b), -math.sin(a)],
               [-math.sin(a) * math.cos(b), math.sin(a) * math.cos(b), -math.cos(a)]]

        agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.65, 0.41, 0.555]))

        # chop
        n_chop = 3
        knife_height = 0.08
        chop_x_interval = 0.015
        to_obj = 0.06
        obj_to_doma = knife_height - to_obj
        n_slice_roundtrip = 3

        for i in range(n_chop):
            agent.moveD('right', [0, 0, -to_obj], relative=True)
            for _ in range(n_slice_roundtrip):
                depth_once = obj_to_doma / (n_slice_roundtrip * 2)
                agent.moveD('right', [0, -0.02, -depth_once], vel=0.3, relative=True)
                agent.moveD('right', [0, 0.02, -depth_once], vel=0.3, relative=True)

            agent.moveD('right', [0.02, 0, 0], relative=True)
            agent.moveD('right', [-0.02, 0, 0], relative=True)
            agent.moveD('right', [0, 0, knife_height], relative=True)

            if i != n_chop - 1:
                agent.moveD('right', [-chop_x_interval, 0, 0], relative=True, vel=0.5, acc=0.5)
        agent.movej('right', top)
