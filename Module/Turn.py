from Instruction import Instruction, Param
import math
import utils
import numpy as np

class Turn(Instruction):
    def __init__(self):
        self.display_name = 'Turn'
        self.function = self.turn
        self.cursor = 0
        self.params = [Param('Object', {
                'Switch' : 'switch',
            }), Param('OnOff', {
                'On' : 'on',
                'Off' : 'off'
            })
        ]

    def turn(self, agent, obj, onoff=True):
        agent.idle('left', view='lateral', start_closed=True)

        # Reach and grip switch
        agent.reach('left', 'switch', align_axis_from=0, align_axis_to=[0, 0, 1])
        agent.close_gripper()

        angle = math.radians(30) if onoff else math.radians(-30)
        # Rotate
        agent.movej('left', [0, 0, 0, 0, 0, angle], vel=0.2, relative=True, wait=True)

        agent.open_gripper()
        agent.moveD('left', [-0.2, 0, 0], relative=True)
        agent.idle('left', view='top', start_closed=True)
