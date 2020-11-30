import time
import math
import utils
import numpy as np
from agent import Agent

def manipulate_knife(agent):
    agent.idle('right', view='top2', start_closed=True)

    agent.reach('right', 'knife_handle')
    agent.hand.grab()

    agent.moveD('right', [0, 0, 0.3], vel=0.25, relative=True)

    agent.movej('right', [0, 0, 0, 0, 1.57, 0], relative=True)
    j = agent.getj('right')

    n = 5
    a = math.pi / 12
    pos1 = np.array([0.82, 0.42, 0.492])
    pos2 = np.array([0.77, 0.42, 0.492])
    rot = np.array([[0, -math.cos(a), -math.sin(a)], [0, -math.sin(a), math.cos(a)], [-1, 0, 0]])
    #rot = np.array([[0, -math.cos(a), -math.sin(a)], [-1, 0, 0], [0, math.sin(a), -math.cos(a)]])
    rot = rot.T

    for k in range(2):
        for i in range(n):
            pos = pos1 * (n - 1 - i) / (n - 1) + pos2 * i / (n - 1)
            agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=pos))
            agent.moveD('right', [0, 0, -0.07], relative=True)
            agent.moveD('right', [0, -0.02, -0.02], vel=0.3, relative=True)
            agent.moveD('right', [0, 0.02, -0.02], vel=0.3, relative=True)
            agent.moveD('right', [0.04, 0, 0], relative=True)
            agent.moveD('right', [-0.04, 0, 0], relative=True)
            agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=pos))

    return j

def turnback_knife(agent, j):
    agent.movej('right', j)
    agent.movej('right', [0, 0, 0, 0, -1.57, 0], relative=True)
    agent.moveD('right', [0, 0, -0.2], vel=0.05, relative=True)
    agent.hand.lib_cmd('home')
    agent.moveD('right', [0, 0, 0.2], relative=True)

if __name__ == '__main__':
    agent = Agent()
    agent.ready('right')
    manipulate_knife(agent)