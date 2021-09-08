import time
import math
import utils
import numpy as np
from agent import Agent

def manipulate_knife(agent, food, pickup_knife=True):
    if pickup_knife:
        agent.idle('right', view='top2', start_closed=True)
        agent.reach('right', 'knife_handle')
        agent.hand.grab()
        agent.moveD('right', [0, 0, 0.3], vel=0.25, relative=True)
        raw_input()
        agent.ready_hand()
        raw_input()
        agent.hand.grab()
        raw_input()

        agent.movej('right', [0, 0, 0, 0, 1.57, 0], relative=True)

    n = 4
    a1 = math.radians(15)
    a2 = math.radians(30)
    if food == 'spam':
        pos1 = np.array([0.74, 0.39, 0.492])
        pos2 = np.array([0.71, 0.39, 0.492])
        height = 0.08
    elif food == 'onion':
        pos1 = np.array([0.75, 0.39, 0.497])
        pos2 = np.array([0.72, 0.39, 0.497])
        height = 0.04
    elif food == 'carrot':
        pos1 = np.array([0.77, 0.39, 0.492])
        pos2 = np.array([0.74, 0.39, 0.492])
        height = 0.02
    rot1 = np.array([[0, -math.cos(a1), -math.sin(a1)], [0, -math.sin(a1), math.cos(a1)], [-1, 0, 0]])
    #rot = np.array([[0, -math.cos(a), -math.sin(a)], [-1, 0, 0], [0, math.sin(a), -math.cos(a)]])
    rot1 = rot1.T

    for i in range(n):
        pos = pos1 * (n - 1 - i) / (n - 1) + pos2 * i / (n - 1)
        agent.moveD('right', utils.affine_to_tcp(dcm=rot1, t=pos))
        agent.moveD('right', [0, 0, -0.05], relative=True)
        agent.moveD('right', [0, -0.04, -0.01], vel=0.3, relative=True)
        agent.moveD('right', [0, 0.04, -0.01], vel=0.3, relative=True)
        agent.moveD('right', [0, -0.04, -0.01], vel=0.3, relative=True)
        agent.moveD('right', [0, 0.04, -0.01], vel=0.3, relative=True)
        agent.moveD('right', [0, -0.04, -0.01], vel=0.3, relative=True)
        agent.moveD('right', [0, 0.04, -0.01], vel=0.3, relative=True)

        agent.moveD('right', [0.04, 0, 0], relative=True)
        agent.moveD('right', [-0.04, 0, 0], relative=True)
        agent.moveD('right', utils.affine_to_tcp(dcm=rot1, t=pos))

def turnback_knife(agent):
    #agent.movej('right', j)
    #agent.movej('right', [0, 0, 0, 0, -1.57, 0], relative=True)
    agent.put_knife_position()
    raw_input()
    agent.hand.lib_cmd('home')
    raw_input()
    agent.hand.grab()
    raw_input()
    agent.moveD('right', [0, 0, -0.1], vel=0.05, relative=True)
    agent.hand.lib_cmd('home')
    raw_input()

    agent.moveD('right', [0, 0, 0.25], relative=True)
    agent.idle('right', view='top2')

if __name__ == '__main__':
    agent = Agent()
    agent.ready('right')
    #manipulate_knife(agent, 'onion', pickup_knife=True)
    turnback_knife(agent)