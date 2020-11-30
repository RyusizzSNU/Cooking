import time
import math
from agent import Agent
import utils
import numpy as np


def manipulate_board(agent):
    agent.idle('left', view='lateral', start_closed=True)

    # Reach and grip board handle
    agent.reach('left', 'board_handle', align_axis_from=0, align_axis_to=[0, 0, 1])
    agent.close_gripper()
    j1 = agent.getj('left')
    time.sleep(1)

    # Raise
    agent.moveD('left', [0, 0, 0.1], relative=True)
    j2 = agent.getj('left')

    # Move above the pan
    agent.moveD('left', [0.38, 0.36, 0.44])

    # Rotate to pour and rotate back
    a = math.radians(30)
    agent.movej('left', [0, 0, 0, 0, 0, -a], vel=0.2, relative=True)
    time.sleep(1)

    a = math.radians(30)
    a2 = math.radians(5)
    d = 0.15
    pos = np.array([0.7779, 0.3306, 0.5286]) + np.array([0, 0, 0.1])
    rot1 = np.array([[-1, 0, 0], [0, math.sin(a), math.cos(a)], [0, math.cos(a), -math.sin(a)]])
    rot2 = np.array([[math.cos(a2), -math.sin(a2), 0], [math.sin(a2), math.cos(a2), 0], [0, 0, 1]])

    rot = np.matmul(rot2, rot1).T

    agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=pos))
    for i in range(3):
        agent.moveD('right', [0, 0, -0.1], relative=True)
        agent.moveD('right', [0, d * math.cos(a), -d * math.sin(a)], relative=True)
        agent.moveD('right', [0, -d * math.cos(a), d * math.sin(a) + 0.1], relative=True)

    agent.movej('left', [0, 0, 0, 0, 0, a], vel=0.2, relative=True)

    # Rewind the moves
    agent.movej('left', j2)
    agent.movej('left', j1, vel=0.2)
    agent.open_gripper()

if __name__ == '__main__':
    agent = Agent()
    agent.ready('left')
    agent.ready('right')
    manipulate_board(agent)