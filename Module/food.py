import math
from agent import Agent
import utils
import time
import rospy
import numpy as np

def manipulate_food(agent, obj, joint=range(6)):
    agent.idle('left', view='top', start_closed=True)
    # Reach and grip the food
    if obj == 'spam' :
        agent.reach('left', obj, align_axis_from=1, align_axis_to=[0, 0, -1])
        agent.gripper_action(127)
    else:
        agent.reach('left', obj, align_axis_from=0, align_axis_to=[0, 0, -1])
        agent.close_gripper()

    time.sleep(1)

    # Raise
    agent.moveD('left', [0, 0, 0.4], relative=True)
    joint[:] = agent.getj('left')

    rot1 = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
    rot2 = [[0, math.sqrt(3) / 2, 0.5], [1, 0, 0], [0, 0.5, -math.sqrt(3)/2]]

    # Move above the board
    tcp = utils.affine_to_tcp(dcm=rot1, t=[0.64, 0.22, 0.59])
    agent.moveD('left', tcp)

    # Move downwards right onto the board
    if obj == 'spam':
        agent.moveD('left', [0, 0, -0.15], vel=0.2, relative=True)
    else:
        agent.moveD('left', [0, 0, -0.18], vel=0.2, relative=True)

    agent.open_gripper()
    time.sleep(1)
    pos = agent.base_to_desk('left', agent.getl('left')[:3])

    if obj == 'onion':
        new_tcp = utils.affine_to_tcp(dcm=rot2, t=pos + np.array([-0.1, 0, -0.022]))
        agent.moveD('left', new_tcp)
        agent.close_gripper()
    elif obj == 'spam':
        new_tcp = utils.affine_to_tcp(dcm=rot2, t=pos + np.array([-0.1, 0, -0.02]))
        agent.moveD('left', new_tcp)
        agent.gripper_action(127)
    elif obj == 'carrot':
        new_tcp = utils.affine_to_tcp(dcm=rot2, t=pos + np.array([-0.08, 0, -0.02]))
        agent.moveD('left', new_tcp)
        agent.close_gripper()

def turnback_food(agent, j):
    agent.moveD('left', [0, 0, 0.2], relative=True)
    agent.movej('left', j)
    agent.moveD('left', [0, 0, -0.4], vel=0.2, relative=True)
    agent.open_gripper()
    agent.moveD('left', [0, 0, 0.4], relative=True)

if __name__ == '__main__':
    agent = Agent()
    agent.ready('left')
    manipulate_food(agent, 'spam')