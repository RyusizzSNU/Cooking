import math
from agent import Agent
import utils
import rospy

def manipulate_food(agent, obj):
    agent.idle('left', view='top', start_closed=True)

    # Reach and grip the food
    if obj == 'spam' :
        agent.reach('left', obj, align_axis_from=1, align_axis_to=[0, 0.5, -math.sqrt(3) / 2])
        #agent.reach('left', obj, align_axis_from=1, align_axis_to=[0, 0, -1])
        agent.gripper_action(127)
    else:
        agent.reach('left', obj, align_axis_from=0, align_axis_to=[0, 0.5, -math.sqrt(3) / 2])
        agent.close_gripper()

    # Raise
    agent.moveD('left', [0, 0, 0.4], relative=True)
    j = agent.getj('left')

    # Move above the board
    tcp = utils.affine_to_tcp(dcm=[[0, math.sqrt(3) / 2, 0.5], [1, 0, 0], [0, 0.5, -math.sqrt(3)/2]], t=[0.57, 0.20, 0.59])
    agent.moveD('left', tcp)

    # Move downwards right onto the board
    agent.moveD('left', [0, 0, -0.16], vel=0.2, relative=True)
    agent.open_gripper()

    if obj == 'onion':
        agent.moveD('left', [-0.035, 0, 0], relative=True)
    agent.close_gripper()

    return j

def turnback_food(agent, j):
    agent.movej('left', j)
    agent.moveD('left', [0, 0, -0.4], vel=0.2, relative=True)
    agent.open_gripper()
    agent.moveD('left', [0, 0, 0.4], relative=True)

if __name__ == '__main__':
    agent = Agent()
    agent.ready('left')
    manipulate_food(agent, 'spam')