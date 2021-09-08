import math
from agent import Agent
import utils
import time

def manipulate_bowl(agent, obj):
    agent.idle('left', view='top', start_closed=True)

    # Reach and grip the bowl
    agent.reach('left', obj, align_axis_from=0, align_axis_to=[0, 0, 1])
    agent.close_gripper()
    j1 = agent.getj('left')
    time.sleep(1)

    # Raise
    agent.moveD('left', [0, 0, 0.4], relative=True)
    j2 = agent.getj('left')

    if obj == 'rice_bowl' or obj == 'salt_bowl':
        agent.movej('left', [0, 0, 0, 0, 0, 0.5], relative=True)

    # Position and Orientations for pouring
    pos = [0.64, 0.40, 0.53]
    a = math.pi / 9
    rot1 = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
    rot2 = [[0, 1, 0], [-math.sin(a), 0, math.cos(a)], [math.cos(a), 0, math.sin(a)]]
    rot3 = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
    rot4 = [[1, 0, 0], [0, math.sin(a), math.cos(a)], [0, -math.cos(a), math.sin(a)]]

    # Get to the position, rotate to pour and rotate back
    if obj == 'rice_bowl' or obj =='oil_bowl':
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

    # Rewind the moves
    agent.movej('left', j2)
    agent.movej('left', j1, vel=0.2)
    agent.open_gripper()
    agent.moveD('left', [0, 0, 0.2], relative=True)

if __name__ == '__main__':
    agent = Agent()
    agent.ready('left')
    manipulate_bowl(agent, 'salt_bowl')