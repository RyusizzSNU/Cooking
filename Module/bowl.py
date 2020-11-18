import math
from agent import Agent
import utils

obj = 'rice_bowl'

agent = Agent()
agent.ready('left')
agent.idle('left', view='top', start_closed=True)

# Reach and grip the bowl
agent.reach('left', obj, align_axis_from=0, align_axis_to=[0, 0, 1])
agent.close_gripper()
j1 = agent.getj('left')

# Raise
agent.moveD('left', [0, 0, 0.4], relative=True)
j2 = agent.getj('left')

# Position and Orientations for pouring
pos = [0.68, 0.38, 0.45]
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

agent.close()
