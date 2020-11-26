import math
from agent import Agent
import utils

obj = 'spam'

agent = Agent()
agent.ready('left')
agent.idle('left', view='top', start_closed=True)

# Reach and grip the food
if obj == 'spam':
    agent.reach('left', obj, align_axis_from=1, align_axis_to=[0, 0, -1])
else:
    agent.reach('left', obj, align_axis_from=0, align_axis_to=[0, 0.5, -math.sqrt(3) / 2])
agent.close_gripper()

# Raise
agent.moveD('left', [0, 0, 0.3], relative=True)

# Move above the board
tcp = utils.affine_to_tcp(dcm=[[0, math.sqrt(3) / 2, 0.5], [1, 0, 0], [0, 0.5, -math.sqrt(3)/2]], t=[0.67, 0.15, 0.49])
agent.moveD('left', tcp)

# Move downwards right onto the board
agent.moveD('left', [0, 0, -0.08], vel=0.2, relative=True)

agent.open_gripper()

agent.close()
