import math
from agent import Agent
import utils

obj = 'onion'

agent = Agent()
agent.ready('left')
agent.idle('left', side_view=False, start_closed=True)

# Reach and grip the food
agent.reach('left', obj, align_axis_from=0, align_axis_to=[0, 0.5, -math.sqrt(3) / 2])
agent.close_gripper()

# Raise
agent.moveD('left', [0, 0, 0.3], relative=True)

# Move above the board
tcp = utils.affine_to_tcp(dcm=[[0, math.sqrt(3) / 2, 0.5], [1, 0, 0], [0, 0.5, -math.sqrt(3)/2]], t=[0.67, 0.15, 0.27])
agent.moveD('left', tcp)

# Move downwards right onto the board
agent.moveD('left', [0, 0, -0.1], vel=0.2, relative=True)

agent.open_gripper()

agent.close()
