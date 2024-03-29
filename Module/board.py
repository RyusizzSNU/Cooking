import time
import math
from agent import Agent

agent = Agent()
agent.ready('left')
agent.idle('left', view='lateral', start_closed=False)

# Reach and grip board handle
agent.reach('left', 'board_handle', align_axis_from=0, align_axis_to=[0, 0, 1])
agent.close_gripper()
j1 = agent.getj('left')

# Raise
agent.moveD('left', [0, 0, 0.1], relative=True)
j2 = agent.getj('left')

# Move above the pan
agent.moveD('left', [0.45, 0.36, 0.38])

# Rotate to pour and rotate back
a = math.radians(30)
agent.movej('left', [0, 0, 0, 0, 0, -a], vel=0.2, relative=True)
time.sleep(1)
agent.movej('left', [0, 0, 0, 0, 0, a], vel=0.2, relative=True)
time.sleep(1)

# Rewind the moves
agent.movej('left', j2)
agent.movej('left', j1, vel=0.2)
agent.open_gripper()

agent.close()
