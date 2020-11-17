import math
from agent import Agent

agent = Agent()
agent.ready('left')
agent.idle('left', side_view=True, start_closed=True)

# Reach and grip switch
agent.reach('left', 'switch')
agent.close_gripper()

# Rotate
agent.movej('left', [0, 0, 0, 0, 0, math.radians(30)], vel=0.2, relative=True)

agent.open_gripper()
agent.close()
