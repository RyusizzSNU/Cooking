import time
import math
from agent import Agent

agent = Agent()
agent.ready('left')
agent.gripper.close_gripper()
agent.idle('left', side_view=True, start_closed=True)
time.sleep(1)

# Reach and grip switch
agent.reach('left', 'switch')
agent.gripper.close_gripper()
time.sleep(2)

# Rotate
agent.robot['left'].movej([0, 0, 0, 0, 0, math.radians(30)], 0.2, 0.2, relative=True)
time.sleep(2)

agent.gripper.open_gripper()
agent.close()
