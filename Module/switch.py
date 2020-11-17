import time
import math
from agent import Agent

agent = Agent()
agent.ready('left')
agent.gripper.close_gripper()
agent.idle('left', side_view=True)
time.sleep(1)
agent.reach('left', 'switch')
agent.gripper.close_gripper()
a = math.radians(30)

time.sleep(2)
agent.robot['left'].movej([0, 0, 0, 0, 0, a], 0.1, 0.1, relative=True)
time.sleep(3)
agent.gripper.open_gripper()
agent.close()
