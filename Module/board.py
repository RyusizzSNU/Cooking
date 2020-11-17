import time
import math
from agent import Agent

agent = Agent()
agent.ready('left')
agent.idle('left', side_view=True)
time.sleep(1)

# Reach and grip board handle
agent.reach('left', 'board_handle')
agent.gripper.close_gripper()
j1 = agent.robot['left'].getj()

# Raise
pose1 = agent.desk_to_base('left', [0, 0, 0.3], translation=0)
agent.movel_with_position('left', pose1, 0.1, 0.1, True)
time.sleep(4)
j2 = agent.robot['left'].getj()

# Move onto the pan
pose2 = agent.desk_to_base('left', [0.45, 0.36, 0.38])
agent.movel_with_position('left', pose2, 0.1, 0.1, False)
time.sleep(3)

# Rotate to pour and rotate back
a = math.radians(30)
agent.robot['left'].movej([0, 0, 0, 0, 0, -a], 0.2, 0.2, relative=True)
time.sleep(2)
agent.robot['left'].movej([0, 0, 0, 0, 0, a], 0.2, 0.2, relative=True)
time.sleep(2)

# Rewind the moves
agent.robot['left'].movej(j2, 0.1, 0.1, relative=False)
time.sleep(1)
agent.robot['left'].movej(j1, 0.2, 0.2, relative=False)
time.sleep(1)

agent.gripper.open_gripper()
time.sleep(2)

agent.close()
