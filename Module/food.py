import time
import math
from agent import Agent
import numpy as np
from scipy.spatial.transform import Rotation as R

agent = Agent()
agent.ready('left')
agent.idle('left', side_view=False, start_closed=True)
time.sleep(1)

# Reach and grip the food
agent.reach('left', 'onion')
agent.gripper.close_gripper()

# Raise
pose = agent.desk_to_base('left', [0, 0, 0.3], translation=0)
agent.movel_with_position('left', pose, 0.1, 0.1, True)
time.sleep(4)

# Move onto the board
pose2 = agent.desk_to_base('left', [0.67, 0.15, 0.27])
L_D_W = np.array([[0, math.sqrt(3) / 2, 0.5], [1, 0, 0], [0, 0.5, -math.sqrt(3)/2]])
rot = R.from_dcm(np.matmul(agent.L_B_D['left'][:3, :3], L_D_W)).as_rotvec()
agent.robot['left'].movel(np.concatenate([pose2, rot]), 0.1, 0.1, relative=False)
time.sleep(2)

# Move downwards right onto the board
pose = agent.desk_to_base('left', [0, 0, -0.1], translation=0)
agent.movel_with_position('left', pose, 0.2, 0.2, True)
time.sleep(2)

agent.gripper.open_gripper()

agent.close()
