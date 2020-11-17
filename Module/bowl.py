import time
import math
from agent import Agent
import numpy as np
from scipy.spatial.transform import Rotation as R

obj = 'rice_bowl'

agent = Agent()
agent.ready('left')
agent.idle('left', side_view=False)
time.sleep(1)

# Reach and grip the bowl
agent.reach('left', obj)
agent.gripper.close_gripper()
j1 = agent.robot['left'].getj()

# Move upwards
pose = agent.desk_to_base('left', [0, 0, 0.4], translation=0)
agent.movel_with_position('left', pose, 0.1, 0.1, True)
time.sleep(4)
j2 = agent.robot['left'].getj()

# Pre-calculate the orientation for pouring manipulation
pose2 = agent.desk_to_base('left', [0.68, 0.38, 0.45])

L_D_W1 = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
rot1 = R.from_dcm(np.matmul(agent.L_B_D['left'][:3, :3], L_D_W1)).as_rotvec()
a = math.pi / 9
L_D_W2 = np.array([[0, 1, 0], [-math.sin(a), 0, math.cos(a)], [math.cos(a), 0, math.sin(a)]])
rot2 = R.from_dcm(np.matmul(agent.L_B_D['left'][:3, :3], L_D_W2)).as_rotvec()
L_D_W3 = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
rot3 = R.from_dcm(np.matmul(agent.L_B_D['left'][:3, :3], L_D_W3)).as_rotvec()
L_D_W4 = np.array([[1, 0, 0], [0, math.sin(a), math.cos(a)], [0, -math.cos(a), math.sin(a)]])
rot4 = R.from_dcm(np.matmul(agent.L_B_D['left'][:3, :3], L_D_W4)).as_rotvec()

# Rotate to pour and rotate back
if obj == 'rice_bowl' or obj =='oil_bowl':
    agent.robot['left'].movel(np.concatenate([pose2, rot1]), 0.1, 0.1, relative=False)
    time.sleep(1)

    agent.robot['left'].movel(np.concatenate([pose2, rot2]), 0.2, 0.2, relative=False)
    time.sleep(1)

    agent.robot['left'].movel(np.concatenate([pose2, rot1]), 0.2, 0.2, relative=False)
    time.sleep(1)
elif obj == 'salt_bowl':
    agent.robot['left'].movel(np.concatenate([pose2, rot1]), 0.2, 0.2, relative=False)
    time.sleep(1)

    agent.robot['left'].movel(np.concatenate([pose2, rot3]), 0.2, 0.2, relative=False)
    time.sleep(1)

    agent.robot['left'].movel(np.concatenate([pose2, rot4]), 0.2, 0.2, relative=False)
    time.sleep(1)

    agent.robot['left'].movel(np.concatenate([pose2, rot3]), 0.2, 0.2, relative=False)
    time.sleep(1)

    agent.robot['left'].movel(np.concatenate([pose2, rot1]), 0.2, 0.2, relative=False)
    time.sleep(1)

# Rewind the moves
agent.robot['left'].movej(j2, 0.1, 0.1, relative=False)
time.sleep(1)
agent.robot['left'].movej(j1, 0.2, 0.2, relative=False)
time.sleep(1)
agent.gripper.open_gripper()

agent.close()
