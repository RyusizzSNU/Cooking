import time
import math
import utils
import numpy as np
from agent import Agent

def manipulate_paddle(agent):
    agent.idle('left', view='top2', start_closed=True)
    agent.idle('right', view='top2', start_closed=True)
    agent.reach('left', 'pan_handle_handle')
    agent.close_gripper()

    agent.reach('right', 'paddle_handle')
    agent.hand.grab()
    agent.moveD('right', [0, 0, 0.3], relative=True)
    raw_input()
    agent.ready_hand()
    raw_input()
    agent.hand.grab()
    raw_input()

    n = 8
    bound = np.array([[0.47, 0.40, 0.34], [0.57, 0.50, 0.341]])

    poses = np.random.uniform(bound[0], bound[1], (n, 3))

    a = math.radians(15)
    rot = np.array([[0, -math.sin(a), -math.cos(a)], [-1, 0, 0], [0, math.cos(a), -math.sin(a)]])
    rot = rot.T
    agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.57, 0.50, 0.48]))

    # Butterfly move
    for i in range(n):
        agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.51, 0.42, 0.38]), vel=0.5)
        agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.63, 0.54, 0.38]), vel=0.5)
        agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.63, 0.42, 0.38]), vel=0.5)
        agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.51, 0.54, 0.38]), vel=0.5)
    agent.moveD('right', [0, 0, 0.2], relative=True)
    # Random move
    '''for i in range(len(poses)):
        print(poses[i])
        agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=poses[i]), vel=0.5)'''

    agent.open_gripper()
    agent.moveD('left', [-0.2, 0, 0.1], relative=True, wait=False)
    agent.put_paddle_position()
    raw_input()
    agent.hand.lib_cmd('home')
    raw_input()
    agent.hand.grab()
    raw_input()
    agent.moveD('right', [0, 0, -0.1], relative=True)
    agent.hand.lib_cmd('home')
    raw_input()
    agent.moveD('right', [0, 0, 0.25], relative=True)
    agent.idle('right', view='top2')

if __name__ == '__main__':
    agent = Agent()
    agent.ready('left')
    time.sleep(1)
    agent.ready('right')
    manipulate_paddle(agent)