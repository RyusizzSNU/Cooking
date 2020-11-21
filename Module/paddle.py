import time
import math
import utils
import numpy as np
from agent import Agent

agent = Agent()
agent.ready('right')
#agent.idle('right', view='top2', start_closed=True)

#agent.reach('right', 'paddle_handle')
#agent.hand.lib_cmd('envelop')
#agent.hand.grab()

#agent.moveD('right', [0, 0, 0.3], relative=True)

n = 8
bound = np.array([[0.47, 0.40, 0.335], [0.57, 0.50, 0.336]])

poses = np.random.uniform(bound[0], bound[1], (n, 3))

rot = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
rot = rot.T
agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.52, 0.45, 0.50]))

pos_list = []
# Butterfly move
for i in range(n):
    agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.47, 0.40, 0.336]), vel=0.5)
    agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.57, 0.50, 0.336]), vel=0.5)
    agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.57, 0.40, 0.336]), vel=0.5)
    agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=[0.47, 0.50, 0.336]), vel=0.5)

# Random move
'''for i in range(len(poses)):
    print(poses[i])
    agent.moveD('right', utils.affine_to_tcp(dcm=rot, t=poses[i]), vel=0.5)'''

agent.close()
