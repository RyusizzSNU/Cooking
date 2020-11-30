import math
from agent import Agent
import time

def manipulate_switch(agent):
    agent.idle('left', view='lateral', start_closed=True)

    # Reach and grip switch
    agent.reach('left', 'switch', align_axis_from=0, align_axis_to=[0, 0, 1])
    agent.close_gripper()
    # Rotate
    agent.movej('left', [0, 0, 0, 0, 0, math.radians(30)], vel=0.2, relative=True)

    agent.open_gripper()

if __name__ == '__main__':
    agent = Agent()
    agent.ready('left')
    manipulate_switch(agent)