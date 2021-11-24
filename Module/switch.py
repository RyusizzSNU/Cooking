import math
import time

def manipulate_switch(agent, clockwise=True, wait=True):
    agent.idle('left', view='lateral', start_closed=True)

    # Reach and grip switch
    agent.reach('left', 'switch', align_axis_from=0, align_axis_to=[0, 0, 1])
    agent.close_gripper()

    angle = math.radians(30) if clockwise else math.radians(-30)
    # Rotate
    agent.movej('left', [0, 0, 0, 0, 0, angle], vel=0.2, relative=True, wait=wait)

    agent.open_gripper()
    agent.moveD('left', [-0.2, 0, 0], relative=True)
    agent.idle('left', view='top', start_closed=True)

    #agent.idle(11 'left', view='lateral', start_closed=True)

