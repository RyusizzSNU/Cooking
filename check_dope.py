import rospy
from dope_reader import DopeReader
from agent import Agent
from multiprocessing import Process
import time

def check_obj(agent, side, obj):
    side_str = 'L' if side == 'left' else 'R'
    rospy.set_param('/dope/activities/%s/%s' % (side_str, obj), True)
    time.sleep(1)
    try:
        agent.dope_reader[side].get_obj_pos(obj)
    except (AssertionError, KeyError):
        print('%s %s failed'%(side, obj))
        rospy.set_param('/dope/activities/%s/%s' % (side_str, obj), False)
        return
    print('%s %s success'%(side, obj))
    rospy.set_param('/dope/activities/%s/%s' % (side_str, obj), False)

agent = Agent()
agent.ready('left')
time.sleep(2)
agent.ready('right')
agent.idle('left', view='top', start_closed=True)
agent.idle('right', view='top2', start_closed=True)

processes = []
for obj in ['carrot', 'onion', 'spam', 'rice_bowl', 'oil_bowl', 'salt_bowl']:
    check_obj(agent, 'left', obj)

for obj in ['knife_handle', 'paddle_handle']:
    check_obj(agent, 'right', obj)

agent.close()