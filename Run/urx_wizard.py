from __future__ import print_function

import urx
import json
import rospy
import thread
import threading
import time
from queue import Queue
# from Module.set_idle import *
from std_msgs.msg import Float64MultiArray
import sys
from select import select


# def raw_input_with_timeout(prompt, timeout=30.0):
#     print(prompt, end=' ')
#     timer = threading.Timer(timeout, thread.interrupt_main)
#     astring = None
#     try:
#         timer.start()
#         astring = input(prompt)
#     except KeyboardInterrupt:
#         pass
#     timer.cancel()
#     return astring

# def raw_input_with_timeout(prompt, timeout=30.0):
#     print(prompt, end=' ')
#     finishat = time.time() + timeout
#     result = []
#     while True:
#         if msvcrt.kbhit():
#             result.append(msvcrt.getche())
#             if result[-1] == '\r':   # or \n, whatever Win returns;-)
#                 return ''.join(result)
#             time.sleep(0.1)          # just to yield to other processes/threads
#         else:
#             if time.time() > finishat:
#                 return None

def raw_input_with_timeout(prompt, timeout=10.0):
    key = None
    print(prompt,)
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.readline()
    # else:
    #     print("No input. Moving on...")
    return key


class URXWizard(object):

    def __init__(self, config_path, arm):
        self.arm = arm
        self.config = {}
        with open(config_path, 'r') as json_file:
            self.config = json.load(json_file)
        # if arm == 'right':
        #     self.rob = urx.Robot(self.config['ip']['right'])
        # elif arm == 'left':
        self.rob = urx.Robot(self.config['ip'][arm])
        self.rob.set_tcp((0, 0, 0, 0, 0, 0))
        self.rob.set_payload(0.9, (0, 0, 0))
        initj = self.rob.getj()
        print("Initial joint configuration is ", initj)
        # else:
        #     raise

        self.action_queue = Queue()


    def change_pose(self, pose_id):
        if 0 <= pose_id < len(self.config['pose'][self.arm]):
            print("pose_id", pose_id, type(pose_id))
            pose = self.config['pose'][self.arm][pose_id]
            print("pose", pose)
            return self.set_posej(pose, 0.3, 0.5)
        else:
            print("exception test")
            return False

    # def set_idle(self, rob, arm, vars, **args):
    #     """
    #     move arm to idle pose
    #     """
    #     if arm == 'left':
    #         idle_pose = vars['left_idle']
    #     elif arm == 'right':
    #         idle_pose = vars['right_idle']
    #     else:
    #         print("wrong arm!")
    #         return False
    #     try:
    #         rob.movej(idle_pose, acc=args['a'], vel=args['v'], relative=False)
    #     except Exception as ex:
    #         print("Robot could not execute move (emergency stop for example), do something", ex)
    #         return False
    #     return True

    def set_posej(self, pose, a, v):
        """
        move arm to given pose
        """
        # time.sleep(1)
        _ = self.rob.movej(pose, acc=a, vel=v, relative=False)
        # time.sleep(1)
        return True

    def loop(self):

        # key = raw_input_with_timeout("pose id : ", timeout=5)
        # key = None
        timeout = 60.0
        prompt = 'pose id : '
        print("Enter something:")
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.readline()
            print("input key", key, type(key), type(int(key)))
            self.action_queue.put(int(key))
        else:
            print("No input. Moving on...")
        # return key
        # if key is not None:
        while not self.action_queue.empty():
            print("output key")
            self.change_pose(self.action_queue.get())
        rospy.rostime.wallsleep(0.5)


if __name__ == '__main__':
    rospy.init_node('urx_' + sys.argv[2])
    # rospy.spin()

    urx_wizard = URXWizard(sys.argv[1], sys.argv[2])
    try:
        while not rospy.core.is_shutdown():
            urx_wizard.loop()
    except Exception as ex:
        print("Robot could not execute move (emergency stop for example), do something", ex)
    finally:
        urx_wizard.rob.close()
