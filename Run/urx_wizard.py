import urx
import json
import rospy
import threading
from queue import Queue
# from Module.set_idle import *
from std_msgs.msg import Float64MultiArray

def raw_input_with_timeout(prompt, timeout=30.0):
    print(prompt, end=' ')
    timer = threading.Timer(timeout, thread.interrupt_main)
    astring = None
    try:
        timer.start()
        astring = input(prompt)
    except KeyboardInterrupt:
        pass
    timer.cancel()
    return astring

class URXWizard(object):

    def __init__(self, config_path, arm):
        self.config = {}
        with open(config_path, 'r') as json_file:
            self.config = json.load(json_file)
        if arm == 'right':
            self.rob = urx.Robot("192.168.1.109")
        elif arm == 'left':
            self.rob = urx.Robot("192.168.1.66")
        else:
            raise

        self.action_queue = Queue()

    def change_pose(self, pose_id):
        pose = self.config_json['pose'][pose_id]
        return set_posej(pose, 0.3, 0.1)

    def set_idle(rob, arm, vars, **args):
        """
        move arm to idle pose
        """
        if arm == 'left':
            idle_pose = vars['left_idle']
        elif arm == 'right':
            idle_pose = vars['right_idle']
        else:
            print("wrong arm!")
            return False
        try:
            rob.movej(idle_pose, acc=args['a'], vel=args['v'], relative=False)
        except Exception as ex:
            print("Robot could not execute move (emergency stop for example), do something", ex)
            return False
        return True

    def set_posej(rob, pose, a, v):
        """
        move arm to given pose
        """
        try:
            rob.movej(pose, acc=a, vel=v, relative=False)
            time.sleep(1)
        except Exception as ex:
            print("Robot could not execute move (emergency stop for example), do something", ex)
            return False
        return True

    def loop(self):
        while not rospy.core.is_shutdown():
            key = raw_input_with_timeout('', timeout=5)
            self.action_queue.put(int(key))
            while not self.action_queue.empty():
                self.change_pose(self.action_queue.pop())
            rospy.rostime.wallsleep(0.5)


if __name__ == '__main__':
    rospy.init_node('urx')
    rospy.spin()

    urx_wizard = URXWizard()
    urx_wizard.main()