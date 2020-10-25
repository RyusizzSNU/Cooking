
import urx
import sys
import time

# allegro_hand
import rospy
import math


def right_pose(pose):
    """
    Send a movel command to the robot. See URScript documentation.
    """
    tpose = [0, 0, 0, 0, 0, 0]  # relative
    b = -pose[0] / math.sqrt(2) - pose[2] / math.sqrt(2)
    c = pose[0] / math.sqrt(2) - pose[2] / math.sqrt(2)
    a = -pose[1]

    tpose[0] = a
    tpose[1] = b
    tpose[2] = c
    return tpose

if __name__ == '__main__':
    rospy.init_node('allegrohand_rospy_youngjae_test')
    rob = urx.Robot("192.168.1.109") #left : "192.168.1.66"
    try:
        v = 0.3
        a = 0.1
        fast_v = 3
        fast_a = 5.0
        right_a = 1.0
        time.sleep(1)
        init_pos = [-0.577,-0.057,0.444, 0.037, -0.69, 1.983]
        rob.movel(init_pos, acc=a, vel=v, relative=False)

        time.sleep(1)

        rob.movel(right_pose([-0.1, 0, 0, 0, 0, 0]), acc=a, vel=v, relative=True)
        rob.movel(right_pose([0, 0, -0.15, 0, 0, 0]), acc=a, vel=v, relative=True)
        rob.movel(right_pose([0, 0, -0.03, 0, 0, 0]), acc=a, vel=v, relative=True)
        # current height : -71
        # next : -146
        # ground : -161
        rob.movel(right_pose([0, 0, 0.1, 0, 0, 0]), acc=fast_a, vel=fast_v, relative=True)

        for i in range(7):
            rob.movel(right_pose([0, 0, -0.1, 0, 0, 0]), acc=fast_a, vel=fast_v, relative=True)
            rob.movel(right_pose([0, 0.01, 0, 0, 0, 0]), acc=right_a, vel=fast_v, relative=True)
            rob.movel(right_pose([0.01, 0, 0, 0, 0, 0]), acc=right_a, vel=fast_v, relative=True)
            rob.movel(right_pose([-0.01, -0.01, 0.1, 0, 0, 0]), acc=right_a, vel=fast_v, relative=True)
            rob.movel(right_pose([-0.02, 0, 0, 0, 0, 0]), acc=right_a, vel=fast_v, relative=True)

            rospy.sleep(0.5)
        rob.movel(right_pose([0, 0, 0.03, 0, 0, 0]), acc=a, vel=v, relative=True)
    except Exception as ex:
        print("Robot could not execute move (emergency stop for example), do something", ex)

    finally:
        rob.close()
        print('done')