import rospy
from std_msgs.msg import Float32, String
from sensor_msgs.msg import JointState
import numpy as np
import time

class AllegroHandController(object):
    def __init__(self):
        envelop_publisher_topic = '/allegroHand_0/envelop_torque'
        libcmd_publisher_topic = '/allegroHand_0/lib_cmd'
        joint_cmd_publisher_topic = '/allegroHand_0/joint_cmd'
        state_topic = '/allegroHand_0/joint_states'
        self.envelop_publisher = rospy.Publisher(envelop_publisher_topic, Float32, queue_size=2)
        self.libcmd_publisher = rospy.Publisher(libcmd_publisher_topic, String, queue_size=2)
        self.joint_cmd_publisher = rospy.Publisher(joint_cmd_publisher_topic, JointState, queue_size=2)

        rospy.Subscriber(state_topic, JointState, self.state_callback)
        self.joint_states = None
        while self.envelop_publisher.get_num_connections() == 0:
            rospy.sleep(0.1)
            print('wait')

    # it can set the torque envelop parameter value
    def envelop(self, val):
        print(Float32(val))
        self.envelop_publisher.publish(Float32(val))
        rospy.sleep(3)

    # it can be used like keyboard_controller
    # string list : envelop, ready, home... etc
    def lib_cmd(self, val):
        print(String(val))
        self.libcmd_publisher.publish(String(val))
        rospy.sleep(1)

    # input the joint
    def joint_cmd(self, val):
        self.joint_cmd_publisher.publish(val)
        print('set next position')
        rospy.sleep(0.5)

    def movej(self, joints=None, joint_num=None, val=None):
        next_position = JointState()
        name = ['joint_' + str(i) + '.0' for i in range(16)]
        next_position.name = name
        if joints is not None:
            next_position.position = joints
        elif joint_num is not None and val is not None:
            print(self.joint_states)
            next_position.position = list(self.joint_states.position)
            next_position.position[joint_num] = val
        effort = np.array([0.5, 0.75, 0.5, 0.75, 0.5, 0.75, 0.5, 0.75, 0.5, 0.75, 0.5, 0.75, 0.75, 0.75, 0.5, 0.5])
        next_position.effort = effort

        self.joint_cmd(next_position)

    def grab(self): #custom grab
        next_position = JointState()
        name = ['joint_' + str(i) + '.0' for i in range(16)]
        velocity = [-3.493018918753451, -40.103029702888406, 120.73586545387582, -36.63389744821353, 252.3295750549728, -115.97092579748497, -7.3510231788609115, 56.5863491018089, -111.2786242991041, -3.362467567000076, -22.549172696996294, 53.08861330082114, 92.40275207505158, -145.99928833604542, -53.02169036962198, 5.906627932489557]
        #position = [-0.03169593538969933, 1.4220984148287619, 0.7101407747942525, 1.1176176912241522, 0.13346401524068366, 1.5280307021719746, 0.49009169687580434, 1.2479782045877417, 0.25531029257282734, 1.634564959105132, 0.3227728848421474, 1.3604674336824014, 1.5428447454559708, 0.15895744413651658, 0.4481706633437269, -0.15737117478512253]
        pos1 = [-0.04, 1.7, 0, 0,
                    0.17, 1.68, 0, 0,
                    0.36, 1.71, 0, 0,
                    1.5, 0.50, 0, -1]
        pos2 = [-0.04, 1.7, 1.4, 0,
                0.17, 1.68, 1.37, 0,
                0.36, 1.71, 1.37, 0,
                1.5, 0.50, 0.5, -1]
        pos3 = [-0.04, 1.7, 1.4, 1.7,
                0.17, 1.68, 1.37, 1.67,
                0.36, 1.71, 1.37, 1.68,
                1.5, 0.50, 1, -1]
        pos4 = [-0.04, 1.7, 1.4, 1.7,
                0.17, 1.68, 1.37, 1.67,
                0.36, 1.71, 1.37, 1.68,
                1.5, 1.5, 1, 1]
        effort = np.array([0.5, 0.75, 0.5, 0.75, 0.5, 0.75, 0.5, 0.75, 0.5, 0.75, 0.5, 0.75, 0.75, 0.75, 0.5, 0.5])

        next_position.name.extend(name)
        # next_position.velocity.extend(velocity)
        next_position.position = pos1
        next_position.effort = effort
        self.joint_cmd(next_position)
        time.sleep(0.5)
        next_position.position = pos2
        next_position.effort = effort
        self.joint_cmd(next_position)
        time.sleep(0.5)
        next_position.position = pos3
        next_position.effort = effort
        self.joint_cmd(next_position)
        time.sleep(0.5)
        next_position.position = pos4
        next_position.effort = effort
        self.joint_cmd(next_position)


        #self.lib_cmd('envelop')

        # next_position = JointState()
        # position = [-0.2548479719202039, 1.580944340340068, 1.5098884528535403, 1.3263597740978301,
        #             -0.07431528095025002, 1.673592454066377, 1.49432093538029, 1.2808780340699661, 0.15803696150783358,
        #             1.6904838961666842, 1.394033798795872, 1.5625461044099245, 1.3645063861341296, 0.2950326750005964,
        #             0.19242855784779547, 1.6112683485881383]
        # next_position.name.extend(name)
        # next_position.position.extend(position)
        # # next_position.velocity.extend(velocity)
        # self.joint_cmd(next_position)

    def state_callback(self, data):
        self.joint_states = data

# it can show state value of hand joint
class AllegroHandStateController(object):
    def __init__(self):
        state_topic = '/allegroHand_0/joint_states'
        rospy.Subscriber(state_topic, JointState, self.state_callback)
        self.joint_states = None

    def state_callback(self, data):
        self.joint_states = data

    def get_joint_states(self):
        return self.joint_states

if __name__ == '__main__':
    rospy.init_node('allegrohand_rospy_youngjae_test')
    grab_mode = True
    if grab_mode:
        hand_controller = AllegroHandController()
        hand_controller.lib_cmd('home')
        hand_controller.grab()
    else:
        hand_state_controller = AllegroHandStateController()
    rospy.spin()

