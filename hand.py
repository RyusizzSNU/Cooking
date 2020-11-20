import rospy
from std_msgs.msg import Float32, String
from sensor_msgs.msg import JointState


class AllegroHandController(object):
    def __init__(self):
        envelop_publisher_topic = '/allegroHand_0/envelop_torque'
        libcmd_publisher_topic = '/allegroHand_0/lib_cmd'
        joint_cmd_publisher_topic = '/allegroHand_0/joint_cmd'
        self.envelop_publisher = rospy.Publisher(envelop_publisher_topic, Float32, queue_size=2)
        self.libcmd_publisher = rospy.Publisher(libcmd_publisher_topic, String, queue_size=2)
        self.joint_cmd_publisher = rospy.Publisher(joint_cmd_publisher_topic, JointState, queue_size=2)
        #while self.envelop_publisher.get_num_connections() == 0:
        #    rospy.sleep(0.1)
        #    print('wait')

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

    def grab(self): #custom grab
        next_position = JointState()
        name = ['joint_' + str(i) + '.0' for i in range(16)]
        velocity = [-3.493018918753451, -40.103029702888406, 120.73586545387582, -36.63389744821353, 252.3295750549728, -115.97092579748497, -7.3510231788609115, 56.5863491018089, -111.2786242991041, -3.362467567000076, -22.549172696996294, 53.08861330082114, 92.40275207505158, -145.99928833604542, -53.02169036962198, 5.906627932489557]
        position = [-0.2186851423965558, 1.5572975886568314, -0.28873799746659295, -0.27224139196685554,
                    -0.02539896673373966, 1.4753744333800771, 0.16015237484164643, -0.2642465593939885,
                    -0.026517322780583195, 1.3449399056894413, 0.34697985770819895, -0.2732642937066182,
                    1.44888367057904, 0.5894044733554243, 0.46002011041744784, -0.27192144040916066]

        next_position.name.extend(name)
        # next_position.velocity.extend(velocity)
        next_position.position.extend(position)
        self.joint_cmd(next_position)

        next_position = JointState()
        position = [-0.2548479719202039, 1.580944340340068, 1.5098884528535403, 1.3263597740978301,
                    -0.07431528095025002, 1.673592454066377, 1.49432093538029, 1.2808780340699661, 0.15803696150783358,
                    1.6904838961666842, 1.394033798795872, 1.5625461044099245, 1.3645063861341296, 0.2950326750005964,
                    0.19242855784779547, 1.6112683485881383]
        next_position.name.extend(name)
        next_position.position.extend(position)
        # next_position.velocity.extend(velocity)
        self.joint_cmd(next_position)



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

