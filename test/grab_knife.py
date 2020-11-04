import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
from sensor_msgs.msg import JointState
import urx
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

class AllegroHandController(object):
    def __init__(self):
        envelop_publisher_topic = '/allegroHand_0/envelop_torque'
        libcmd_publisher_topic = '/allegroHand_0/lib_cmd'
        joint_cmd_publisher_topic = '/allegroHand_0/joint_cmd'
        self.envelop_publisher = rospy.Publisher(envelop_publisher_topic, Float32, queue_size=2)
        self.libcmd_publisher = rospy.Publisher(libcmd_publisher_topic, String, queue_size=2)
        self.joint_cmd_publisher = rospy.Publisher(joint_cmd_publisher_topic, JointState, queue_size=2)
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

class DopeController(object):
    def __init__(self):
        self.pan_pose = None
        pan_handle_topic = '/dope/pose_pan_handle_handle'
        oil_bowl_topic = '/dope/pose_oil_bowl'
        knife_handle_topic = '/dope/pose_knife_handle'
        board_handle_topic = '/dope/pose_board_handle'
        switch_topic = '/dope/pose_switch'
        paddle_topic = '/dope/pose_paddle_handle'

        rospy.Subscriber(pan_handle_topic, PoseStamped, self.pan_handle_callback)
        rospy.Subscriber(oil_bowl_topic, PoseStamped, self.oil_bowl_callback)
        rospy.Subscriber(knife_handle_topic, PoseStamped, self.knife_handlel_callback)
        rospy.Subscriber(board_handle_topic, PoseStamped, self.board_handle_callback)
        rospy.Subscriber(switch_topic, PoseStamped, self.switch_callback)
        rospy.Subscriber(paddle_topic, PoseStamped, self.paddle_handle_callback)

        # rospy.Subscriber('/dope/rgb_points', Image, self.dope_image_callback)

    def pan_handle_callback(self, data):
        self.pan_handle_pose = data.pose

    def oil_bowl_callback(self, data):
        self.oil_bowl_pose = data.pose

    def knife_handlel_callback(self, data):
        self.knife_handle_pose = data.pose

    def board_handle_callback(self, data):
        self.board_handle_pose = data.pose

    def switch_callback(self, data):
        self.switch_pose = data.pose

    def salt_bowl_callback(self, data):
        self.salt_bowl_pose = data.pose

    def rice_bowl_callback(self, data):
        self.rice_bowl_pose = data.pose

    def paddle_handle_callback(self, data):
        self.paddle_handle_pose = data.pose


    def get_pose(self, object_name):
        try:
            if object_name == 'pan_handle':
                return self.pan_handle_pose.position, self.pan_handle_pose.orientation
            elif object_name == 'oil_bowl':
                return self.oil_bowl_pose.position, self.oil_bowl_pose.orientation
            elif object_name == 'knife_handle':
                return self.knife_handle_pose.position, self.knife_handle_pose.orientation
            elif object_name == 'board_handle':
                return self.board_handle_pose.position, self.board_handle_pose.orientation
            elif object_name == 'switch':
                return self.switch_pose.position, self.switch_pose.orientation
            elif object_name == 'salt_bowl':
                return self.salt_bowl_pose.position, self.salt_bowl_pose.orientation
            elif object_name == 'rice_bowl':
                return self.rice_bowl_pose.position, self.rice_bowl_pose.orientation
            elif object_name == 'paddle_handle':
                return self.paddle_handle_pose.position, self.paddle_handle_pose.orientation


        except Exception as e:
            print(e)
            print('[ERROR] Object is not detected yet!!!')
            return None, None

def go_to_handle(rob, knife_pose, knife_orient):
    v = 0.3
    a = 0.1
    # print('right_pose',right_pose([0, 0, 0, knife_orient.x, knife_orient.y, knife_orient.z]))
    rob.movel(right_pose([-knife_pose.y, -knife_pose.x,-knife_pose.z/3, 0, 0, 0]), acc=a, vel=v, relative=True)


    # rob.movel([0, 0, 0, knife_orient.x, knife_orient.y, knife_orient.z], acc=a, vel=v, relative=True)

    return Truea

if __name__ == '__main__':
    rospy.init_node('grab_knife')
    try:
        rob = urx.Robot("192.168.1.109") #left : "192.168.1.66"
        v = 0.3
        a = 0.1
        dope_confroller = DopeController()
        hand_controller = AllegroHandController()
        hand_controller.lib_cmd('home')
        init_pos = [-0.652, -0.035, 0.179, 0.367, -1.597, -3.732]
        # knife_orient = [-0.37384092840261085, -0.4464537731569344, 0.6221638157666863, 0.5232916730043076]
        rob.movel(init_pos, acc=a, vel=v, relative=False)
        rospy.sleep(10)
        handle_pose, handle_orient = dope_confroller.get_pose('paddle_handle')
        print('pan_pose x, y, z', handle_pose.x, handle_pose.y, handle_pose.z)
        print('pan_orient x, y, z, w', handle_orient.x, handle_orient.y, handle_orient.z, handle_orient.w)

        success = go_to_handle(rob, handle_pose, handle_orient)
        print('go to knife', str(success))
    except Exception as ex:
        print("Robot could not execute move (emergency stop for example), do something", ex)

    finally:
        rob.close()
        print('done')
