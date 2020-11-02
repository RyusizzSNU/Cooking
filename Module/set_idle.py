import urx


def set_idle(rob, arm, vars, **args):
    """
    move right arm to idel pose
    """
    if arm == 'left':
        idle_pose = vars['left_idle']
    elif arm == 'right':
        idle_pose = vars['right_idle']
    else:
        print("wrong arm!")
        return False
    try:
        rob.movej(idle_pose, acc, acc=args['a'], vel=args['v'], relative=False)
    except Exception as ex:
        print("Robot could not execute move (emergency stop for example), do something", ex)
        return False
    return True

if __name__ == '__main__':
    rob = urx.Robot("192.168.1.109") #left : "192.168.1.66", right: "192.168.1.109"
    args = {}
    vars = {}
    args['v'] = 0.3
    args['a'] = 0.1
    vars['left_idle'] = [5.73, -1.32, 2.27, -3.23, -2.14, 1.24]
    set_idle(rob, 'right', vars, **args)