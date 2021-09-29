import urx
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
import threading, time
from pynput.keyboard import Key, Listener
from pynput import keyboard
import Module.frypan as frypan

inst = "ideal"
def get_inst(key):
    # while True:
    global inst
    inst = int(key.char)
    print("key: ", inst)

def main():
    keyboard_thread = keyboard.Listener(on_press=get_inst)
    keyboard_thread.start()
    # robot_L = urx.Robot('192.168.1.66')
    # robot_R = urx.Robot('192.168.1.109')
    try:
        while True:
            loop()
    except Exception as ex:
        print(ex)
    finally:
        pass
        # robot_L.close()
        # robot_R.close()

def loop():
    global inst
    if inst == 0:
        # print(inst)
        agent = frypan.left()
        agent.ready()
        agent.grip('pan_handle_handle')
        agent.robot.close()
        inst = 1
    elif inst == 1:
        print(inst)
        inst = "ideal"


if __name__ == '__main__':
    main()