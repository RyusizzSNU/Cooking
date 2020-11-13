import urx
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
import threading, time
from pynput.keyboard import Key, Listener
from pynput import keyboard
import Module.frypan as frypan

inst = "lol"
def get_inst(key):
    # while True:
    global inst
    inst = key
    print("this is thread1()")

def main():
    keyboard_thread = keyboard.Listener(on_press=get_inst)
    keyboard_thread.start()
    robot_L = urx.Robot('192.168.1.66')
    robot_R = urx.Robot('192.168.1.109')
    try:
        while true:
            loop()
    except Exception as ex:
        print(ex)
    finally:
        robot_L.close()
        robot_R.close()

def loop():
    if inst == '0':
        agent = frypan.left()
        agent.ready()
        agent.grip('pan_handle_handle')
        agent.robot.close()
    elif inst == '1':
        print(inst)

if __name__ == '__main__':
    main()