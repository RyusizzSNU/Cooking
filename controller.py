import pygame
from agent import Agent
from enum import Enum
from recipe_parser import get_all_recipes, read_recipe

import time
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CameraInfo, Image as ImageSensor_msg
import multiprocessing as mp

from cv_bridge import CvBridge

class State(Enum):
    MAIN = 0
    CONTROL = 1
    MOVE = 2
    TEST_ACTION = 3
    TEST_RECIPE = 4

class Controller:
    def __init__(self, agent):
        self.agent = agent
        self.state = State.MAIN
        self.cursor = 1
        self.side = 'left'
        self.hand_control = False
        self.move_scale = 1

        self.selected_pose = None
        self.selected_action = None
        self.selected_params = []
        self.selected_recipe = None
        self.selected_instruction = None

        self.font = 'calibri'
        self.text_size = 30
        self.screen_scale = 1
        self.screen_size = np.array([1800, 800])
        self.bind_key_with_control()

        self.control_explain_image = { 'left' : cv2.imread('left.png'), 'right' : cv2.imread('right.png') }

        self.cv_bridge = CvBridge()
        self.cam_connected = {'left' : False, 'right' : False}
        
        self.sides = ['left', 'right']

        pygame.init()
        self.screen = pygame.display.set_mode((self.screen_size[0] * self.screen_scale, self.screen_size[1] * self.screen_scale))
        pygame.display.set_caption('Robot Controller')

    def check_connections(self):
        for side in self.sides:
            topic = 'dope/L/rgb_points' if side == 'left' else 'dope/R/rgb_points'
            
            try:
                t = time.time()
                rospy.wait_for_message(topic, ImageSensor_msg, timeout=1)
                self.cam_connected[side] = True
                print('%s camera connected in %s seconds'%(side, time.time() - t))
            except rospy.exceptions.ROSException:
                self.cam_connected[side] = False
                print('Failed to connect %s camera'%side)

    def display_text(self, text, pos, size, font=None, color=None):
        if color is None:
            color = (0, 0, 0)
        if font is None:
            font = self.font

        fontObj = pygame.font.SysFont(font, size * self.screen_scale)
        text = fontObj.render(text, True, color)
        self.screen.blit(text, np.array(pos) * self.screen_scale)

    def display_image(self, image, pos, size=None):
        h, w = image.shape[:2]
        if size == None:
            size = np.array([w, h])
        elif size[0] == None:
            size = np.array([w * size[1] / h, size[1]])
        elif size[1] == None:
            size = np.array([size[0], h * size[0] / w])
        size = size * self.screen_scale

        image = cv2.resize(image, (size[0], size[1]), interpolation=cv2.INTER_CUBIC)
        image = cv2.transpose(image)
        surf = pygame.surfarray.make_surface(image)
        self.screen.blit(surf, np.array(pos) * self.screen_scale)

    def bind_key_with_control(self):
        self.key_control_dict = {
            pygame.K_s: ['d', [0, -0.01, 0]], pygame.K_w: ['d', [0, 0.01, 0]],
            pygame.K_a: ['d', [-0.01, 0, 0]], pygame.K_d: ['d', [0.01, 0, 0]],
            pygame.K_q: ['d', [0, 0, -0.01]], pygame.K_e: ['d', [0, 0, 0.01]],
            pygame.K_g: ['l', [0, -0.01, 0]], pygame.K_t: ['l', [0, 0.01, 0]],
            pygame.K_f: ['l', [-0.01, 0, 0]], pygame.K_h: ['l', [0.01, 0, 0]],
            pygame.K_r: ['l', [0, 0, -0.01]], pygame.K_y: ['l', [0, 0, 0.01]],
            pygame.K_1: ['j', [-0.01, 0, 0, 0, 0, 0]], pygame.K_2: ['j', [0.01, 0, 0, 0, 0, 0]],
            pygame.K_3: ['j', [0, -0.01, 0, 0, 0, 0]], pygame.K_4: ['j', [0, 0.01, 0, 0, 0, 0]],
            pygame.K_5: ['j', [0, 0, -0.01, 0, 0, 0]], pygame.K_6: ['j', [0, 0, 0.01, 0, 0, 0]],
            pygame.K_7: ['j', [0, 0, 0, -0.01, 0, 0]], pygame.K_8: ['j', [0, 0, 0, 0.01, 0, 0]],
            pygame.K_9: ['j', [0, 0, 0, 0, -0.01, 0]], pygame.K_0: ['j', [0, 0, 0, 0, 0.01, 0]],
            pygame.K_o: ['j', [0, 0, 0, 0, 0, -0.01]], pygame.K_p: ['j', [0, 0, 0, 0, 0, 0.01]],
        }
        qwerty_key_list1 = [pygame.K_q, pygame.K_w, pygame.K_e, pygame.K_r, pygame.K_t, pygame.K_y, pygame.K_u, pygame.K_i, pygame.K_o, pygame.K_p]
        qwerty_key_list2 = [pygame.K_a, pygame.K_s, pygame.K_d, pygame.K_f, pygame.K_g, pygame.K_h, pygame.K_j, pygame.K_k, pygame.K_l, pygame.K_SEMICOLON]

        self.key_control_dict_hand = {}
        for i in range(8):
            joint = np.zeros(16)
            joint[i] = 1
            self.key_control_dict_hand[qwerty_key_list1[i]] = joint

            joint = np.zeros(16)
            joint[i+8] = 1
            self.key_control_dict_hand[qwerty_key_list2[i]] = joint


    def move(self, side, command, vec, relative):
        if command == 'd':
            self.agent.moveD(side, vec, acc=1, vel=1, relative=relative, wait=False)
        elif command == 'j':
            self.agent.movej(side, vec, acc=1, vel=1, relative=relative, wait=False)
        elif command == 'l':
            self.agent.movel(side, vec, acc=1, vel=1, relative=relative, wait=False)

    def get_text_action_list(self):
        text_action_list = []
        if self.state == State.MAIN:
            def transit_state(state):
                self.state = state
                self.cursor = 1
            text_action_list = [
                ['Manual Control', transit_state, [State.CONTROL]],
                ['Simple Movement', transit_state, [State.MOVE]],
                ['Test Action', transit_state, [State.TEST_ACTION]],
                ['Test Full Recipe', transit_state, [State.TEST_RECIPE]]
            ]
        elif self.state == State.CONTROL:
            '''text_action_list = [['Press Space to switch arm, current : ' + self.side],
                ['W, A, S, D, Q, E for Desk-based Movement'],
                ['T, F, G, H, R, Y for Robot-base-based Movement'],
                ['1, 2 for Base Joint'],
                ['3, 4 for Shoulder Joint'],
                ['5, 6 for Elbow Joint'],
                ['7, 8 for Wrist 1 Joint'],
                ['9, 0 for Wrist 2 Joint'],
                ['O, P for Wrist 3 Joint']]'''
            text_action_list = []
        elif self.state == State.MOVE:
            def select_pose(side, name):
                self.side = side
                self.selected_pose = name
            for side in self.agent.poses:
                for name in self.agent.poses[side]:
                    text_action_list.append([side + ' -> ' + name, select_pose, [side, name]])
        elif self.state == State.TEST_ACTION:
            actions_dict = agent.get_actions_dict()
            def select_action(action):
                self.selected_action = action
                self.cursor = 1

            def select_param(param_name):
                self.selected_action.assign_current_param(param_name)

            if self.selected_action is None:
                for action in actions_dict:
                    text_action_list.append([action.display_name, select_action, [action]])
            else:
                param_to_select = self.selected_action.current_param
                for param_name in param_to_select.name_value_dict:
                    text_action_list.append([param_to_select.display_name + " : " + param_name, select_param, [param_name]])

        elif self.state == State.TEST_RECIPE:
            recipes = get_all_recipes()
            def select_recipe(recipe):
                self.selected_recipe = recipe
                self.instructions = read_recipe(recipe)
                self.cursor = 1

            def select_instruction(instruction):
                self.selected_instruction = instruction

            if self.selected_recipe is None:
                for recipe in recipes:
                    text_action_list.append([recipe, select_recipe, [recipe]])
            else:
                text_action_list.append(['Run all', select_instruction, ['RunAll']])
                for instruction in self.instructions:
                    text_action_list.append([instruction[0] + ' ' + instruction[1], select_instruction, [instruction]])

        return text_action_list

    def process_state(self):
        if self.state == State.MAIN:
            self.side = 'left'
            self.selected_pose = None
            self.selected_action = None
            self.selected_params = []
            self.selected_recipe = None
            self.instructions = []
            self.selected_instruction = None
            
        elif self.state == State.MOVE:
            if self.selected_pose is not None:
                self.move(self.side, 'j', self.agent.poses[self.side][self.selected_pose], False)
                self.state = State.MAIN
                self.cursor = 1
        elif self.state == State.CONTROL:
            if self.state == State.CONTROL:
                keys = pygame.key.get_pressed()
                ctrl = keys[pygame.K_LCTRL]

                if self.hand_control:
                    for i in range(len(self.agent.hand_poses)):
                        if keys[pygame.K_1 + i]:
                            self.agent.move_hand(self.agent.hand_poses[i], relative=False)

                    for key in self.key_control_dict_hand:
                        if keys[key]:
                            joint = self.key_control_dict_hand[key]
                            if ctrl:
                                joint = -joint
                            self.agent.move_hand(joint * self.move_scale * 0.33, relative=True)

                else:
                    for key in self.key_control_dict:
                        if keys[key]:
                            fp = self.key_control_dict[key]
                            self.move(self.side, fp[0], np.array(fp[1]) * self.move_scale, True)

        elif self.state == State.TEST_ACTION:
            if self.selected_action is not None and self.selected_action.current_param is None:
                try:
                    self.selected_action(self.agent)
                except (AttributeError, TypeError):
                    pass
                self.state = State.MAIN
                self.cursor = 1
        elif self.state == State.TEST_RECIPE:
            if self.selected_recipe is not None and self.selected_instruction is not None:
                if self.selected_instruction == 'RunAll':
                    for instruction in self.instructions:
                         agent.execute_instruction(instruction)
                else:
                    agent.execute_instruction(self.selected_instruction)
                self.state = State.MAIN
                self.cursor = 1

    def display(self, text_action_list):
        self.display_text(self.state.name, [0, 0], self.text_size)
        if self.state == State.CONTROL:
            self.display_text('Move scale : ' + str(self.move_scale), [200, 0], self.text_size)
            self.display_text('Arm : ' + str(self.side), [450, 0], self.text_size)
            self.display_text('Mode : ' + ('Hand' if self.hand_control else 'Arm'), [600, 0], self.text_size)
            self.display_image(self.control_explain_image[self.side], [0, self.text_size])
        pos = [0, self.text_size]
        for i in range(len(text_action_list)):
            number = i if self.state == State.TEST_RECIPE and self.selected_recipe is not None else i + 1

            text = text_action_list[i][0]
            text = str(number) + '. ' + text
            if i == self.cursor - 1:
                text += '  <'
            self.display_text(text, pos, self.text_size)
            pos[1] += self.text_size

        if self.cam_connected[self.side]:
            try:
                img = rospy.wait_for_message('dope/L/rgb_points', ImageSensor_msg, timeout=1)
            except rospy.exceptions.ROSException:
                img = None
            
            if img is not None:
                img = self.cv_bridge.imgmsg_to_cv2(img, "rgb8")
                self.display_image(img, [1000, 0], [800, None])

    def run(self):
        clock = pygame.time.Clock()
        running = True
        
        self.check_connections()

        while running:
            self.screen.fill((255, 255, 255))

            text_action_list = self.get_text_action_list()
            self.display(text_action_list)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.agent.close()
                    running = False
                if event.type == pygame.KEYDOWN:
                    selected = -1
                    for i in range(len(text_action_list)):
                        if event.key == pygame.K_1 + i:
                            selected = i + 1
                    if event.key == pygame.K_RETURN or event.key == pygame.K_RIGHT:
                        selected = self.cursor

                    elif event.key == pygame.K_DOWN:
                        if self.cursor - 1 < len(text_action_list) - 1:
                            self.cursor += 1
                    elif event.key == pygame.K_UP:
                        if self.cursor - 1 > 0:
                            self.cursor -= 1

                    if event.key == pygame.K_PERIOD:
                        print(self.agent.hand.target)
                        print(self.agent.hand.joint)
                    
                    if self.state == State.CONTROL:
                        if event.key == pygame.K_SPACE:
                            self.side = 'right' if self.side == 'left' else 'left'
                        elif event.key == pygame.K_TAB:
                            self.hand_control = not self.hand_control
                        elif event.key == pygame.K_MINUS:
                            self.move_scale /= 2
                        elif event.key == pygame.K_EQUALS:
                            self.move_scale *= 2
                        self.move_scale = max(min(self.move_scale, 16), 1.0 / 16.0)

                    if selected != -1 and self.state != State.CONTROL:
                        text_action_list[selected - 1][1](*text_action_list[selected - 1][2])

                    elif (event.key == pygame.K_ESCAPE or event.key == pygame.K_LEFT) and self.state != State.MAIN:
                        self.state = State.MAIN
                        self.cursor = 1

            self.process_state()
            clock.tick(30)
            pygame.display.flip()

if __name__ == '__main__':
    agent = Agent()
    agent.ready('left')
    agent.ready('right')
    controller = Controller(agent)
    controller.run()
