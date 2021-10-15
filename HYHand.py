import socket
import struct
import select
import numpy as np
import pygame
import time

class HYHandController():
    def __init__(self):
        self.robot_address = ('169.254.186.72', 7)
        self.target1 = np.array([0.33145142, -0.22052002, -0.50689697, -0.36474609,
                                -0.13763428, -0.67892456, -0.64254761,  0.60275269,
                                0.,          0.,          0.,          0.,
                                0.,          0.,          0.,          0.,        ])
        self.target2 = np.array([0.48254395, -0.4151001,  -0.64779663, -0.15914917,
                                0.,          0.,          0.,          0.,
                                0.,         -0.65106201, -0.57388306,  0.53234863,
                                0.,          0.,          0.,          0.        ])
        # self.target3 = np.array([0.73104858, -0.58682251, -0.68453979, -0.12161255,
        #                         0.,          0.,          0.,          0.,
        #                         0.,          0.,          0.,          0.,
        #                         0.24029541, -0.4473877,  -0.60327148,  0.52587891])
        # self.target4 = np.array([0.69778442, -0.38372803, -0.69482422, -0.69482422,
        #                         0.,         -0.69482422, -0.69482422,  0.52587891,
        #                         0.,         -0.69482422, -0.69482422,  0.52587891,
        #                         0.30517578, -0.69482422, -0.69482422,  0.52587891])
        self.target3 = np.array([0.69778442, -0.38, -0.69, -0.69482422,
                                 0., -1.5, -1.5, 1,
                                 0., -1.5, -1.5, 1,
                                 0.30517578, -1.5, -1.5, 1])
        self.target4 = np.array([0.69778442, -1, -1, -0.69482422,
                                 0., -1.5, -1.5, 1,
                                 0., -1.5, -1.5, 1,
                                 0.30517578, -1.5, -1.5, 1])
        self.target5 = np.zeros(16)
        self.target = np.zeros(16)
        self.joint = np.zeros(16)
        self.speed = 1 / 3.

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(self.robot_address)
        self._send_zero_msg()

    def _send_zero_msg(self):
        self.sock.send(self._torques_to_msg(np.zeros(16)))

    def _send_target_msg(self, target, current_pos):
        p = np.array(current_pos)
        assert len(p) == 16
        torque = 2 * (target - p)
        torque = np.clip(torque, -1, 1)
        self.sock.send(self._torques_to_msg(torque))

    def _msg_to_pos(self, msg):
        p = []
        for i in range(16):
            p.append(struct.unpack('>H', msg[2*i: 2*i+2])[0])
        p = (np.array(p) - 32768) / 32768.
        return p

    def _torques_to_msg(self, torques):
        assert len(torques) == 16
        torques = np.clip((torques * 32768 + 32768).astype(int), 0, 65534)
        msg = '\x00'
        for torque in torques:
            msg += struct.pack('>H', torque)
        return msg

    def _recv_all(self, length):
        buf = b''
        try:
            step = length
            while True:
                data = self.sock.recv(step)
                buf += data
                if len(buf) == length:
                    break
                elif len(buf) < length:
                    step = length - len(buf)
        except Exception as e:
            print('Error while receiving hand data :', e)
        return buf[:length]

    def move_joint(self, val, relative=True):
        prev_joint = np.zeros(16)
        start_time = time.time()
        not_moving = False
        
        if relative:
            self.target = self.target + val
        else:
            self.target = val

        while True:
            readied, _, _ = select.select([self.sock], [], [])
            #msg = self.sock.recv(32)
            msg = self._recv_all(32)
            self.joint = self._msg_to_pos(msg)

            self._send_target_msg(self.target, self.joint)
            if np.linalg.norm(self.joint - prev_joint) < 0.01:
                not_moving = True
            else:
                not_moving = False
                start_time = time.time()

            if not_moving and time.time() - start_time > 0.5:
                break
           
            prev_joint = self.joint

    def control(self):
        pygame.init()
        screen = pygame.display.set_mode((800, 800))

        target = self.target5

        self._send_zero_msg()
        np.set_printoptions(precision=2)

        val = 1 / 3.
        running = True
        i = 0
        prev_p = np.zeros(16)
        while running:
            keys = pygame.key.get_pressed()
            shift = keys[pygame.K_LSHIFT]
            ctrl = keys[pygame.K_LCTRL]
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.KEYDOWN:

                    if event.key == pygame.K_a:
                        target = self.target1
                        self.move_joint(target, relative=False)
                        break
                    elif event.key == pygame.K_s:
                        target = self.target2
                        self.move_joint(target, relative=False)
                        break
                    elif event.key == pygame.K_d:
                        target = self.target3
                        self.move_joint(target, relative=False)
                        break
                    elif event.key == pygame.K_f:
                        target = self.target4
                        self.move_joint(target, relative=False)
                        break
                    elif event.key == pygame.K_g:
                        target = self.target5
                        self.move_joint(target, relative=False)
                        break

                    elif event.key == pygame.K_MINUS:
                        val /= 2
                        print(val)
                    elif event.key == pygame.K_EQUALS:
                        val *= 2
                        print(val)

                    for i in range(8):
                        if event.key == pygame.K_1 + i:
                            if shift:
                                target[i + 8] += -val if ctrl else val
                                self.move_joint(target, relative=False)
                                print(target)
                            else:
                                target[i] += -val if ctrl else val
                                self.move_joint(target, relative=False)
                                print(target)

        self.sock.close()

    def mainLoop(self):
        pygame.init()
        screen = pygame.display.set_mode((800, 800))

        target = self.target5

        self._send_zero_msg()
        np.set_printoptions(precision=2)

        val = 1 / 3.
        running = True
        i = 0
        prev_p = np.zeros(16)
        while running:
            keys = pygame.key.get_pressed()
            shift = keys[pygame.K_LSHIFT]
            ctrl = keys[pygame.K_LCTRL]
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.KEYDOWN:

                    if event.key == pygame.K_a:
                        target = self.target1
                        break
                    elif event.key == pygame.K_s:
                        target = self.target2
                        break
                    elif event.key == pygame.K_d:
                        target = self.target3
                        break
                    elif event.key == pygame.K_f:
                        target = self.target4
                        break
                    elif event.key == pygame.K_g:
                        target = self.target5
                        break

                    elif event.key == pygame.K_MINUS:
                        val /= 2
                        print(val)
                    elif event.key == pygame.K_EQUALS:
                        val *= 2
                        print(val)

                    for i in range(8):
                        if event.key == pygame.K_1 + i:
                            if shift:
                                target[i + 8] += -val if ctrl else val
                                print(target)
                            else:
                                target[i] += -val if ctrl else val
                                print(target)
            readied, _, _ = select.select([self.sock], [], [])
            #msg = self.sock.recv(32)
            msg = self._recv_all(32)
            p = self._msg_to_pos(msg)

            prev_p = p

            self._send_target_msg(target, p)
            i += 1
            if i % 1 == 0:
                print(p - prev_p)
        self.sock.close()

if __name__ == '__main__':
    hand = HYHandController()
    hand.connect()
    hand.control()
