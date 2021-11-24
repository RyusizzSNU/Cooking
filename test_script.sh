from agent import Agent
import numpy as np
import math
import utils

agent = Agent()
a = math.radians(30)
b = math.radians(45)
rot0 = np.array([[1, 0, 0], [0, -math.cos(a), -math.sin(a)], [0, math.sin(a), -math.cos(a)]])
rot0 = np.array([[math.cos(b), -math.sin(b), 0], 
		[-math.sin(b) * math.cos(a), -math.cos(a) * math.cos(b), -math.sin(a)], 
		[math.sin(b) * math.sin(a), math.cos(b) * math.sin(a), -math.cos(a)]])

agent.ready('right')
agent.moveD('right', utils.affine_to_tcp(dcm=rot0, t=[0.87, 0.38, 0.367]))
