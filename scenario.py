from agent import Agent
from Module.food import manipulate_food, turnback_food
from Module.knife import manipulate_knife, turnback_knife
from Module.paddle import manipulate_paddle
from Module.board import manipulate_board
from Module.switch import manipulate_switch
from Module.bowl import manipulate_bowl

agent = Agent()
agent.ready('left')
agent.ready('right')

agent.idle('left', view='top', start_closed=True, wait=False)
agent.idle('right', view='top2', start_closed=True)

j1 = manipulate_food(agent, 'spam')
j2 = manipulate_knife(agent)

agent.moveD('right', [0, 0, 0.25], relative=True, wait=False)
agent.open_gripper()
agent.moveD('left', [0, 0, 0.3], relative=True)
#turnback_food(agent, j1)

manipulate_board(agent)

turnback_knife(agent, j2)

agent.moveD('left', [-0.2, 0, 0], relative=True)

agent.open_gripper()
agent.idle('right', view='top2')
agent.close_gripper()

manipulate_switch(agent)

agent.moveD('left', [-0.2, 0, 0], relative=True)
manipulate_bowl(agent, 'oil_bowl')
