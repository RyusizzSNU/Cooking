from agent import Agent
from Module.food import manipulate_food, turnback_food
from Module.knife import manipulate_knife, turnback_knife
from Module.paddle import manipulate_paddle
from Module.board import manipulate_board
from Module.switch import manipulate_switch
from Module.bowl import manipulate_bowl
from overlay_recipe import RecipeController
from Module.segmentation import SegmentationController
import time

joint_knife = range(6)
def place_chop_and_put_food(agent, seg_controller, food, pickup_knife=True, turnback=True):
    joint_food = range(6)
    manipulate_food(agent, food, joint_food)

    agent.idle('right', 'board')
    seg_controller.stimulate(True)
    time.sleep(1)
    seg_controller.stimulate(False)

    manipulate_knife(agent, food, pickup_knife)
    agent.moveD('right', [0, 0, 0.2], relative=True, wait=False)
    agent.open_gripper()
    agent.moveD('left', [0, 0, 0.3], relative=True)
    raw_input()
    # turnback_food(agent, joint_food[:])
    # Raise up the board and put the food into the pan
    manipulate_board(agent)

    # Turn back the knife
    if turnback:
        turnback_knife(agent)
    else:
        agent.idle('right', view='board')

agent = Agent()
recipe_controller = RecipeController()
seg_controller = SegmentationController(635, [(75,220), (240,340)])
agent.ready('left')
time.sleep(2)
agent.ready('right')

# Move both hands to each idle position
agent.idle('left', view='top', start_closed=True)
agent.idle('right', view='top2', start_closed=True)

# Place the food and chop, and put it into the pan
recipe_controller.set_index(0)
place_chop_and_put_food(agent, seg_controller, 'spam', pickup_knife=True, turnback=False)
raw_input()

# Turn on the switch
recipe_controller.set_index(1)
manipulate_switch(agent, clockwise=True)

# Pick up the oil bowl and pour into the pan
recipe_controller.set_index(2)
manipulate_bowl(agent, 'oil_bowl')
raw_input()
recipe_controller.set_index(3)
place_chop_and_put_food(agent, seg_controller, 'onion', pickup_knife=False, turnback=False)
raw_input()

recipe_controller.set_index(4)
place_chop_and_put_food(agent, seg_controller, 'carrot', pickup_knife=False, turnback=True)
raw_input()

# Stir with the paddle
recipe_controller.set_index(5)
manipulate_paddle(agent)
raw_input()

# Pick up the oil bowl and pour into the pan
recipe_controller.set_index(6)
manipulate_bowl(agent, 'rice_bowl')

recipe_controller.set_index(7)
manipulate_bowl(agent, 'salt_bowl')
raw_input()

# Stir with the paddle
recipe_controller.set_index(8)
manipulate_paddle(agent)
raw_input()

# Turn off the switch
recipe_controller.set_index(9)
manipulate_switch(agent, clockwise=False)