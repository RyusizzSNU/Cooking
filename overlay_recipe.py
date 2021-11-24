#!/usr/bin/env python

import rospy
from jsk_rviz_plugins.msg import OverlayMenu

class RecipeController(object):
    def __init__(self):
        self.recipe = ["No Recipe"]
        '''self.recipe = [
          "1. chop the spam",
        #  "put the potato",
          "2. turn on the induction",
          "3. put the oil",
          "4. chop the onion",
          "5. chop the carrot",
          "6. fry for 30 sec",
            # "7. fry for 10 sec",
          "7. put the rice",
          "8. put the salt",
          "9. fry for 30 sec",
          "10. turn off the induction"
        ]'''
        self.p = rospy.Publisher("recipe", OverlayMenu, queue_size=1)
        # self.r = rospy.Rate(10)
        # self.current_index = 0
        self.menu = OverlayMenu()
        self.menu.title = "Recipe"
    #  menu.menus = ["John Lennon", "Paul McCartney", "George Harrison",
    #                "Ringo Starr"]
        self.menu.menus = self.recipe
        self.menu.current_index = 0 #(counter/10) % len(menu.menus)
    #  if counter % 100 == 0:
    #    menu.action = OverlayMenu.ACTION_CLOSE
        self.menu.fg_color.r = 1.0
        self.menu.fg_color.g = 1.0
        self.menu.fg_color.b = 1.0
        self.menu.fg_color.a = 1.0
        self.menu.bg_color.r = 0.0
        self.menu.bg_color.g = 0.0
        self.menu.bg_color.b = 0.0
        self.menu.bg_color.a = 1.0
        self.p.publish(self.menu)

    def set_instructions(self, instructions):
        self.recipe = []
        for i in range(len(instructions)):
            inst = instructions[i]
            self.recipe.append("%s. %s"%(i + 1, inst[0] + ' ' + inst[1]))
        self.menu.menus = self.recipe
        self.set_index(0)

    def set_index(self, index):
        self.menu.current_index = index
        self.p.publish(self.menu)

if __name__ == '__main__':
    rospy.init_node("recipe")
    recipe_control = RecipeController()
    counter = 0
    while not rospy.is_shutdown():
        recipe_control.set_index(counter/10000 % 11)
        counter += 1
