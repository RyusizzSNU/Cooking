from Instruction import Instruction, Param
import time
class Take(Instruction):
    def __init__(self):
        self.display_name = 'Take'
        self.function = self.take
        self.cursor = 0
        self.params = [Param('Object', {
                    'Oil Bowl' : 'oil_bowl',
                    # 'Onion' : 'onion',
                    'Kettle Handle' : 'kettle_handle',
                    'Pepper Bowl' : 'pepper_bowl',
                    'Ladle Handle' : 'ladle_handle',
                    'Noodle' : 'noodle',
                    'Spam' : 'spam',
                    #'Spam' : 'spam',
                    #'Board Handle' : 'board_handle',
                    'Knife Handle' : 'knife_handle',
                    'Pan Handle' : 'pan_handle_handle',
                    #'Paddle Handle' : 'paddle_handle',
                    #'Switch' : 'switch',
            })
        ]

    def take(self, agent, obj):
        # Different objects require different arm, starting position, axis align, gripper/hand action

        if obj == 'rice_bowl' or obj == 'salt_bowl' or obj == 'oil_bowl':
            side, view, align_axis_from, align_axis_to = 'left', 'top', 0, [0, 0, 1]
            action = 1
        elif obj == 'spam' :
            side, view, align_axis_from, align_axis_to = 'left', 'top', 1, [0, 0, -1]
            action = 0.6
        elif obj == 'onion' or obj == 'carrot':
            side, view, align_axis_from, align_axis_to = 'left', 'top', 0, [0, 0, -1]
            action = 1
        elif obj == 'paddle_handle':
            side, view, align_axis_from, align_axis_to = 'right', 'top3', None, None
            action = 'grab'
        elif obj == 'knife_handle' or obj == 'paddle_handle':
            side, view, align_axis_from, align_axis_to = 'right', 'top3', None, None
            action = 'grab2'

        elif obj == 'ladle_handle' :
            side, view, align_axis_from, align_axis_to = 'right', 'top2', None, None
            action = 'grab'
        elif obj == 'pan_handle_handle':
            side, view, align_axis_from, align_axis_to = 'left', 'top', None, None
            action = 1
        elif obj == 'kettle_handle' :
            side, view, align_axis_from, align_axis_to = 'left', 'top', None, None
            action = 1
        elif obj == 'pepper_bowl' :
            side, view, align_axis_from, align_axis_to = 'left', 'top3', None, None
            action = 1
        elif obj == 'noodle' :
            side, view, align_axis_from, align_axis_to = 'left', 'top5', None, None
            action = 1
        elif obj == 'ham' :
            side, view, align_axis_from, align_axis_to = 'left', 'top4', None, None
            action = 1


        agent.idle(side, view=view, start_closed=True)

        #jesnk added
        # if obj=='knife_handle' or obj=='ladle_handle' :
        #    agent.movej('right', [-2.2607, -3.4469, 1.5627, -2.39, 0.2098,1.7532])

        agent.reach(side, obj, align_axis_from=align_axis_from, align_axis_to=align_axis_to)

        trajectory = [side]
        #if obj=='knife_handle' :
        #    agent.moveD('right',[0,0.02,0],relative=True)
        #    trajectory.append(agent.getj(side))

        if side == 'left':
            agent.gripper_action(action)
        else:
            agent.hand_action(action)
        if side == 'left':
            trajectory.append(agent.getj(side))
            if obj != 'pan_handle_handle':
                agent.moveD(side, [0, 0, 0.3], relative=True)
                trajectory.append(agent.getj(side))
        else:
            if obj == 'knife_handle' :
                trajectory.append(agent.getj(side))
                agent.moveD(side, [0, 0, 0.03], relative=True)
                trajectory.append(agent.getj(side))
                raw_input()
                agent.moveD(side, [0, -0.09, 0], relative=True)
                trajectory.append(agent.getj(side))
                agent.moveD(side, [-0.06, 0, 0], relative=True)
                trajectory.append(agent.getj(side))
            else : # ladle_handle
                trajectory.append(agent.getj(side))
                agent.moveD(side, [0, 0, 0.02], relative=True)
                raw_input()
                trajectory.append(agent.getj(side))
                agent.moveD(side, [0.05, -0.05, 0], relative=True)
                trajectory.append(agent.getj(side))
                agent.moveD(side, [0, -0.1, 0.1], relative=True)
                trajectory.append(agent.getj(side))
        agent.memorize_object_trajectory(obj, trajectory)
