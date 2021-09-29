
class Param:
    def __init__(self, display_name, name_value_dict):
        self.display_name = display_name
        self.name_value_dict = name_value_dict
        self.current_name = None

class Action:
    def __init__(self, display_name, function, params):
        self.display_name = display_name
        self.function = function
        self.params = params
        self.cursor = 0

    @property
    def all_params_assigned(self):
        return len(self.params) <= self.cursor

    @property
    def current_param(self):
        if len(self.params) <= self.cursor:
            return None
        return self.params[self.cursor]

    def assign_current_param(self, name):
        self.current_param.current_name = name
        self.cursor += 1

    def unassign_current_param(self):
        self.current_param.current_name = None
        self.cursor -= 1

    def __call__(self, agent):
        params_list = []
        for param in self.params:
            params_list.append(param.name_value_dict[param.current_name])
        self.function(agent, *params_list)