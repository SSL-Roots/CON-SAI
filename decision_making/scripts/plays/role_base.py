from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

class Role(object):
    def __init__(self, my_role):
        self.my_role = my_role
        self.behavior = Sequence(my_role) # my_role is like a "Role_0"
        self.loop_enable = False

    
    def clear_behavior(self):
        self.behavior = Sequence(self.my_role)
