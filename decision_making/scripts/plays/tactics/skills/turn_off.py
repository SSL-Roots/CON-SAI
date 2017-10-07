
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import sys,os
sys.path.append(os.pardir)
from world_model import WorldModel


class TurnOff(Task):
    def __init__(self, name, my_role):
        super(TurnOff, self).__init__(name)
        self.my_role = my_role

    def run(self):
        WorldModel.commands[self.my_role].set_target_velocity(0.0, 0.0, 0.0)

        return TaskStatus.RUNNING


