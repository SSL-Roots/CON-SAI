
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import sys,os
sys.path.append(os.pardir)
from world_model import WorldModel


class WithKick(Task):
    def __init__(self, name, my_role, kick_power=6.0):
        super(WithKick, self).__init__(name)

        self._my_role = my_role
        self._kick_power = kick_power

    def run(self):
        WorldModel.commands[self._my_role].set_kick(self._kick_power)

        return TaskStatus.RUNNING


class NoNavigation(Task):
    def __init__(self, name, my_role):
        super(NoNavigation, self).__init__(name)

        self._my_role = my_role

    def run(self):
        WorldModel.commands[self._my_role].navigation_enable = False

        return TaskStatus.RUNNING


class WithChip(Task):
    def __init__(self, name, my_role, kick_power=6.0):
        super(WithChip, self).__init__(name)

        self._my_role = my_role
        self._kick_power = kick_power


    def run(self):
        WorldModel.commands[self._my_role].set_kick(self._kick_power,True)

        return TaskStatus.RUNNING
