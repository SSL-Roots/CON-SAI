
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import sys,os
sys.path.append(os.pardir)
from world_model import WorldModel

class DriveToTarget(Task):
    def __init__(self, name, my_role, x, y, theta):
        super(DriveToTarget, self).__init__(name)
        self._my_role = my_role
        self._x = x
        self._y = y
        self._theta = theta

    def run(self):
        WorldModel.commands[self._my_role].set_target_pose(self._x, self._y, self._theta, 'map')

        return TaskStatus.RUNNING
