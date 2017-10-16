
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import sys,os
sys.path.append(os.pardir)
from world_model import WorldModel
from coordinate import Coordinate

class DriveToDynamicTarget(Task):
    def __init__(self, name, my_role, coordinate):
        super(DriveToDynamicTarget, self).__init__(name)
        self._my_role = my_role
        self._coordinate = coordinate

    def run(self):
        self._coordinate.update()

        x, y, theta = self._coordinate.pose

        WorldModel.commands[self._my_role].set_target_pose(x, y, theta, 'map')

        return TaskStatus.RUNNING
