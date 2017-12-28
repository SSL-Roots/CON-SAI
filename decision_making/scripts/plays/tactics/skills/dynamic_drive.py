from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import sys,os
sys.path.append(os.pardir)
from world_model import WorldModel
from coordinate import Coordinate


class DynamicDrive(Task):
    def __init__(self, name, my_role, coordinate, always_running=False):
        super(DynamicDrive, self).__init__(name)
        self._my_role = my_role
        self._coordinate = coordinate
        self._always_running = always_running

    def run(self):

        if self._coordinate.update() == False:
            return TaskStatus.FAILURE

        pose = self._coordinate.pose

        WorldModel.commands[self._my_role].set_target_pose(pose.x, pose.y, pose.theta, 'map')

        if not self._always_running and \
                self._coordinate.is_arrived(self._my_role):
            return TaskStatus.SUCCESS

        return TaskStatus.RUNNING
