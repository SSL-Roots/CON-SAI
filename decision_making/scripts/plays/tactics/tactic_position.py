
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.drive_to_target import DriveToTarget

class TacticPosition(ParallelAll):
    def __init__(self, name, my_role, x, y, theta):
        super(TacticPosition, self).__init__(name)

        self.add_child(DriveToTarget('DriveToTarget', my_role, x, y, theta))
