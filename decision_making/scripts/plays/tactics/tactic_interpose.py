
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.drive_to_dynamic_target import DriveToDynamicTarget

sys.path.append(os.pardir)
from coordinate import Coordinate

class TacticInterpose(ParallelAll):
    def __init__(self, name, my_role, to_dist=None, from_dist=None):
        super(TacticInterpose, self).__init__(name)

        self._coordinate = Coordinate()
        self._coordinate.set_interpose(to_dist = to_dist, from_dist = from_dist)
        self.add_child(DriveToDynamicTarget('DriveToDynamicTarget', my_role, self._coordinate))
