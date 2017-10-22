
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.drive_to_dynamic_target import DriveToDynamicTarget

sys.path.append(os.pardir)
from coordinate import Coordinate

class TacticMark(ParallelAll):
    def __init__(self, name, my_role, target, from_dist=0.5):
        super(TacticMark, self).__init__(name)

        self._coordinate = Coordinate()
        self._coordinate.set_interpose(target=target, from_dist = from_dist)
        self.add_child(DriveToDynamicTarget('DriveToDynamicTarget', my_role, self._coordinate))
