
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.dynamic_drive import DynamicDrive

sys.path.append(os.pardir)
from coordinate import Coordinate

class TacticMark(ParallelAll):
    def __init__(self, name, my_role, target, from_dist=0.5):
        super(TacticMark, self).__init__(name)

        self._coordinate = Coordinate()
        self._coordinate.set_interpose(target=target, from_dist = from_dist)
        self.add_child(DynamicDrive('DynamicDrive', my_role, self._coordinate,
            always_running = True))
