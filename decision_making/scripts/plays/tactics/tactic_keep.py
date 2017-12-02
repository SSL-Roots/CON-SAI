
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.dynamic_drive import DynamicDrive

sys.path.append(os.pardir)
from coordinate import Coordinate

class TacticKeep(ParallelAll):
    def __init__(self, name, my_role, to_dist=None, target='Ball', keep_x=None, keep_y=None):
        super(TacticKeep, self).__init__(name)

        self._coordinate = Coordinate()
        if keep_x:
            self._coordinate.set_keep_x(keep_x=keep_x, target=target, 
                    range_y_high=1.5, range_y_low=-1.5)
        else:
            self._coordinate.set_keep_y(keep_y=keep_y, target=target,
                    range_x_high=-0.5, range_x_low=-2.5)


        self.add_child(DynamicDrive('DynamicDrive', my_role, self._coordinate,
            always_running = True))
