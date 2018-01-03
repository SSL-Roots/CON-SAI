
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.dynamic_drive import DynamicDrive

sys.path.append(os.pardir)
from coordinate import Coordinate

class TacticPosition(Selector):
    def __init__(self, name, my_role, x, y, theta, always_running=False):
        super(TacticPosition, self).__init__(name)

        coord = Coordinate()
        coord.set_pose(x, y, theta)

        self.add_child(DynamicDrive('drive_to_pose', my_role, coord,
            always_running = always_running))
