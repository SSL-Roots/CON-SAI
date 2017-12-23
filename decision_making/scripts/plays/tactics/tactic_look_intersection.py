
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.dynamic_drive import DynamicDrive
from consai_msgs.msg import Pose

sys.path.append(os.pardir)
from coordinate import Coordinate

class TacticLookIntersection(ParallelAll):
    def __init__(self, name, my_role, target='Enemy_1',
            pose1=Pose(-2,3,0), pose2=Pose(-2,-3,0)):
        super(TacticLookIntersection, self).__init__(name)

        self._coordinate = Coordinate()
        self._coordinate.set_look_intersection(target=target,
                pose1=pose1, pose2=pose2)
        self.add_child(DynamicDrive('DynamicDrive', my_role, self._coordinate,
            always_running = True))
