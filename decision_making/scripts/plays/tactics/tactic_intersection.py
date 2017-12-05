
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.dynamic_drive import DynamicDrive
from consai_msgs.msg import Pose

sys.path.append(os.pardir)
from coordinate import Coordinate

class TacticIntersection(ParallelAll):
    def __init__(self, name, my_role,  base='CONST_OUR_GOAL', target='Ball',
            pose1=Pose(-2,3,0), pose2=Pose(-2,-3,0)):
        super(TacticIntersection, self).__init__(name)

        self._coordinate = Coordinate()
        self._coordinate.set_intersection(base=base, target=target,
                pose1=pose1, pose2=pose2)
        self.add_child(DynamicDrive('DynamicDrive', my_role, self._coordinate,
            always_running = True))
