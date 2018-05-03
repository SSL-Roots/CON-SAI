
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.dynamic_drive import DynamicDrive
from skills.adjustments import NoDefenceAreaAvoidance

from tactic_receive import TacticReceive
from tactic_look_intersection import TacticLookIntersection

from consai_msgs.msg import Pose

sys.path.append(os.pardir)
from coordinate import Coordinate


class TacticGoalie(ParallelAll):
    def __init__(self, name, my_role, target='Ball', keep_x=None, 
            range_high=10.0, range_low= -10.0):
        super(TacticGoalie, self).__init__(name)

        coord1 = Coordinate()
        coord1.set_receive_ball(my_role)

        coord2 = Coordinate()
        coord2.set_keep_x(keep_x, target, range_high, range_low)

        DRIVE = Selector('DRIVE')
        DRIVE.add_child(DynamicDrive('drive_to_receive', my_role, coord1,
            always_running = True))
        DRIVE.add_child(DynamicDrive('keep_drive', my_role, coord2,
            always_running = True))

        self.add_child(NoDefenceAreaAvoidance('NoDefenceAreaAvoidance', my_role))
        self.add_child(DRIVE)

class TacticNewGoalie(Selector):
    def __init__(self, name, my_role, target='Threat_0',
            pose1=Pose(-2, 3, 0), pose2=Pose(-2, -3, 0)):
        super(TacticNewGoalie, self).__init__(name)

        self.add_child(TacticReceive('TacticReceive', my_role))
        self.add_child(TacticLookIntersection('TacticLookIntersection',
            my_role, target, pose1, pose2))



