
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.dynamic_drive import DynamicDrive
from skills.observations import BallKicked
from skills.adjustments import WithChip, NoBallAvoidance, NoDefenceAreaAvoidance

sys.path.append(os.pardir)
from coordinate import Coordinate


class TacticClear(Sequence):
    def __init__(self, name, my_role):
        super(TacticClear, self).__init__(name)

        coord = Coordinate()
        coord.set_interpose(from_dist = 0.05)

        DRIVE = ParallelOne('DRIVE')
        DRIVE.add_child(DynamicDrive('drive_to_ball', my_role, coord))
        DRIVE.add_child(NoBallAvoidance('NoBallAvoidance', my_role))
        DRIVE.add_child(NoDefenceAreaAvoidance('NoDefenceAreaAvoidance', my_role))

        SHOOT = ParallelOne('SHOOT')
        SHOOT.add_child(DynamicDrive('drive_to_shoot', my_role, coord, 
            always_running = True))
        SHOOT.add_child(WithChip('WithChip', my_role, 3.0))
        SHOOT.add_child(NoBallAvoidance('NoBallAvoidance', my_role))
        SHOOT.add_child(NoDefenceAreaAvoidance('NoDefenceAreaAvoidance', my_role))
        SHOOT.add_child(BallKicked('BallKicked'))

        self.add_child(DRIVE)
        self.add_child(SHOOT)
