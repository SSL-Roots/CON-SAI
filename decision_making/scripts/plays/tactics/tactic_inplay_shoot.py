
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.dynamic_drive import DynamicDrive
from skills.observations import BallKicked
from skills.adjustments import WithKick, NoNavigation

sys.path.append(os.pardir)
from coordinate import Coordinate


class TacticInplayShoot(Sequence):
    def __init__(self, name, my_role):
        super(TacticInplayShoot, self).__init__(name)

        coord = Coordinate()
        coord.set_approach_to_shoot(my_role, target='CONST_THEIR_GOAL')

        DRIVE = ParallelOne('DRIVE')
        DRIVE.add_child(DynamicDrive('drive_to_ball', my_role, coord))
        DRIVE.add_child(NoNavigation('NoNavigation', my_role))

        SHOOT = ParallelOne('SHOOT')
        SHOOT.add_child(DynamicDrive('drive_to_shoot', my_role, coord, 
            always_running = True))
        SHOOT.add_child(WithKick('WithKick', my_role))
        SHOOT.add_child(NoNavigation('NoNavigation', my_role))
        SHOOT.add_child(BallKicked('BallKicked'))

        self.add_child(DRIVE)
        self.add_child(SHOOT)
