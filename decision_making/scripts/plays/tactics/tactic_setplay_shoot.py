
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.dynamic_drive import DynamicDrive
from skills.observations import BallKicked
from skills.adjustments import WithKick, NoBallAvoidance

sys.path.append(os.pardir)
from coordinate import Coordinate


class TacticSetplayShoot(Sequence):
    def __init__(self, name, my_role):
        super(TacticSetplayShoot, self).__init__(name)

        coord_1 = Coordinate()
        coord_1.set_interpose(base='Ball', target='CONST_THEIR_GOAL', to_dist = -0.3)
        self.add_child(DynamicDrive('drive_to_ball_back', my_role, coord_1))


        SHOOT = ParallelOne('SHOOT')
        coord_2 = Coordinate()
        coord_2.set_interpose(base='Ball', target='CONST_THEIR_GOAL', to_dist = 0.0)
        SHOOT.add_child(DynamicDrive('drive_to_ball', my_role, coord_2))
        SHOOT.add_child(WithKick('WithKick', my_role))
        SHOOT.add_child(NoBallAvoidance('NoBallAvoidance', my_role))
        SHOOT.add_child(BallKicked('BallKicked'))

        self.add_child(SHOOT)
