
from pi_trees_lib.pi_trees_lib import *

from skills.observations import CanShoot, IsLooking
from skills.adjustments import WithDribble, NoBallAvoidance
from skills.dynamic_drive import DynamicDrive

import sys, os
sys.path.append(os.pardir)
from coordinate import Coordinate

        
class TacticDribble(MemorylessSequence):
    def __init__(self, name, my_role):
        super(TacticDribble, self).__init__(name)

        self.add_child(NoBallAvoidance('NoBallAvoidance', my_role))
        self.add_child(WithDribble('WithDribble', my_role))

        coord = Coordinate()
        coord.set_approach_to_shoot(my_role, target='ShootTarget')

        self.add_child(DynamicDrive('drive_to_dribble', my_role, coord,
            always_running = True))

