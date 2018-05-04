
from pi_trees_lib.pi_trees_lib import *

from skills.observations import CanReflectShoot, IsLooking
from skills.adjustments import WithKick, WithDribble, NoBallAvoidance
from skills.dynamic_drive import DynamicDrive

import sys, os
sys.path.append(os.pardir)
from coordinate import Coordinate

        
class TacticReflectShoot(MemorylessSequence):
    def __init__(self, name, my_role):
        super(TacticReflectShoot, self).__init__(name)

        target = 'CONST_THEIR_GOAL'

        self.add_child(CanReflectShoot('CanReflectShoot', my_role, target))
        self.add_child(NoBallAvoidance('NoBallAvoidance', my_role))
        self.add_child(WithDribble('WithDribble', my_role))

        coord = Coordinate()
        coord.set_reflect(my_role, target)

        self.add_child(DynamicDrive('drive_to_shoot', my_role, coord))
        self.add_child(WithKick('WithKick', my_role))

