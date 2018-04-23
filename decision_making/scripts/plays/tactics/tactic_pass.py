
from pi_trees_lib.pi_trees_lib import *

from skills.observations import CanPass, IsLooking
from skills.adjustments import WithKick, NoBallAvoidance
from skills.dynamic_drive import DynamicDrive

import sys, os
sys.path.append(os.pardir)
from coordinate import Coordinate


class TacticPass(MemorylessSequence):
    def __init__(self, name, my_role):
        super(TacticPass, self).__init__(name)

        self.add_child(CanPass('CanPass', my_role))
        self.add_child(NoBallAvoidance('NoBallAvoidance', my_role))

        coord = Coordinate()
        coord.set_approach_to_shoot(my_role, target='PassTarget')

        self.add_child(DynamicDrive('drive_to_pass', my_role, coord))
        self.add_child(IsLooking('IsLooking_pass', my_role, 'PassTarget'))
        self.add_child(WithKick('WithKick_pass', my_role, kick_power=3.0))
