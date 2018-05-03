
from pi_trees_lib.pi_trees_lib import *

from skills.observations import CanReceive
from skills.adjustments import WithKick, NoBallAvoidance, WithDribble
from skills.dynamic_drive import DynamicDrive

import sys, os
sys.path.append(os.pardir)
from coordinate import Coordinate


class TacticReceive(MemorylessSequence):
    def __init__(self, name, my_role):
        super(TacticReceive, self).__init__(name)


        self.add_child(CanReceive('CanReceive', my_role))
        self.add_child(NoBallAvoidance('NoBallAvoidance', my_role))
        self.add_child(WithDribble('WithDribble', my_role))

        coord = Coordinate()
        coord.set_receive_ball(my_role)

        self.add_child(DynamicDrive('drive_to_receive', my_role, coord,
            always_running = True))
        
