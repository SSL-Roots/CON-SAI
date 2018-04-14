
from pi_trees_lib.pi_trees_lib import *

from skills.turn_off import TurnOff
from skills.observations import CanReceive
from skills.dynamic_drive import DynamicDrive

import sys, os
sys.path.append(os.pardir)
from coordinate import Coordinate

class TacticAttacker(Selector):
    def __init__(self, name, my_role):
        super(TacticAttacker, self).__init__(name)

        self.add_child(Receive('Receive', my_role))
        self.add_child(TurnOff('TurnOff', my_role))

class Receive(MemorylessSequence):
    def __init__(self, name, my_role):
        super(Receive, self).__init__(name)


        self.add_child(CanReceive('CanReceive', my_role))

        coord = Coordinate()
        coord.set_receive_ball(my_role)

        self.add_child(DynamicDrive('drive_to_receive', my_role, coord,
            always_running = True))
        
