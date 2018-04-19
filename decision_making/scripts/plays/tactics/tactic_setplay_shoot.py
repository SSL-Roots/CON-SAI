
from pi_trees_lib.pi_trees_lib import *

from skills.turn_off import TurnOff
from skills.dynamic_drive import DynamicDrive
from skills.observations import CanShoot, CanPass
from skills.adjustments import WithKick, NoBallAvoidance

import sys, os
sys.path.append(os.pardir)
from coordinate import Coordinate

class TacticSetplayShoot(Selector):
    def __init__(self, name, my_role):
        super(TacticSetplayShoot, self).__init__(name)

        self.add_child(Shoot('Shoot', my_role))
        self.add_child(Pass('Pass', my_role))
        self.add_child(TurnOff('TurnOff', my_role))


class Shoot(MemorylessSequence):
    def __init__(self, name, my_role):
        super(Shoot, self).__init__(name)

        self.add_child(CanShoot('CanShoot', my_role))
        self.add_child(WithKick('WithKick', my_role))
        self.add_child(NoBallAvoidance('NoBallAvoidance', my_role))

        coord = Coordinate()
        coord.set_approach_to_shoot(my_role, target='ShootTarget')

        self.add_child(DynamicDrive('drive_to_shoot', my_role, coord,
            always_running = True))

class Pass(MemorylessSequence):
    def __init__(self, name, my_role):
        super(Pass, self).__init__(name)

        self.add_child(CanPass('CanPass', my_role))
        self.add_child(WithKick('WithKick', my_role, kick_power=3.0))
        self.add_child(NoBallAvoidance('NoBallAvoidance', my_role))

        coord = Coordinate()
        coord.set_approach_to_shoot(my_role, target='PassTarget')

        self.add_child(DynamicDrive('drive_to_pass', my_role, coord,
            always_running = True))
