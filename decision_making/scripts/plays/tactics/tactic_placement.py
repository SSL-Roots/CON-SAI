
from pi_trees_lib.pi_trees_lib import *

from skills.observations import CanPass, IsLooking, IsClose, BallIsInField, HasBall
from skills.adjustments import WithKick, WithDribble, NoBallAvoidance
from skills.dynamic_drive import DynamicDrive
from skills.turn_off import TurnOff

from tactic_receive import TacticReceive
from tactic_interpose import TacticInterpose

import sys, os
sys.path.append(os.pardir)
from coordinate import Coordinate


class TacticPlacement(Selector):
    def __init__(self, name, my_role):
        super(TacticPlacement, self).__init__(name)

        self.add_child(TacticReceive("TacticReceive", my_role))
        self.add_child(Placement("Placement", my_role))


class Placement(MemorylessSequence):
    def __init__(self, name, my_role):
        super(Placement, self).__init__(name)

        REMOVE = ParallelOne("REMOVE")
        REMOVE.add_child(BallIsInField("BallIsInField"))
        REMOVE.add_child(Remove("Remove", my_role))
        self.add_child(REMOVE)

        PASS = ParallelOne("PASS")
        
        PASS.add_child(IsClose("IsClose", "DesignatedPosition", "Ball", 0.5))
        PASS.add_child(Pass("Pass", my_role))
        self.add_child(PASS)

        DRIBBLE = ParallelOne("DRIBBLE")
        DRIBBLE.add_child(IsClose("IsClose_", "DesignatedPosition", "Ball", 0.15))
        DRIBBLE.add_child(Dribble("Dribble", my_role))
        self.add_child(DRIBBLE)

        # self.add_child(TurnOff("TurnOff", my_role))
        self.add_child(Push("Push", my_role))

class Remove(MemorylessSequence):
    def __init__(self, name, my_role):
        super(Remove, self).__init__(name)

        self.add_child(WithDribble("WithDribble", my_role))

        APPROACH = ParallelOne("APPROACH")
        APPROACH.add_child(TacticInterpose("TacticInterpose", my_role,
            "CONST_CENTER", "Ball", from_dist = 0.09))
        APPROACH.add_child(HasBall('HasBall', my_role))
        self.add_child(APPROACH)

        self.add_child(TacticInterpose("TacticInterposeBack", my_role,
            my_role, "Ball", to_dist = -0.05))

class Pass(MemorylessSequence):
    def __init__(self, name, my_role):
        super(Pass, self).__init__(name)

        # self.add_child(NoBallAvoidance('NoBallAvoidance', my_role))

        coord = Coordinate()
        coord.set_approach_to_shoot(my_role, target='DesignatedPosition')

        self.add_child(DynamicDrive('drive_to_pass', my_role, coord))
        self.add_child(IsLooking('IsLooking_pass', my_role, 'DesignatedPosition'))
        self.add_child(WithKick('WithKick_pass', my_role, 
            target_name = 'DesignatedPosition', is_pass = True))

class Dribble(MemorylessSequence):
    def __init__(self, name, my_role):
        super(Dribble, self).__init__(name)

        # self.add_child(NoBallAvoidance('NoBallAvoidance', my_role))

        coord = Coordinate()
        coord.set_approach_to_shoot(my_role, target='DesignatedPosition')

        self.add_child(WithDribble('WithDribble', my_role))
        self.add_child(DynamicDrive('drive_to_shoot', my_role, coord,
            always_running = True))

class Push(MemorylessSequence):
    def __init__(self, name, my_role):
        super(Push, self).__init__(name)

        # self.add_child(NoBallAvoidance('NoBallAvoidance', my_role))

        coord = Coordinate()
        coord.set_approach_to_shoot(my_role, target='DesignatedPosition')

        self.add_child(DynamicDrive('drive_to_shoot', my_role, coord,
            always_running = True))
