
from pi_trees_lib.pi_trees_lib import *

from skills.turn_off import TurnOff

class TacticAttacker(MemorylessSequence):
    def __init__(self, name, my_role):
        super(TacticAttacker, self).__init__(name)

        self.add_child(TurnOff('TurnOff', my_role))
