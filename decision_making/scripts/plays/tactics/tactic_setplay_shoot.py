
from pi_trees_lib.pi_trees_lib import *

from tactic_receive import TacticReceive
from tactic_shoot import TacticShoot
from tactic_pass import TacticPass

from skills.turn_off import TurnOff

class TacticSetplayShoot(Selector):
    def __init__(self, name, my_role):
        super(TacticSetplayShoot, self).__init__(name)

        # self.add_child(TacticShoot('TacticShoot', my_role))
        self.add_child(TacticPass('TacticPass', my_role))
        self.add_child(TurnOff('TurnOff', my_role))

