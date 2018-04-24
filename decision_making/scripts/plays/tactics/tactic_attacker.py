
from pi_trees_lib.pi_trees_lib import *

from tactic_receive import TacticReceive
from tactic_shoot import TacticShoot
from tactic_pass import TacticPass
from tactic_dribble import TacticDribble

class TacticAttacker(Selector):
    def __init__(self, name, my_role):
        super(TacticAttacker, self).__init__(name)

        self.add_child(TacticReceive('TacticReceive', my_role))
        self.add_child(TacticShoot('TacticShoot', my_role))
        self.add_child(TacticPass('TacticPass', my_role))
        self.add_child(TacticDribble('TacticDribble', my_role))

