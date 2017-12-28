
from play_base import Play

from tactics.tactic_halt import TacticHalt
from tactics.tactic_inplay_shoot import TacticInplayShoot

class Test0(Play):
    def __init__(self):
        super(Test0, self).__init__('Test0')

        self.applicable = "TEST0"
        self.done_aborted = "TEST0"
        self.assignment_type = "CLOSEST_BALL"


        self.roles[0].loop_enable = True
        self.roles[0].behavior.add_child(
                TacticInplayShoot('TacticInplayShoot', self.roles[0].my_role)
                )

        for i in range(1,6):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticHalt("TacticHalt", self.roles[i].my_role))
