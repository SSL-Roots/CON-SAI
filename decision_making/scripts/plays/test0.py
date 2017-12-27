
from play_base import Play

from tactics.tactic_halt import TacticHalt

class Test0(Play):
    def __init__(self):
        super(Test0, self).__init__('Test0')

        self.applicable = "TEST0"
        self.done_aborted = "TEST0"

        for i in range(6):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticHalt("TacticHalt", self.roles[i].my_role))
