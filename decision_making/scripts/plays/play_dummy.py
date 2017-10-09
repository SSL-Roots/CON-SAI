
from play_base import Play

from tactics.tactic_halt import TacticHalt

class PlayDummy(Play):
    def __init__(self):
        super(PlayDummy, self).__init__('PlayDummy')

        self.timeout = 1.0

        for i in range(6):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticHalt("TacticHalt", self.roles[i].my_role))
