
from play_base import Play

from tactics.tactic_halt import TacticHalt

class PlayHalt(Play):
    def __init__(self):
        super(PlayHalt, self).__init__('PlayHalt')

        self.applicable = "HALT"
        self.done_aborted = "HALT"

        for i in range(6):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticHalt("TacticHalt", self.roles[i].my_role))
