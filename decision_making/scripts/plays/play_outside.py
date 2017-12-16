
from play_base import Play

from tactics.tactic_halt import TacticHalt

class PlayOutside(Play):
    def __init__(self):
        super(PlayOutside, self).__init__('PlayOutside')

        self.applicable = "BALL_IN_OUTSIDE"
        self.done_aborted = "BALL_IN_OUTSIDE"

        for i in range(6):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticHalt("TacticHalt", self.roles[i].my_role))
