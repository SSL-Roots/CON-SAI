
from play_base import Play
from play_stop import PlayStop

from tactics.tactic_goalie import TacticGoalie
from tactics.tactic_interpose import TacticInterpose
from tactics.tactic_position import TacticPosition
import constants

class PlayTheirIndirect(PlayStop):
    def __init__(self):
        super(PlayTheirIndirect, self).__init__('PlayTheirIndirect')

        self.applicable = "THEIR_INDIRECT"
        self.done_aborted = "THEIR_INDIRECT"

        for i in range(2,6):
            x = - constants.PenaltyX + 0.3
            y = 0.45 - 0.3*(i-2)
            theta = 0

            self.roles[i].clear_behavior()
            self.roles[i].behavior.add_child(
                    TacticPosition('TacticPosition', self.roles[i].my_role,
                        x, y ,theta))
