
from play_base import Play

from tactics.tactic_position import TacticPosition
from tactics.tactic_keep import TacticKeep
import constants

class PlayTheirPreKickoff(Play):
    def __init__(self):
        super(PlayTheirPreKickoff, self).__init__('PlayTheirPreKickoff')

        self.applicable = "THEIR_PRE_KICKOFF"
        self.done_aborted = "THEIR_PRE_KICKOFF"

        keep_x = -constants.FieldHalfX + constants.RobotRadius * 2.0
        self.roles[0].loop_enable = True
        self.roles[0].behavior.add_child(
                TacticKeep('TacticKeep', self.roles[0].my_role, keep_x = keep_x,
                    range_high = constants.GoalHalfSize,
                    range_low = -constants.GoalHalfSize)
                )

        for i in range(1,6):
            x = -2.0
            y = 1.5 - 0.5*i
            theta = 0

            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticPosition('TacticPosition', self.roles[i].my_role,
                        x, y ,theta))
