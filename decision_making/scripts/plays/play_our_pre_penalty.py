
from play_base import Play

from tactics.tactic_goalie import TacticGoalie
from tactics.tactic_interpose import TacticInterpose
from tactics.tactic_position import TacticPosition
from consai_msgs.msg import Pose
import constants
import math

class PlayOurPrePenalty(Play):
    def __init__(self):
        super(PlayOurPrePenalty, self).__init__('PlayOurPrePenalty')

        self.applicable = "OUR_PRE_PENALTY"
        self.done_aborted = "OUR_PRE_PENALTY"

        keep_x = -constants.FieldHalfX + constants.RobotRadius * 2.0
        self.roles[0].loop_enable = True
        self.roles[0].behavior.add_child(
                TacticGoalie('TacticGoalie', self.roles[0].my_role, keep_x=keep_x,
                    range_high = constants.GoalHalfSize,
                    range_low = -constants.GoalHalfSize)
                )

        self.roles[1].loop_enable = True
        self.roles[1].behavior.add_child(
                TacticInterpose('TacticInterpose', self.roles[1].my_role, 
                    from_dist = 0.2)
                )

        for i in range(2,6):
            x = -3.0 + 0.3*i
            y = 2.0
            theta = -math.pi * 0.5

            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticPosition('TacticPosition', self.roles[i].my_role,
                        x, y ,theta))
