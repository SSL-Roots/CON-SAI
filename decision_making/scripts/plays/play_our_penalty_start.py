
from play_base import Play

from tactics.tactic_goalie import TacticGoalie
from tactics.tactic_setplay_shoot import TacticSetplayShoot
from tactics.tactic_position import TacticPosition
from consai_msgs.msg import Pose
import constants
import math

class PlayOurPenaltyStart(Play):
    def __init__(self):
        super(PlayOurPenaltyStart, self).__init__('PlayOurPenaltyStart')

        self.applicable = "OUR_PENALTY_START"
        self.done_aborted = "OUR_PENALTY_START"

        keep_x = -constants.FieldHalfX + constants.RobotRadius * 2.0
        self.roles[0].loop_enable = True
        self.roles[0].behavior.add_child(
                TacticGoalie('TacticGoalie', self.roles[0].my_role, keep_x=keep_x,
                    range_high = constants.GoalHalfSize,
                    range_low = -constants.GoalHalfSize)
                )

        self.roles[1].loop_enable = True
        self.roles[1].behavior.add_child(
                TacticSetplayShoot('TacticSetplayShoot', self.roles[1].my_role)
                )

        for i in range(2,6):
            x = -3.0 + 0.3*i
            y = 2.0
            theta = -math.pi * 0.5

            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticPosition('TacticPosition', self.roles[i].my_role,
                        x, y ,theta))
