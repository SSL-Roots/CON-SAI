
from play_base import Play

from tactics.tactic_keep import TacticKeep
from tactics.tactic_intersection import TacticIntersection
from tactics.tactic_interpose import TacticInterpose
from tactics.tactic_inplay_shoot import TacticInplayShoot
from consai_msgs.msg import Pose
import constants

class PlayInPlayTheirDefence(Play):
    def __init__(self):
        super(PlayInPlayTheirDefence, self).__init__('PlayInPlayTheirDefence')

        self.applicable = "BALL_IN_THEIR_DEFENCE"
        self.done_aborted = "BALL_IN_THEIR_DEFENCE"
        self.assignment_type = "CLOSEST_BALL"

        keep_x = -constants.FieldHalfX + constants.RobotRadius * 2.0
        self.roles[0].loop_enable = True
        self.roles[0].behavior.add_child(
                TacticKeep('TacticKeep', self.roles[0].my_role, keep_x = keep_x,
                    range_high = constants.GoalHalfSize,
                    range_low = -constants.GoalHalfSize)
                )

        keep_x = constants.FieldHalfX - constants.DefenceLength - 0.5
        self.roles[1].loop_enable = True
        self.roles[1].behavior.add_child(
                TacticKeep('TacticKeep', self.roles[1].my_role, keep_x = keep_x)
                )

        self.roles[2].loop_enable = True
        self.roles[2].behavior.add_child(
                TacticInterpose('TacticInterpose', self.roles[2].my_role,
                    to_dist = 1.5)
                )

        range_y = constants.FieldHalfY - 0.7
        self.roles[3].loop_enable = True
        self.roles[3].behavior.add_child(
                TacticKeep('TacticKeep', self.roles[3].my_role, keep_x = -2.0,
                    range_high = range_y,
                    range_low = 0.5)
                )

        self.roles[4].loop_enable = True
        self.roles[4].behavior.add_child(
                TacticKeep('TacticKeep', self.roles[4].my_role, keep_x = -2.0,
                    range_high = -0.5,
                    range_low = -range_y)
                )

        pose1 = Pose(-2.5, range_y, 0)
        pose2 = Pose(-2.5, -range_y, 0)
        self.roles[5].loop_enable = True
        self.roles[5].behavior.add_child(
                TacticIntersection('TacticIntersection', self.roles[5].my_role,
                    pose1 = pose1, pose2 = pose2)
                )

