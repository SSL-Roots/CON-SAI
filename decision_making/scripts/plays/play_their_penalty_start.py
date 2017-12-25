
from play_base import Play

from tactics.tactic_keep import TacticKeep
from tactics.tactic_look_intersection import TacticLookIntersection
from tactics.tactic_position import TacticPosition
from consai_msgs.msg import Pose
import constants
import math

class PlayTheirPenaltyStart(Play):
    def __init__(self):
        super(PlayTheirPenaltyStart, self).__init__('PlayTheirPenaltyStart')

        self.applicable = "THEIR_PENALTY_START"
        self.done_aborted = "THEIR_PENALTY_START"

        pose_x = -constants.FieldHalfX + 0.05
        pose_y = constants.GoalHalfSize - constants.RobotRadius

        pose1 = Pose(pose_x, pose_y, 0)
        pose2 = Pose(pose_x, -pose_y, 0)
        self.roles[0].loop_enable = True
        self.roles[0].behavior.add_child(
                TacticLookIntersection('TacticLookIntersection',
                    self.roles[0].my_role, target='Threat_0',
                    pose1 = pose1, pose2= pose2)
                )

        for i in range(1,6):
            x = -3.0 + 0.3*i
            y = 2.0
            theta = -math.pi * 0.5

            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticPosition('TacticPosition', self.roles[i].my_role,
                        x, y ,theta))
