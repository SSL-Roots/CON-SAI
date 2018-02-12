
from play_stop import PlayStop

from tactics.tactic_keep import TacticKeep
from tactics.tactic_intersection import TacticIntersection
from tactics.tactic_interpose import TacticInterpose
from tactics.tactic_setplay_shoot import TacticSetplayShoot
from consai_msgs.msg import Pose
import constants

class PlayOurKickoffStart(PlayStop):
    def __init__(self):
        super(PlayOurKickoffStart, self).__init__('PlayOurKickoffStart')

        self.applicable = "OUR_KICKOFF_START"
        self.done_aborted = "OUR_KICKOFF_START"

        self.roles[1].clear_behavior()
        self.roles[1].behavior.add_child(
                TacticSetplayShoot('TacticSetplayShoot', self.roles[1].my_role)
                )
