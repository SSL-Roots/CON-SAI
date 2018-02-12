
from play_stop import PlayStop

from tactics.tactic_keep import TacticKeep
import constants

class PlayInPlayTheirDefence(PlayStop):
    def __init__(self):
        super(PlayInPlayTheirDefence, self).__init__('PlayInPlayTheirDefence')

        self.applicable = "BALL_IN_THEIR_DEFENCE"
        self.done_aborted = "BALL_IN_THEIR_DEFENCE"
        self.assignment_type = "CLOSEST_BALL"

        keep_x = constants.FieldHalfX - constants.DefenceLength - 0.5
        self.roles[1].clear_behavior()
        self.roles[1].behavior.add_child(
                TacticKeep('TacticKeep', self.roles[1].my_role, keep_x = keep_x)
                )

