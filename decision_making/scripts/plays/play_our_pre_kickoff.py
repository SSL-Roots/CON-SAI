
from play_stop import PlayStop

from tactics.tactic_interpose import TacticInterpose

class PlayOurPreKickoff(PlayStop):
    def __init__(self):
        super(PlayOurPreKickoff, self).__init__('PlayOurPreKickoff')

        self.applicable = "OUR_PRE_KICKOFF"
        self.done_aborted = "OUR_PRE_KICKOFF"

        self.roles[1].clear_behavior()
        self.roles[1].behavior.add_child(
                TacticInterpose('TacticInterpose', self.roles[1].my_role, 
                    from_dist = 0.2)
                )
