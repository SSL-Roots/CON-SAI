
from play_stop import PlayStop

from tactics.tactic_inplay_shoot import TacticInplayShoot

class PlayForceStart(PlayStop):
    def __init__(self):
        super(PlayForceStart, self).__init__('PlayForceStart')

        self.applicable = "FORCE_START"
        self.done_aborted = "FORCE_START"

        self.roles[1].clear_behavior()
        self.roles[1].behavior.add_child(
                TacticInplayShoot('TacticInplayShoot', self.roles[1].my_role)
                )

