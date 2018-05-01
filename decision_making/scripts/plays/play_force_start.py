
from play_stop import PlayStop

from tactics.tactic_attacker import TacticAttacker

class PlayForceStart(PlayStop):
    def __init__(self):
        super(PlayForceStart, self).__init__('PlayForceStart')

        self.applicable = "FORCE_START"
        self.done_aborted = "FORCE_START"
        self.formation_type = None

        self.roles[1].clear_behavior()
        self.roles[1].behavior.add_child(
                TacticAttacker('TacticAttacker', self.roles[1].my_role)
                )

