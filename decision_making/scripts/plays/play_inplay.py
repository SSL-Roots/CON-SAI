
from play_stop import PlayStop

from tactics.tactic_attacker import TacticAttacker

class PlayInPlay(PlayStop):
    def __init__(self):
        super(PlayInPlay, self).__init__('PlayInPlay')

        self.applicable = "IN_PLAY"
        self.done_aborted = "IN_PLAY"
        self.assignment_type = "CLOSEST_BALL"

        self.roles[1].clear_behavior()
        self.roles[1].behavior.add_child(
                TacticAttacker('TacticAttacker', self.roles[1].my_role)
                )

