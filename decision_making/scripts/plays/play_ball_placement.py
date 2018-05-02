
from play_stop import PlayStop

from tactics.tactic_halt import TacticHalt

class PlayBallPlacement(PlayStop):
    def __init__(self):
        super(PlayBallPlacement, self).__init__('PlayBallPlacement')

        self.applicable = "OUR_BALL_PLACEMENT"
        self.done_aborted = "OUR_BALL_PLACEMENT"
        self.assignment_type = "CLOSEST_BALL"
        self.formation_type = None

        self.roles[1].clear_behavior()
        self.roles[1].behavior.add_child(
                TacticHalt('TacticHalt', self.roles[1].my_role)
                )

