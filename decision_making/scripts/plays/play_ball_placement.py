
from play_stop import PlayStop

from tactics.tactic_placement import TacticPlacement
from tactics.tactic_interpose import TacticInterpose

class PlayBallPlacement(PlayStop):
    def __init__(self):
        super(PlayBallPlacement, self).__init__('PlayBallPlacement')

        self.applicable = "OUR_BALL_PLACEMENT"
        self.done_aborted = "OUR_BALL_PLACEMENT"
        self.assignment_type = "CLOSEST_BALL"
        self.formation_type = None

        self.roles[1].clear_behavior()
        self.roles[1].behavior.add_child(
                TacticPlacement('TacticPlacement', self.roles[1].my_role)
                )

        self.roles[2].clear_behavior()
        self.roles[2].behavior.add_child(
                TacticInterpose('TacticInterpose', self.roles[2].my_role,
                    base="DesignatedPosition", target="Ball",
                    to_dist=-0.3)
                )
