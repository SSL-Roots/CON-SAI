
from play_base import Play

from tactics.tactic_position import TacticPosition
from tactics.tactic_interpose import TacticInterpose

class PlayStop(Play):
    def __init__(self):
        super(PlayStop, self).__init__('PlayStop')

        self.applicalbe = "STOP"
        self.done_aborted = "STOP"

        for i in range(6):
            x = -1.0
            y = 2.0 - 0.4 * i
            yaw = 0

            # to_dist = 0.4 + 0.4 * i
            from_dist = 0.3 + 0.3 * i

            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    # TacticPosition('TacticPosition', self.roles[i].my_role, x, y, yaw))
                    TacticInterpose('TacticInterpose', self.roles[i].my_role, from_dist=from_dist))
