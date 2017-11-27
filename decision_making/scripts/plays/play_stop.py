
from play_base import Play

from tactics.tactic_position import TacticPosition
from tactics.tactic_interpose import TacticInterpose
from tactics.tactic_mark import TacticMark

class PlayStop(Play):
    def __init__(self):
        super(PlayStop, self).__init__('PlayStop')

        self.applicable = "STOP"
        self.done_aborted = "STOP"

        self.roles[0].loop_enable = True
        self.roles[0].behavior.add_child(
                TacticMark('TacticMark', self.roles[0].my_role, target='Enemy_Goalie'))

        for i in range(1,6):
            target_name = 'Enemy_' + str(i)

            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticMark('TacticMark', self.roles[i].my_role, target=target_name))

