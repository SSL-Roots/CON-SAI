
from play_stop import PlayStop

from tactics.tactic_setplay_shoot import TacticSetplayShoot

class PlayIndirect(PlayStop):
    def __init__(self):
        super(PlayIndirect, self).__init__('PlayIndirect')

        self.applicable = "OUR_INDIRECT"
        self.done_aborted = "OUR_INDIRECT"


        self.roles[1].clear_behavior()
        self.roles[1].behavior.add_child(
                TacticSetplayShoot('TacticSetplayShoot', self.roles[1].my_role)
                )

