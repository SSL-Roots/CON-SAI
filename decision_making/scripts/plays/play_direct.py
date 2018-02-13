
from play_stop import PlayStop

from tactics.tactic_setplay_shoot import TacticSetplayShoot

class PlayDirect(PlayStop):
    def __init__(self):
        super(PlayDirect, self).__init__('PlayDirect')

        self.applicable = 'OUR_DIRECT'
        self.done_aborted = 'OUR_DIRECT'

        self.roles[1].clear_behavior()
        self.roles[1].behavior.add_child(
                TacticSetplayShoot('TacticSetplayShoot', self.roles[1].my_role)
                )

