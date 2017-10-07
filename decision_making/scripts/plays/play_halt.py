
from play_base import Play

from tactics.stop import Stop

class PlayHalt(Play):
    def __init__(self):
        super(PlayHalt, self).__init__('PlayHalt')

        self.applicalbe = "HALT"
        self.done_aborted = "HALT"

        for i in range(6):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    Stop("Stop", self.roles[i].my_role))
