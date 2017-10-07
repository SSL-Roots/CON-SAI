
from play_base import Play

from tactics.stop import Stop

class PlayDummy(Play):
    def __init__(self):
        super(PlayDummy, self).__init__('PlayDummy')

        for i in range(6):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    Stop("Stop", self.roles[i].my_role))
