
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *


from skills.turn_off import TurnOff

class Stop(ParallelAll):
    def __init__(self, name, my_role):
        super(Stop, self).__init__(name)

        self.add_child(TurnOff('TurnOff', my_role))

