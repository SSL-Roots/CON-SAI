
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *


from skills.static_drive import SendVelocity
from skills.static_drive import SendCommand

class TacticCommand(ParallelAll):
    def __init__(self, name, my_role):
        super(TacticCommand, self).__init__(name)

        self.add_child(SendCommand('SendCommand', my_role))

