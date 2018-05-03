
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *


from skills.static_drive import SendVelocity
from skills.static_drive import SendCommand
from skills.static_drive import SendTrapezoidalCommand

class TacticCommand(ParallelAll):
    def __init__(self, name, my_role):
        super(TacticCommand, self).__init__(name)

        self.add_child(SendCommand('SendCommand', my_role))

class TacticTrapezoidalCommand(ParallelAll):
    def __init__(self, name, my_role, is_vel_x, is_vel_y, is_vel_yaw):
        super(TacticTrapezoidalCommand, self).__init__(name)

        self.add_child(SendTrapezoidalCommand(
            'TacticTrapezoidalCommand', my_role, is_vel_x, is_vel_y, is_vel_yaw))
