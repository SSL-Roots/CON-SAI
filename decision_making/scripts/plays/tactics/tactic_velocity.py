
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *


from skills.static_drive import SendVelocity

class TacticVelocity(ParallelAll):
    def __init__(self, name, my_role, vel_x, vel_y, vel_yaw):
        super(TacticVelocity, self).__init__(name)

        self.add_child(SendVelocity('SendVelocity', my_role, vel_x, vel_y, vel_yaw))

