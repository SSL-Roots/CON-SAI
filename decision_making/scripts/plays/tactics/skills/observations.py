
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import sys,os
sys.path.append(os.pardir)
from world_model import WorldModel


class BallKicked(Task):
    def __init__(self, name):
        super(BallKicked, self).__init__(name)

    
    def run(self):
        ball_velocity = WorldModel.get_velocity('Ball')
        if WorldModel._observer.ball_has_kicked(ball_velocity):
            return TaskStatus.SUCCESS

        return TaskStatus.RUNNING
