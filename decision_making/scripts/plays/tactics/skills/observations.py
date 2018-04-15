
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import sys,os
sys.path.append(os.pardir)
from world_model import WorldModel


class BallKicked(Task):
    def __init__(self, name):
        super(BallKicked, self).__init__(name)

    
    def run(self):
        if WorldModel.ball_kicked():
            return TaskStatus.SUCCESS

        return TaskStatus.RUNNING

class CanReceive(Task):
    def __init__(self, name, my_role):
        super(CanReceive, self).__init__(name)

        self._my_role = my_role

    
    def run(self):
        if WorldModel.can_receive(self._my_role):
            return TaskStatus.SUCCESS
        
        return TaskStatus.FAILURE

class CanShoot(Task):
    def __init__(self, name, my_role):
        super(CanShoot, self).__init__(name)

        self._my_role = my_role

    def run(self):
        if WorldModel.can_shoot():
            return TaskStatus.SUCCESS

        return TaskStatus.FAILURE

class CanPass(Task):
    def __init__(self, name, my_role):
        super(CanPass, self).__init__(name)

        self._my_role = my_role

    def run(self):
        if WorldModel.can_pass(self._my_role):
            return TaskStatus.SUCCESS

        return TaskStatus.FAILURE
