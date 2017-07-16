from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from CheckRefboxCommand import IsTIMEOUT_ENEMY
import BaseSkill

class TimeoutEnemy(ParallelAll):
    def __init__(self, name):
        super(TimeoutEnemy , self).__init__(name)

        IS_TIMEOUT_ENEMY = IsTIMEOUT_ENEMY('is timeout enemy')
        HALT = BaseSkill.AllRobots_Halt("AllRobots_Halt")

        self.add_child(IS_TIMEOUT_ENEMY)
        self.add_child(HALT)
