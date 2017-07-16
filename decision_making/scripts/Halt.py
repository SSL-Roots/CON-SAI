from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from CheckRefboxCommand import IsHALT
import BaseSkill


class Halt(ParallelAll):
    def __init__(self, name):
        super(Halt , self).__init__(name)

        IS_HALT = IsHALT("IS_HALT")
        HALT = BaseSkill.AllRobots_Halt("AllRobots_Halt")

        self.add_child(IS_HALT)
        self.add_child(HALT)
