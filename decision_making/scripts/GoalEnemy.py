from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from CheckRefboxCommand import IsGOAL_ENEMY
import BaseSkill


class GoalEnemy(ParallelAll):
    def __init__(self, name):
        super(GoalEnemy , self).__init__(name)

        IS_GOAL_ENEMY = IsGOAL_ENEMY('is Goal enemy')
        HALT = BaseSkill.AllRobots_Halt("AllRobots_Halt")

        self.add_child(IS_GOAL_ENEMY)
        self.add_child(HALT)
