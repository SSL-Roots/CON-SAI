from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from CheckRefboxCommand import IsGOAL_FRIEND
import BaseSkill


class GoalFriend(ParallelAll):
    def __init__(self, name):
        super(GoalFriend , self).__init__(name)

        IS_GOAL_FRIEND = IsGOAL_FRIEND('is Goal friend')
        HALT = BaseSkill.AllRobots_Halt("AllRobots_Halt")

        self.add_child(IS_GOAL_FRIEND)
        self.add_child(HALT)
